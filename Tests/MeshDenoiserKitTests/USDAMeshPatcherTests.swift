#if os(macOS)
import Foundation
import XCTest
@testable import MeshDenoiserKit

final class USDAMeshPatcherTests: XCTestCase {
    private typealias Patch = MeshAssetDenoiser.DenoisedMeshPatch

    private func patch(_ text: String, _ meshes: [Patch], recomputesNormals: Bool = true) throws -> String {
        let output = try USDAMeshPatcher.patch(Array(text.utf8), denoisedMeshes: meshes, recomputesNormals: recomputesNormals)
        return String(decoding: output, as: UTF8.self)
    }

    /// Two right triangles folded 90 degrees along their shared edge (points 0 and 2):
    /// face 0 lies in z = 0 (normal +z), face 1 in x = 0 (normal +x).
    private let foldPoints = "[(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]"
    private let foldFaces = "int[] faceVertexCounts = [3, 3]\n    int[] faceVertexIndices = [0, 1, 2, 0, 2, 3]"
    private var foldPatch: Patch {
        let points: [SIMD3<Float>] = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
        return Patch(originalPositions: points, denoisedPositions: points)
    }

    private func foldMesh(normals: String) -> String {
        "#usda 1.0\ndef Mesh \"Fold\"\n{\n    \(foldFaces)\n    \(normals)\n    point3f[] points = \(foldPoints)\n}\n"
    }

    private func float3Array(named name: String, in text: String) -> [SIMD3<Float>]? {
        let usda = USDAText(Array(text.utf8))
        guard let array = usda.arrayValue(named: name, in: 0..<text.utf8.count) else { return nil }
        return usda.parseFloat3Array(array)
    }

    private func assertNormals(
        _ actual: [SIMD3<Float>]?, _ expected: [SIMD3<Float>],
        file: StaticString = #filePath, line: UInt = #line
    ) {
        guard let actual, actual.count == expected.count else {
            return XCTFail("expected \(expected), got \(String(describing: actual))", file: file, line: line)
        }
        for (got, want) in zip(actual, expected) {
            let difference = got - want
            XCTAssertLessThan(max(abs(difference.x), abs(difference.y), abs(difference.z)), 1e-6,
                              "expected \(expected), got \(actual)", file: file, line: line)
        }
    }

    func testMeshesAreMatchedByPositionNotOrder() throws {
        let text = """
        #usda 1.0
        def Mesh "B"
        {
            point3f[] points = [(5, 0, 0), (6, 0, 0), (5, 1, 0)]
        }
        def BasisCurves "Curve"
        {
            point3f[] points = [(9, 9, 9), (8, 8, 8)]
        }
        def Mesh "A"
        {
            point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        }
        """
        let a = Patch(originalPositions: [[0, 0, 0], [1, 0, 0], [0, 1, 0]], denoisedPositions: [[0, 0, 0.5], [1, 0, 0.5], [0, 1, 0.5]])
        let b = Patch(originalPositions: [[5, 0, 0], [6, 0, 0], [5, 1, 0]], denoisedPositions: [[5, 0, 2], [6, 0, 2], [5, 1, 2]])
        let result = try patch(text, [a, b])
        XCTAssertTrue(result.contains("def Mesh \"B\"\n{\n    point3f[] points = [(5, 0, 2), (6, 0, 2), (5, 1, 2)]"))
        XCTAssertTrue(result.contains("def Mesh \"A\"\n{\n    point3f[] points = [(0, 0, 0.5), (1, 0, 0.5), (0, 1, 0.5)]"))
        XCTAssertTrue(result.contains("point3f[] points = [(9, 9, 9), (8, 8, 8)]"), "non-mesh points must be left alone")
    }

    func testUnlocatableMeshThrows() {
        let text = "#usda 1.0\ndef Mesh \"A\"\n{\n    point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]\n}\n"
        let other = Patch(originalPositions: [[3, 3, 3], [4, 3, 3], [3, 4, 3]], denoisedPositions: [[3, 3, 3], [4, 3, 3], [3, 4, 3]])
        XCTAssertThrowsError(try patch(text, [other])) { error in
            XCTAssertTrue(error is USDAMeshPatcher.UnmatchedMeshes)
        }
    }

    func testFaceVaryingPrimvarNormalsKeepHardEdgesAndSmoothSharedValues() throws {
        let faceVarying = "(\n        interpolation = \"faceVarying\"\n    )"
        let z = SIMD3<Float>(0, 0, 1), x = SIMD3<Float>(1, 0, 0)
        let d = Float(0.5).squareRoot(), diagonal = SIMD3<Float>(d, 0, d)

        // Hard edge: each face authored its own normal at the shared points 0 and 2.
        let hard = try patch(foldMesh(normals:
            "normal3f[] primvars:normals = [(1, 1, 1), (1, 1, 1), (1, 1, 1), (2, 2, 2), (2, 2, 2), (2, 2, 2)] \(faceVarying)"
        ), [foldPatch])
        assertNormals(float3Array(named: "normal3f[] primvars:normals", in: hard), [z, z, z, x, x, x])
        XCTAssertTrue(hard.contains("interpolation = \"faceVarying\""))

        // Smooth: corners at points 0 and 2 carry one authored value per point.
        let smooth = try patch(foldMesh(normals:
            "normal3f[] primvars:normals = [(4, 4, 4), (1, 1, 1), (3, 3, 3), (4, 4, 4), (3, 3, 3), (2, 2, 2)] \(faceVarying)"
        ), [foldPatch])
        assertNormals(float3Array(named: "normal3f[] primvars:normals", in: smooth), [diagonal, z, diagonal, diagonal, diagonal, x])
    }

    func testIndexedPrimvarNormalsRecomputeValuesAndKeepIndices() throws {
        let result = try patch(foldMesh(normals: """
            normal3f[] primvars:normals = [(9, 9, 9), (8, 8, 8)] (
                    interpolation = "faceVarying"
                )
                int[] primvars:normals:indices = [0, 0, 0, 1, 1, 1]
            """), [foldPatch])
        assertNormals(float3Array(named: "normal3f[] primvars:normals", in: result), [[0, 0, 1], [1, 0, 0]])
        XCTAssertTrue(result.contains("int[] primvars:normals:indices = [0, 0, 0, 1, 1, 1]"))
    }

    func testVertexNormalsSmoothAcrossTheFoldAndHonorRecomputeOff() throws {
        let normals = "normal3f[] normals = [(9, 9, 9), (8, 8, 8), (7, 7, 7), (6, 6, 6)]"
        let d = Float(0.5).squareRoot()
        let recomputed = try patch(foldMesh(normals: normals), [foldPatch])
        assertNormals(float3Array(named: "normal3f[] normals", in: recomputed), [[d, 0, d], [0, 0, 1], [d, 0, d], [1, 0, 0]])

        let untouched = try patch(foldMesh(normals: normals), [foldPatch], recomputesNormals: false)
        XCTAssertTrue(untouched.contains(normals))
    }

    func testRootLayerIsFirstArchiveEntryNotAlphabeticallyFirst() throws {
        let directory = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }
        try "#usda 1.0\n(\n    subLayers = [@./aaa_geometry.usda@]\n)\n".write(
            to: directory.appendingPathComponent("scene.usda"), atomically: true, encoding: .utf8)
        try "#usda 1.0\n".write(
            to: directory.appendingPathComponent("aaa_geometry.usda"), atomically: true, encoding: .utf8)
        let archive = directory.appendingPathComponent("asset.usdz")
        try MeshAssetDenoiser.runTool(
            "/usr/bin/usdzip",
            arguments: [archive.path, "scene.usda", "aaa_geometry.usda"],
            currentDirectoryURL: directory
        )

        XCTAssertEqual(try MeshAssetDenoiser.rootLayerPath(ofUSDZ: archive, extractedTo: directory), "scene.usda")
    }

    /// When the mesh lives in a sublayer, the root layer has nothing to patch; the
    /// exporter must patch the flattened stage instead of failing or skipping the mesh.
    func testUSDZWithMeshInSublayerIsPatchedThroughFlattenedStage() async throws {
        let directory = FileManager.default.temporaryDirectory.appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let mesh = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        let points = mesh.positions.map { "(\(Float($0.x)), \(Float($0.y)), \(Float($0.z)))" }.joined(separator: ", ")
        let counts = Array(repeating: "3", count: mesh.indices.count / 3).joined(separator: ", ")
        let indices = mesh.indices.map(String.init).joined(separator: ", ")
        try """
        #usda 1.0
        def Mesh "Sphere"
        {
            int[] faceVertexCounts = [\(counts)]
            int[] faceVertexIndices = [\(indices)]
            point3f[] points = [\(points)]
        }
        """.write(to: directory.appendingPathComponent("aaa_geometry.usda"), atomically: true, encoding: .utf8)
        try "#usda 1.0\n(\n    defaultPrim = \"Sphere\"\n    subLayers = [@./aaa_geometry.usda@]\n)\n".write(
            to: directory.appendingPathComponent("scene.usda"), atomically: true, encoding: .utf8)
        let input = directory.appendingPathComponent("input.usdz")
        try MeshAssetDenoiser.runTool(
            "/usr/bin/usdzip",
            arguments: [input.path, "scene.usda", "aaa_geometry.usda"],
            currentDirectoryURL: directory
        )

        let output = directory.appendingPathComponent("output.usdz")
        let summary = try await MeshAssetDenoiser.process(inputURL: input, outputURL: output)
        XCTAssertEqual(summary.meshesProcessed, 1)

        let flattened = directory.appendingPathComponent("check.usda")
        try MeshAssetDenoiser.runTool("/usr/bin/usdcat", arguments: ["--flatten", output.path, "-o", flattened.path])
        let text = try String(contentsOf: flattened, encoding: .utf8)
        let result = try XCTUnwrap(float3Array(named: "point3f[] points", in: text))
        XCTAssertEqual(result.count, mesh.positions.count)
        let maxMove = zip(result, mesh.positions).map { got, want in
            max(abs(got.x - Float(want.x)), abs(got.y - Float(want.y)), abs(got.z - Float(want.z)))
        }.max()!
        XCTAssertGreaterThan(maxMove, 1e-4, "points were not denoised")
    }
}
#endif
