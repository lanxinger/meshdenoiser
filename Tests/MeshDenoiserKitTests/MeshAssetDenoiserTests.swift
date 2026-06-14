#if os(macOS)
import Foundation
import ModelIO
import AppKit
import SceneKit
import SceneKit.ModelIO
import XCTest
@testable import MeshDenoiserKit

final class MeshAssetDenoiserTests: XCTestCase {

    func testPreviewReturnsDenoisedGeometryWithoutWritingAsset() async throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let inputURL = directory.appendingPathComponent("input.usdz")
        try writeUSDZ(asset: makeBoxAsset(), to: inputURL)

        var parameters = MeshDenoiseParameters()
        parameters.backend = .nativeCPU
        let options = MeshAssetDenoiseOptions(parameters: parameters)

        let preview = try await MeshAssetDenoiser.preview(inputURL: inputURL, options: options)

        XCTAssertEqual(preview.summary.meshesProcessed, 1)
        XCTAssertEqual(preview.meshes.count, 1)
        XCTAssertEqual(preview.meshes[0].originalPositions.count, preview.meshes[0].denoisedPositions.count)
        XCTAssertGreaterThan(preview.meshes[0].indices.count, 0)
        XCTAssertTrue(preview.meshes[0].denoisedPositions.allSatisfy { position in
            position.x.isFinite && position.y.isFinite && position.z.isFinite
        })
        XCTAssertEqual(
            try FileManager.default.contentsOfDirectory(atPath: directory.path).sorted(),
            ["input.usdz"]
        )
    }

    func testProcessesUSDAssetAndWritesOutputFile() async throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let inputURL = directory.appendingPathComponent("input.usdz")
        let outputURL = directory.appendingPathComponent("output.usdz")
        try writeUSDZ(asset: makeBoxAsset(), to: inputURL)

        var parameters = MeshDenoiseParameters()
        parameters.backend = .nativeCPU
        let options = MeshAssetDenoiseOptions(parameters: parameters)

        let summary = try await MeshAssetDenoiser.process(
            inputURL: inputURL,
            outputURL: outputURL,
            options: options
        )

        XCTAssertTrue(FileManager.default.fileExists(atPath: outputURL.path))
        XCTAssertEqual(summary.meshesProcessed, 1)
        XCTAssertGreaterThan(summary.verticesProcessed, 0)
        XCTAssertGreaterThan(summary.facesProcessed, 0)
    }

    func testURLProcessingPreservesUSDZTextureMaterials() async throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let inputURL = directory.appendingPathComponent("input.usdz")
        let outputURL = directory.appendingPathComponent("output.usdz")
        try writeTexturedUSDZ(to: inputURL, textureDirectory: directory)

        let inputTextureCount = textureMaterialPropertyCount(in: inputURL)
        XCTAssertGreaterThan(inputTextureCount, 0)

        var parameters = MeshDenoiseParameters()
        parameters.backend = .nativeCPU
        let options = MeshAssetDenoiseOptions(parameters: parameters)

        _ = try await MeshAssetDenoiser.process(
            inputURL: inputURL,
            outputURL: outputURL,
            options: options
        )

        XCTAssertEqual(textureMaterialPropertyCount(in: outputURL), inputTextureCount)
    }

    func testURLProcessingRejectsInPlaceOutput() async throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let inputURL = directory.appendingPathComponent("input.usdz")
        try writeUSDZ(asset: makeBoxAsset(), to: inputURL)

        var parameters = MeshDenoiseParameters()
        parameters.backend = .nativeCPU
        let options = MeshAssetDenoiseOptions(parameters: parameters)

        do {
            _ = try await MeshAssetDenoiser.process(
                inputURL: inputURL,
                outputURL: inputURL,
                options: options
            )
            XCTFail("Expected in-place output to be rejected")
        } catch let error as MeshAssetDenoiseError {
            XCTAssertEqual(error, .inputOutputURLsMustDiffer)
        } catch {
            XCTFail("Expected MeshAssetDenoiseError.inputOutputURLsMustDiffer, got \(error)")
        }
    }

    func testProcessesInMemoryAssetAndWritesOutputFile() async throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let outputURL = directory.appendingPathComponent("output.usdz")
        var parameters = MeshDenoiseParameters()
        parameters.backend = .nativeCPU
        let options = MeshAssetDenoiseOptions(parameters: parameters)

        let summary = try await MeshAssetDenoiser.process(
            asset: makeBoxAsset(),
            outputURL: outputURL,
            options: options
        )

        XCTAssertTrue(FileManager.default.fileExists(atPath: outputURL.path))
        XCTAssertEqual(summary.meshesProcessed, 1)
        XCTAssertGreaterThan(summary.verticesProcessed, 0)
        XCTAssertGreaterThan(summary.facesProcessed, 0)
    }

    func testProcessingReplacesExistingOutputFile() async throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let outputURL = directory.appendingPathComponent("output.usdz")
        try Data("existing".utf8).write(to: outputURL)

        var parameters = MeshDenoiseParameters()
        parameters.backend = .nativeCPU
        let options = MeshAssetDenoiseOptions(parameters: parameters)

        _ = try await MeshAssetDenoiser.process(
            asset: makeBoxAsset(),
            outputURL: outputURL,
            options: options
        )

        let exportedAsset = MDLAsset(url: outputURL)
        XCTAssertEqual(exportedAsset.childObjects(of: MDLMesh.self).count, 1)
    }

    func testAtomicExportWritesToTemporarySiblingBeforeReplacingOutput() throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let outputURL = directory.appendingPathComponent("output.usdz")
        try Data("old".utf8).write(to: outputURL)
        var exportURL: URL?

        try MeshAssetDenoiser.exportAtomically(to: outputURL) { temporaryURL in
            exportURL = temporaryURL
            XCTAssertEqual(temporaryURL.deletingLastPathComponent(), directory)
            XCTAssertNotEqual(temporaryURL, outputURL)
            XCTAssertEqual(try String(contentsOf: outputURL, encoding: .utf8), "old")
            try Data("new".utf8).write(to: temporaryURL)
        }

        XCTAssertEqual(try String(contentsOf: outputURL, encoding: .utf8), "new")
        if let exportURL {
            XCTAssertFalse(FileManager.default.fileExists(atPath: exportURL.path))
        } else {
            XCTFail("Expected export closure to receive a temporary URL")
        }
    }

    func testAtomicExportRemovesTemporaryFileWhenExportFails() throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let outputURL = directory.appendingPathComponent("output.usdz")
        try Data("old".utf8).write(to: outputURL)
        var exportURL: URL?

        XCTAssertThrowsError(try MeshAssetDenoiser.exportAtomically(to: outputURL) { temporaryURL in
            exportURL = temporaryURL
            try Data("partial".utf8).write(to: temporaryURL)
            throw MeshAssetDenoiseError.exportFailed(temporaryURL.path)
        })

        XCTAssertEqual(try String(contentsOf: outputURL, encoding: .utf8), "old")
        if let exportURL {
            XCTAssertFalse(FileManager.default.fileExists(atPath: exportURL.path))
        } else {
            XCTFail("Expected export closure to receive a temporary URL")
        }
    }

    func testProcessReportsProgressFromZeroToOne() async throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let outputURL = directory.appendingPathComponent("output.usdz")
        var parameters = MeshDenoiseParameters()
        parameters.backend = .nativeCPU
        let options = MeshAssetDenoiseOptions(parameters: parameters)
        let progressRecorder = ProgressRecorder()

        _ = try await MeshAssetDenoiser.process(
            asset: makeBoxAsset(),
            outputURL: outputURL,
            options: options
        ) { progress in
            progressRecorder.append(progress)
        }

        let progressValues = progressRecorder.values
        XCTAssertEqual(progressValues.first, 0)
        XCTAssertEqual(progressValues.last, 1)
        XCTAssertEqual(progressValues, progressValues.sorted())
    }

    func testConservativeValidationAllowsUnusedVerticesWithoutChangingAssetTopology() async throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let outputURL = directory.appendingPathComponent("output.usdz")
        var parameters = MeshDenoiseParameters()
        parameters.backend = .nativeCPU
        let options = MeshAssetDenoiseOptions(
            parameters: parameters,
            preprocessing: .conservativeValidation(.conservative())
        )

        let summary = try await MeshAssetDenoiser.process(
            asset: makeTetrahedronWithUnusedVertexAsset(),
            outputURL: outputURL,
            options: options
        )

        XCTAssertTrue(FileManager.default.fileExists(atPath: outputURL.path))
        XCTAssertEqual(summary.meshesProcessed, 1)
        XCTAssertEqual(summary.verticesProcessed, 5)
        XCTAssertEqual(summary.facesProcessed, 4)
        XCTAssertEqual(summary.preprocessingDiagnostics.first?.removedUnreferencedVertices, 1)
    }

    func testConservativeValidationAllowsCoincidentSeamVertices() throws {
        let options = MeshAssetDenoiseOptions(
            preprocessing: .conservativeValidation(.conservative())
        )

        let summary = try MeshAssetDenoiser.preflight(
            asset: makeSeamedQuadAsset(),
            options: options
        )

        XCTAssertEqual(summary.meshesProcessed, 1)
        XCTAssertEqual(summary.verticesProcessed, 6)
        XCTAssertEqual(summary.facesProcessed, 2)
        XCTAssertEqual(summary.preprocessingDiagnostics.first?.verticesBefore, 6)
        XCTAssertEqual(summary.preprocessingDiagnostics.first?.verticesAfter, 6)
        XCTAssertEqual(summary.preprocessingDiagnostics.first?.removedUnreferencedVertices, 0)
    }

    func testPreflightReportsProcessableInMemoryAssetWithoutExporting() throws {
        let asset = makeTetrahedronWithUnusedVertexAsset()
        let options = MeshAssetDenoiseOptions(
            preprocessing: .conservativeValidation(.conservative())
        )

        let summary = try MeshAssetDenoiser.preflight(asset: asset, options: options)

        XCTAssertEqual(summary.meshesProcessed, 1)
        XCTAssertEqual(summary.verticesProcessed, 5)
        XCTAssertEqual(summary.facesProcessed, 4)
        XCTAssertEqual(summary.preprocessingDiagnostics.first?.removedUnreferencedVertices, 1)
    }

    func testPreflightReportsProcessableAssetFromURL() throws {
        let directory = FileManager.default.temporaryDirectory
            .appendingPathComponent(UUID().uuidString, isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        defer { try? FileManager.default.removeItem(at: directory) }

        let inputURL = directory.appendingPathComponent("input.usdz")
        try writeUSDZ(asset: makeBoxAsset(), to: inputURL)

        let summary = try MeshAssetDenoiser.preflight(inputURL: inputURL)

        XCTAssertEqual(summary.meshesProcessed, 1)
        XCTAssertGreaterThan(summary.verticesProcessed, 0)
        XCTAssertGreaterThan(summary.facesProcessed, 0)
    }

    func testPreflightRejectsInvalidDenoiseParameters() {
        var parameters = MeshDenoiseParameters()
        parameters.lambda = -1
        let options = MeshAssetDenoiseOptions(parameters: parameters)

        XCTAssertThrowsError(try MeshAssetDenoiser.preflight(
            asset: makeBoxAsset(),
            options: options
        )) { error in
            XCTAssertEqual(error as? MeshDenoiseError, .invalidParameters)
        }
    }

    func testPreflightAllowsNonPositiveDisplacementEpsilonToDisableEarlyStop() throws {
        var parameters = MeshDenoiseParameters()
        parameters.meshUpdateDisplacementEps = -1
        let options = MeshAssetDenoiseOptions(parameters: parameters)

        let summary = try MeshAssetDenoiser.preflight(asset: makeBoxAsset(), options: options)

        XCTAssertEqual(summary.meshesProcessed, 1)
    }

    private func makeBoxAsset() -> MDLAsset {
        let mesh = MDLMesh(
            boxWithExtent: SIMD3<Float>(1, 1, 1),
            segments: SIMD3<UInt32>(1, 1, 1),
            inwardNormals: false,
            geometryType: .triangles,
            allocator: nil
        )
        let asset = MDLAsset()
        asset.add(mesh)
        return asset
    }

    private func makeTetrahedronWithUnusedVertexAsset() -> MDLAsset {
        let positions: [SIMD3<Float>] = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [2, 2, 2],
        ]
        let indices: [UInt32] = [
            0, 2, 1,
            0, 1, 3,
            1, 2, 3,
            2, 0, 3,
        ]

        let descriptor = MDLVertexDescriptor()
        descriptor.attributes[0] = MDLVertexAttribute(
            name: MDLVertexAttributePosition,
            format: .float3,
            offset: 0,
            bufferIndex: 0
        )
        descriptor.layouts[0] = MDLVertexBufferLayout(stride: MemoryLayout<SIMD3<Float>>.stride)

        let vertexData = positions.withUnsafeBufferPointer { Data(buffer: $0) }
        let indexData = indices.withUnsafeBufferPointer { Data(buffer: $0) }
        let vertexBuffer = MDLMeshBufferData(type: .vertex, data: vertexData)
        let indexBuffer = MDLMeshBufferData(type: .index, data: indexData)
        let submesh = MDLSubmesh(
            indexBuffer: indexBuffer,
            indexCount: indices.count,
            indexType: .uInt32,
            geometryType: .triangles,
            material: nil
        )
        let mesh = MDLMesh(
            vertexBuffer: vertexBuffer,
            vertexCount: positions.count,
            descriptor: descriptor,
            submeshes: [submesh]
        )

        let asset = MDLAsset()
        asset.add(mesh)
        return asset
    }

    private func makeSeamedQuadAsset() -> MDLAsset {
        let positions: [SIMD3<Float>] = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
        ]
        let indices: [UInt32] = [
            0, 1, 2,
            3, 4, 5,
        ]

        let descriptor = MDLVertexDescriptor()
        descriptor.attributes[0] = MDLVertexAttribute(
            name: MDLVertexAttributePosition,
            format: .float3,
            offset: 0,
            bufferIndex: 0
        )
        descriptor.layouts[0] = MDLVertexBufferLayout(stride: MemoryLayout<SIMD3<Float>>.stride)

        let vertexData = positions.withUnsafeBufferPointer { Data(buffer: $0) }
        let indexData = indices.withUnsafeBufferPointer { Data(buffer: $0) }
        let vertexBuffer = MDLMeshBufferData(type: .vertex, data: vertexData)
        let indexBuffer = MDLMeshBufferData(type: .index, data: indexData)
        let submesh = MDLSubmesh(
            indexBuffer: indexBuffer,
            indexCount: indices.count,
            indexType: .uInt32,
            geometryType: .triangles,
            material: nil
        )
        let mesh = MDLMesh(
            vertexBuffer: vertexBuffer,
            vertexCount: positions.count,
            descriptor: descriptor,
            submeshes: [submesh]
        )

        let asset = MDLAsset()
        asset.add(mesh)
        return asset
    }

    private func writeUSDZ(asset: MDLAsset, to url: URL) throws {
        let scene = SCNScene(mdlAsset: asset)
        guard scene.write(to: url, options: nil, delegate: nil, progressHandler: nil) else {
            throw NSError(domain: "MeshAssetDenoiserTests", code: 1)
        }
    }

    private func writeTexturedUSDZ(to url: URL, textureDirectory: URL) throws {
        let textureURL = textureDirectory.appendingPathComponent("albedo.png")
        let image = NSImage(size: NSSize(width: 2, height: 2))
        image.lockFocus()
        NSColor.red.setFill()
        NSRect(x: 0, y: 0, width: 2, height: 2).fill()
        image.unlockFocus()

        guard let tiffData = image.tiffRepresentation,
              let bitmap = NSBitmapImageRep(data: tiffData),
              let pngData = bitmap.representation(using: .png, properties: [:])
        else {
            throw NSError(domain: "MeshAssetDenoiserTests", code: 2)
        }
        try pngData.write(to: textureURL)

        let geometry: SCNGeometry = SCNBox(width: 1, height: 1, length: 1, chamferRadius: 0)
        let material = SCNMaterial()
        material.name = "Texture"
        material.diffuse.contents = textureURL
        geometry.materials = [material]

        let scene = SCNScene()
        scene.rootNode.addChildNode(SCNNode(geometry: geometry))
        guard scene.write(to: url, options: nil, delegate: nil, progressHandler: nil) else {
            throw NSError(domain: "MeshAssetDenoiserTests", code: 3)
        }
    }

    private func textureMaterialPropertyCount(in url: URL) -> Int {
        let asset = MDLAsset(url: url)
        asset.loadTextures()
        let semantics: [MDLMaterialSemantic] = [
            .baseColor,
            .tangentSpaceNormal,
            .roughness,
            .metallic,
            .ambientOcclusion,
        ]

        return asset.childObjects(of: MDLMesh.self)
            .compactMap { $0 as? MDLMesh }
            .flatMap { ($0.submeshes as? [MDLSubmesh]) ?? [] }
            .compactMap(\.material)
            .reduce(0) { count, material in
                count + semantics.reduce(0) { semanticCount, semantic in
                    semanticCount + material.properties(with: semantic).filter { $0.type == .texture }.count
                }
            }
    }
}

private final class ProgressRecorder: @unchecked Sendable {
    private let lock = NSLock()
    private var storage = [Double]()

    var values: [Double] {
        lock.lock()
        defer { lock.unlock() }
        return storage
    }

    func append(_ value: Double) {
        lock.lock()
        defer { lock.unlock() }
        storage.append(value)
    }
}
#endif
