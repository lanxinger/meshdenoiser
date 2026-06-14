import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class GuidanceNormalsTests: XCTestCase {

    func testFlatMeshGuidanceMatchesFaceNormals() throws {
        let positions: [SIMD3<Float>] = [
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
        ]
        let mesh = try MeshValidation.makeMesh(positions: positions, indices: [0, 1, 2, 0, 2, 3])
        let connectivity = try MeshConnectivity.build(mesh: mesh)

        let guidance = GuidanceNormals.compute(connectivity: connectivity)

        XCTAssertEqual(guidance.count, connectivity.faceGeometry.normals.count)
        for (got, want) in zip(guidance, connectivity.faceGeometry.normals) {
            XCTAssertEqual(got.x, want.x, accuracy: 1e-6)
            XCTAssertEqual(got.y, want.y, accuracy: 1e-6)
            XCTAssertEqual(got.z, want.z, accuracy: 1e-6)
        }
    }

    func testOctahedronGuidanceReturnsUnitNormalPerFace() throws {
        let meshData = DenoiseContractTests.noisyOctahedron()
        let mesh = try MeshValidation.makeMesh(positions: meshData.positions, indices: meshData.indices)
        let connectivity = try MeshConnectivity.build(mesh: mesh)

        let guidance = GuidanceNormals.compute(connectivity: connectivity)

        XCTAssertEqual(guidance.count, connectivity.faceGeometry.normals.count)
        for normal in guidance {
            XCTAssertEqual(length(normal), 1, accuracy: 1e-5)
        }
    }
}
