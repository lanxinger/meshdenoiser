import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class VertexUpdateTests: XCTestCase {

    func testSystemMatrixSingleTriangleBlockMatchesMeanCenteringPlusCloseness() {
        let triangles = [SIMD3<UInt32>(0, 1, 2)]
        let matrix = VertexUpdate.systemMatrixForTesting(
            vertexCount: 3,
            triangles: triangles,
            closenessWeight: 0.25
        )

        XCTAssertEqual(matrix[0][0], 2.0 / 3.0 + 0.25, accuracy: 1e-12)
        XCTAssertEqual(matrix[1][1], 2.0 / 3.0 + 0.25, accuracy: 1e-12)
        XCTAssertEqual(matrix[2][2], 2.0 / 3.0 + 0.25, accuracy: 1e-12)
        XCTAssertEqual(matrix[0][1], -1.0 / 3.0, accuracy: 1e-12)
        XCTAssertEqual(matrix[0][2], -1.0 / 3.0, accuracy: 1e-12)
        XCTAssertEqual(matrix[1][2], -1.0 / 3.0, accuracy: 1e-12)
    }

    func testPlaneProjectionPreservesCenteredTriangleCentroid() {
        let centered = [
            SIMD3<Double>(-1, -1, 0.25),
            SIMD3<Double>(1, 0, -0.5),
            SIMD3<Double>(0, 1, 0.25),
        ]
        let projected = VertexUpdate.projectToTargetPlaneForTesting(
            centeredPositions: centered,
            targetNormal: SIMD3<Double>(0, 0, 1)
        )

        let centroid = projected.reduce(SIMD3<Double>(repeating: 0), +) / 3
        XCTAssertEqual(centroid.x, 0, accuracy: 1e-12)
        XCTAssertEqual(centroid.y, 0, accuracy: 1e-12)
        XCTAssertEqual(centroid.z, 0, accuracy: 1e-12)
        for point in projected {
            XCTAssertEqual(point.z, 0, accuracy: 1e-12)
        }
    }

    func testCompleteUpdatePassProducesFinitePositionsAndPreservesCount() throws {
        let meshData = DenoiseContractTests.noisyOctahedron()
        let mesh = try MeshValidation.makeMesh(positions: meshData.positions, indices: meshData.indices)
        let connectivity = try MeshConnectivity.build(mesh: mesh)
        var params = NativeDenoiseParameters()
        params.meshUpdateIterations = 2

        let updated = try VertexUpdate.run(
            mesh: mesh,
            targetNormals: connectivity.faceGeometry.normals,
            parameters: params
        )

        XCTAssertEqual(updated.count, mesh.positions.count)
        for position in updated {
            XCTAssertTrue(position.x.isFinite)
            XCTAssertTrue(position.y.isFinite)
            XCTAssertTrue(position.z.isFinite)
        }
    }
}
