import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class MeshConnectivityTests: XCTestCase {

    func testOctahedronConnectivityCounts() throws {
        let meshData = DenoiseContractTests.noisyOctahedron()
        let mesh = try MeshValidation.makeMesh(positions: meshData.positions, indices: meshData.indices)
        let connectivity = try MeshConnectivity.build(mesh: mesh)

        XCTAssertEqual(connectivity.faceGeometry.normals.count, 8)
        XCTAssertEqual(connectivity.faceGeometry.centroids.count, 8)
        XCTAssertEqual(connectivity.faceGeometry.areaWeights.count, 8)
        XCTAssertEqual(connectivity.edges.count, 12)
        XCTAssertEqual(connectivity.vertexToFaces.offsets.count, mesh.positions.count + 1)
        XCTAssertEqual(connectivity.vertexToNonboundaryEdges.offsets.count, mesh.positions.count + 1)
        XCTAssertGreaterThan(connectivity.averageNeighborFaceCentroidDistance, 0)
    }

    func testFaceGeometryHasUnitNormalsAndMeanAreaWeightOne() throws {
        let meshData = DenoiseContractTests.noisyOctahedron()
        let mesh = try MeshValidation.makeMesh(positions: meshData.positions, indices: meshData.indices)
        let geometry = try MeshConnectivity.faceGeometry(mesh: mesh)

        for normal in geometry.normals {
            XCTAssertEqual(length(normal), 1, accuracy: 1e-5)
        }

        let meanArea = geometry.areaWeights.reduce(0, +) / Float(geometry.areaWeights.count)
        XCTAssertEqual(meanArea, 1, accuracy: 1e-6)
    }

    func testNonManifoldEdgeThrowsInvalidInput() throws {
        let positions: [SIMD3<Float>] = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, -1],
        ]
        let indices: [UInt32] = [
            0, 1, 2,
            1, 0, 3,
            0, 1, 4,
        ]
        let mesh = try MeshValidation.makeMesh(positions: positions, indices: indices)

        XCTAssertThrowsError(try MeshConnectivity.build(mesh: mesh)) { error in
            XCTAssertEqual(error as? NativeDenoiseError, .invalidInput)
        }
    }
}
