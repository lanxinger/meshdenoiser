import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class NeighborSearchTests: XCTestCase {

    func testUniformGridMatchesBruteForcePairs() throws {
        let meshData = DenoiseContractTests.noisyOctahedron()
        let mesh = try MeshValidation.makeMesh(positions: meshData.positions, indices: meshData.indices)
        let connectivity = try MeshConnectivity.build(mesh: mesh)
        let radius = 3 * 2.2 * connectivity.averageNeighborFaceCentroidDistance

        let gridPairs = try NeighborSearch.findPairs(
            centroids: connectivity.faceGeometry.centroids,
            radius: radius
        )
        let bruteForcePairs = bruteForcePairs(
            centroids: connectivity.faceGeometry.centroids,
            radius: radius
        )

        XCTAssertEqual(pairKeys(gridPairs), pairKeys(bruteForcePairs))
    }

    func testUniformGridCarriesSquaredDistance() throws {
        let pairs = try NeighborSearch.findPairs(
            centroids: [[0, 0, 0], [3, 4, 0]],
            radius: 6
        )

        XCTAssertEqual(pairs, [
            NeighborPair(face0: 0, face1: 1, distanceSquared: 25),
        ])
    }

    private func bruteForcePairs(centroids: [SIMD3<Float>], radius: Float) -> [NeighborPair] {
        let radiusSquared = radius * radius
        var pairs = [NeighborPair]()
        for face0 in 0..<centroids.count {
            for face1 in (face0 + 1)..<centroids.count {
                let diff = centroids[face0] - centroids[face1]
                let distanceSquared = lengthSquared(diff)
                if distanceSquared < radiusSquared {
                    pairs.append(NeighborPair(
                        face0: UInt32(face0),
                        face1: UInt32(face1),
                        distanceSquared: distanceSquared
                    ))
                }
            }
        }
        return pairs
    }

    private func pairKeys(_ pairs: [NeighborPair]) -> Set<String> {
        Set(pairs.map { "\($0.face0)-\($0.face1)" })
    }
}
