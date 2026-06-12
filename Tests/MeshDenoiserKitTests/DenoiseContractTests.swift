import simd
import XCTest
@testable import MeshDenoiserKit

final class DenoiseContractTests: XCTestCase {

    /// Octahedron with deterministic per-vertex noise; closed manifold, 8 faces.
    static func noisyOctahedron() -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        let base: [SIMD3<Float>] = [
            [1, 0, 0], [-1, 0, 0],
            [0, 1, 0], [0, -1, 0],
            [0, 0, 1], [0, 0, -1],
        ]
        // Fixed offsets stand in for noise; keeps the test deterministic.
        let noise: [SIMD3<Float>] = [
            [0.03, -0.02, 0.01], [-0.01, 0.04, -0.03],
            [0.02, 0.01, -0.04], [-0.03, -0.01, 0.02],
            [0.01, -0.04, 0.03], [-0.02, 0.03, -0.01],
        ]
        let positions = zip(base, noise).map { $0 + $1 }
        let indices: [UInt32] = [
            0, 2, 4,  2, 1, 4,  1, 3, 4,  3, 0, 4,
            2, 0, 5,  1, 2, 5,  3, 1, 5,  0, 3, 5,
        ]
        return (positions, indices)
    }

    func testDenoisePreservesVertexCountAndMovesVertices() async throws {
        let (positions, indices) = Self.noisyOctahedron()
        let result = try await MeshDenoiser.denoise(positions: positions, indices: indices)

        XCTAssertEqual(result.count, positions.count)
        for p in result {
            XCTAssertTrue(p.x.isFinite && p.y.isFinite && p.z.isFinite)
        }
        // Denoising must actually move something...
        let maxDisplacement = zip(result, positions).map { length($0 - $1) }.max()!
        XCTAssertGreaterThan(maxDisplacement, 0)
        // ...but stay close to the input (closeness term).
        XCTAssertLessThan(maxDisplacement, 1.0)
    }
}
