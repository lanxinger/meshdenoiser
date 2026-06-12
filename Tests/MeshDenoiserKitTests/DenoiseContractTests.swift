import os
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

    func testNaNInputThrowsInvalidInput() async {
        var (positions, indices) = Self.noisyOctahedron()
        positions[2].y = .nan
        await assertThrows(MeshDenoiseError.invalidInput) {
            _ = try await MeshDenoiser.denoise(positions: positions, indices: indices)
        }
    }

    func testOutOfRangeIndexThrowsInvalidInput() async {
        var (positions, indices) = Self.noisyOctahedron()
        indices[5] = UInt32(positions.count) // one past the end
        await assertThrows(MeshDenoiseError.invalidInput) {
            _ = try await MeshDenoiser.denoise(positions: positions, indices: indices)
        }
    }

    func testEmptyInputThrowsInvalidInput() async {
        await assertThrows(MeshDenoiseError.invalidInput) {
            _ = try await MeshDenoiser.denoise(positions: [], indices: [])
        }
    }

    func testNegativeLambdaThrowsInvalidParameters() async {
        let (positions, indices) = Self.noisyOctahedron()
        var params = MeshDenoiseParameters()
        params.lambda = -1
        await assertThrows(MeshDenoiseError.invalidParameters) {
            _ = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: params)
        }
    }

    func testCancellationThrowsCancellationError() async {
        let (positions, indices) = Self.noisyOctahedron()
        var params = MeshDenoiseParameters()
        params.outerIterations = 50 // cancellation is checked after each outer iteration

        let task = Task {
            try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: params)
        }
        task.cancel()
        do {
            _ = try await task.value
            XCTFail("Expected CancellationError")
        } catch is CancellationError {
            // expected
        } catch {
            XCTFail("Expected CancellationError, got \(error)")
        }
    }

    func testProgressReportsMonotonicallyToOne() async throws {
        let (positions, indices) = Self.noisyOctahedron()
        var params = MeshDenoiseParameters()
        params.outerIterations = 3

        let recorded = OSAllocatedUnfairLock(initialState: [Double]())
        _ = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: params) { p in
            recorded.withLock { $0.append(p) }
        }
        let values = recorded.withLock { $0 }
        XCTAssertEqual(values, [1.0 / 3.0, 2.0 / 3.0, 1.0])
    }

    private func assertThrows<E: Error & Equatable>(
        _ expected: E,
        file: StaticString = #filePath, line: UInt = #line,
        _ body: () async throws -> Void
    ) async {
        do {
            try await body()
            XCTFail("Expected \(expected) to be thrown", file: file, line: line)
        } catch let error as E {
            XCTAssertEqual(error, expected, file: file, line: line)
        } catch {
            XCTFail("Expected \(expected), got \(error)", file: file, line: line)
        }
    }
}
