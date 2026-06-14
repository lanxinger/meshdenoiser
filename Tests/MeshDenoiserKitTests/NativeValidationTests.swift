import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class NativeValidationTests: XCTestCase {

    func testDefaultBackendIsReference() {
        XCTAssertEqual(MeshDenoiseParameters().backend, .reference)
    }

    func testNativeCPUPreservesVertexCountFromPublicAPI() async throws {
        let mesh = DenoiseContractTests.noisyOctahedron()
        var params = MeshDenoiseParameters()
        params.backend = .nativeCPU

        let result = try await MeshDenoiser.denoise(
            positions: mesh.positions,
            indices: mesh.indices,
            parameters: params
        )

        XCTAssertEqual(result.count, mesh.positions.count)
        for position in result {
            XCTAssertTrue(position.x.isFinite)
            XCTAssertTrue(position.y.isFinite)
            XCTAssertTrue(position.z.isFinite)
        }
    }

    func testNativeValidationRejectsEmptyBuffers() {
        XCTAssertThrowsError(try MeshValidation.makeMesh(positions: [], indices: [])) { error in
            XCTAssertEqual(error as? NativeDenoiseError, .invalidInput)
        }
    }

    func testNativeValidationRejectsIndexCountThatIsNotMultipleOfThree() {
        let positions: [SIMD3<Float>] = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
        XCTAssertThrowsError(try MeshValidation.makeMesh(positions: positions, indices: [0, 1])) { error in
            XCTAssertEqual(error as? NativeDenoiseError, .invalidInput)
        }
    }

    func testNativeValidationRejectsNaNPosition() {
        let positions: [SIMD3<Float>] = [[0, 0, 0], [1, .nan, 0], [0, 1, 0]]
        XCTAssertThrowsError(try MeshValidation.makeMesh(positions: positions, indices: [0, 1, 2])) { error in
            XCTAssertEqual(error as? NativeDenoiseError, .invalidInput)
        }
    }

    func testNativeValidationRejectsOutOfRangeIndex() {
        let positions: [SIMD3<Float>] = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
        XCTAssertThrowsError(try MeshValidation.makeMesh(positions: positions, indices: [0, 1, 3])) { error in
            XCTAssertEqual(error as? NativeDenoiseError, .invalidInput)
        }
    }

    func testNativeValidationRejectsDegenerateTriangle() {
        let positions: [SIMD3<Float>] = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
        XCTAssertThrowsError(try MeshValidation.makeMesh(positions: positions, indices: [0, 1, 1])) { error in
            XCTAssertEqual(error as? NativeDenoiseError, .invalidInput)
        }
    }

    func testNativeParametersValidateDocumentedRanges() {
        XCTAssertTrue(NativeDenoiseParameters().isValid)

        var params = NativeDenoiseParameters()
        params.lambda = 0
        XCTAssertFalse(params.isValid)

        params = NativeDenoiseParameters()
        params.meshUpdateDisplacementEps = -0.1
        XCTAssertTrue(params.isValid)
    }

    func testNormalizeRejectsZeroScaleMesh() {
        let positions = [SIMD3<Float>(1, 1, 1), SIMD3<Float>(1, 1, 1), SIMD3<Float>(1, 1, 1)]
        XCTAssertThrowsError(try MeshNormalization.normalize(positions)) { error in
            XCTAssertEqual(error as? NativeDenoiseError, .invalidInput)
        }
    }

    func testNormalizeRestoreRoundTripsPositions() throws {
        let positions: [SIMD3<Float>] = [
            [1, 2, 3],
            [4, 2, 3],
            [1, 6, 3],
            [1, 2, 8],
        ]

        let normalized = try MeshNormalization.normalize(positions)
        let restored = normalized.transform.restore(normalized.positions)

        for (got, want) in zip(restored, positions) {
            XCTAssertEqual(got.x, want.x, accuracy: 1e-6)
            XCTAssertEqual(got.y, want.y, accuracy: 1e-6)
            XCTAssertEqual(got.z, want.z, accuracy: 1e-6)
        }
    }

    private func assertThrows<E: Error & Equatable>(
        _ expected: E,
        file: StaticString = #filePath,
        line: UInt = #line,
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
