import XCTest
@testable import MeshDenoiserKit

final class GoldenParityTests: XCTestCase {

    /// MeshDenoiserKit must reproduce the CLI's output on the same input with
    /// the same (default) parameters in deterministic mode.
    /// Regenerate fixtures with scripts/generate_test_fixture.py + the CLI
    /// (`MeshDenoiser <fixture> <golden> --deterministic`) whenever the
    /// algorithm intentionally changes.
    func testMatchesCLIGoldenOutput() async throws {
        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let input = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        let golden = try OBJLoader.load(fixtures.appendingPathComponent("golden_denoised.obj"))

        XCTAssertEqual(input.positions.count, 162)
        XCTAssertEqual(golden.positions.count, input.positions.count)
        XCTAssertEqual(golden.indices, input.indices)

        var params = MeshDenoiseParameters() // defaults == CLI built-in defaults
        params.deterministic = true

        let result = try await MeshDenoiser.denoise(
            positions: input.positions.map { SIMD3<Float>(Float($0.x), Float($0.y), Float($0.z)) },
            indices: input.indices,
            parameters: params
        )

        var maxError = 0.0
        for (got, want) in zip(result, golden.positions) {
            maxError = max(maxError, abs(Double(got.x) - want.x))
            maxError = max(maxError, abs(Double(got.y) - want.y))
            maxError = max(maxError, abs(Double(got.z) - want.z))
        }
        XCTAssertLessThan(maxError, 1e-6, "Kit output diverged from CLI golden output")
    }
}
