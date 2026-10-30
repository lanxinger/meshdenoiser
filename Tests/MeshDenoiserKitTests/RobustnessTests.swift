import CMeshDenoiserCore
import XCTest
@testable import MeshDenoiserKit

final class RobustnessTests: XCTestCase {

    private static func fixture() throws -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let input = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        return (input.positions.map { SIMD3<Float>(Float($0.x), Float($0.y), Float($0.z)) }, input.indices)
    }

    /// Flat-shaded exports give every triangle its own vertices. Such a soup must
    /// denoise like the connected mesh instead of failing (no shared edges).
    func testTriangleSoupDenoisesLikeConnectedMesh() async throws {
        let (positions, indices) = try Self.fixture()
        let soupPositions = indices.map { positions[Int($0)] }
        let soupIndices = (0..<UInt32(indices.count)).map { $0 }

        for backend in [MeshDenoiseParameters.Backend.reference, .nativeCPU] {
            var params = MeshDenoiseParameters()
            params.backend = backend
            params.deterministic = true

            let connected = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: params)
            let soup = try await MeshDenoiser.denoise(
                positions: soupPositions,
                indices: soupIndices,
                parameters: params
            )

            XCTAssertEqual(soup.count, soupPositions.count)
            var maxError: Float = 0
            for (corner, vertex) in indices.enumerated() {
                let difference = soup[corner] - connected[Int(vertex)]
                maxError = max(maxError, abs(difference.x), abs(difference.y), abs(difference.z))
            }
            XCTAssertLessThan(maxError, 1e-5, "\(backend) soup output diverged from the connected mesh")
        }
    }

    func testIndexCountNotMultipleOfThreeThrowsInvalidInput() async {
        let (positions, indices) = try! Self.fixture()
        do {
            _ = try await MeshDenoiser.denoise(positions: positions, indices: Array(indices.dropLast()))
            XCTFail("Expected invalidInput")
        } catch {
            XCTAssertEqual(error as? MeshDenoiseError, .invalidInput)
        }
    }

    /// The cancel hook is polled between solver iterations, so a single outer
    /// iteration (the default) can still be interrupted.
    func testReferenceCancelHookStopsWithinOuterIteration() throws {
        let (positions, indices) = try Self.fixture()
        let flat = positions.flatMap { [$0.x, $0.y, $0.z] }
        var output = [Float](repeating: 0, count: flat.count)
        var params = md_params()
        md_params_init_defaults(&params)
        XCTAssertEqual(params.outer_iterations, 1)

        final class Counter { var polls = 0; var progressCalls = 0 }
        let counter = Counter()
        let cancel: md_cancel_fn = { ctx in
            let counter = Unmanaged<Counter>.fromOpaque(ctx!).takeUnretainedValue()
            counter.polls += 1
            return counter.polls >= 3
        }
        let progress: md_progress_fn = { _, _, ctx in
            Unmanaged<Counter>.fromOpaque(ctx!).takeUnretainedValue().progressCalls += 1
            return true
        }

        let status = withExtendedLifetime(counter) {
            md_denoise(flat, positions.count, indices, indices.count / 3, &params, &output,
                       progress, cancel, Unmanaged.passUnretained(counter).toOpaque())
        }

        XCTAssertEqual(status, MD_CANCELLED)
        XCTAssertEqual(counter.polls, 3)
        XCTAssertEqual(counter.progressCalls, 0, "cancelled before the outer iteration completed")
    }

    func testZeroExtentMeshThrowsInvalidInput() async {
        let positions = [SIMD3<Float>](repeating: [1, 2, 3], count: 3)
        do {
            var params = MeshDenoiseParameters()
            params.backend = .reference
            _ = try await MeshDenoiser.denoise(positions: positions, indices: [0, 1, 2], parameters: params)
            XCTFail("Expected invalidInput")
        } catch {
            XCTAssertEqual(error as? MeshDenoiseError, .invalidInput)
        }
    }

    #if os(macOS)
    /// ModelIO's `.float3` is 12 bytes; writing a 16-byte SIMD3<Float> would clobber
    /// the next attribute in interleaved layouts and run past packed buffers.
    func testFloat3AttributeAccessStaysWithinTwelveBytes() {
        let values: [SIMD3<Float>] = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

        // Interleaved position + normal (stride 24) with a sentinel in every normal.
        var interleaved = [Float](repeating: 7, count: values.count * 6)
        interleaved.withUnsafeMutableBytes { bytes in
            MeshAssetDenoiser.writeFloat3Attribute(values, to: bytes.baseAddress!, stride: 24)
            XCTAssertEqual(
                MeshAssetDenoiser.readFloat3Attribute(at: UnsafeRawPointer(bytes.baseAddress!), stride: 24, count: values.count),
                values
            )
        }
        for vertex in values.indices {
            XCTAssertEqual(Array(interleaved[(vertex * 6 + 3)..<(vertex * 6 + 6)]), [7, 7, 7])
        }

        // Tightly packed (stride 12) followed by a guard value past the end.
        var packed = [Float](repeating: 0, count: values.count * 3 + 1)
        packed[values.count * 3] = 42
        packed.withUnsafeMutableBytes { bytes in
            MeshAssetDenoiser.writeFloat3Attribute(values, to: bytes.baseAddress!, stride: 12)
        }
        XCTAssertEqual(Array(packed.prefix(values.count * 3)), [1, 2, 3, 4, 5, 6, 7, 8, 9])
        XCTAssertEqual(packed[values.count * 3], 42)
    }
    #endif
}
