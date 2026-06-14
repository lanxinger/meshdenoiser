import os
import simd
import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class NativeParityTests: XCTestCase {
    func testNativeCPUProducesFiniteMovedOutput() async throws {
        let mesh = DenoiseContractTests.noisyOctahedron()
        var params = MeshDenoiseParameters()
        params.backend = .nativeCPU

        let result = try await MeshDenoiser.denoise(
            positions: mesh.positions,
            indices: mesh.indices,
            parameters: params
        )

        XCTAssertEqual(result.count, mesh.positions.count)
        let maxDisplacement = zip(result, mesh.positions).map { simd.length($0 - $1) }.max()!
        XCTAssertGreaterThan(maxDisplacement, 0)
        XCTAssertLessThan(maxDisplacement, 1)
        for position in result {
            XCTAssertTrue(position.x.isFinite && position.y.isFinite && position.z.isFinite)
        }
    }

    func testNativeCPUReportsProgress() async throws {
        let mesh = DenoiseContractTests.noisyOctahedron()
        var params = MeshDenoiseParameters()
        params.backend = .nativeCPU
        params.outerIterations = 3

        let recorded = OSAllocatedUnfairLock(initialState: [Double]())
        _ = try await MeshDenoiser.denoise(
            positions: mesh.positions,
            indices: mesh.indices,
            parameters: params
        ) { progress in
            recorded.withLock { $0.append(progress) }
        }

        let progressValues = recorded.withLock { $0 }
        XCTAssertEqual(progressValues.count, 12)
        XCTAssertEqual(progressValues.first, 1.0 / 12.0)
        XCTAssertEqual(progressValues.last, 1.0)
        XCTAssertTrue(zip(progressValues, progressValues.dropFirst()).allSatisfy { $0 <= $1 })
    }

    func testNativeCPUCancellationThrowsCancellationError() async {
        let mesh = DenoiseContractTests.noisyOctahedron()
        var params = MeshDenoiseParameters()
        params.backend = .nativeCPU
        params.outerIterations = 10

        let task = Task {
            try await MeshDenoiser.denoise(
                positions: mesh.positions,
                indices: mesh.indices,
                parameters: params
            )
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

    func testNativeCPUPreservesCoincidentSeamVertices() async throws {
        let positions: [SIMD3<Float>] = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [1, 1, 0],
            [1, 0, 0],
            [2, 0, 0.4],
            [1, 1, 0],
            [2, 1, 0],
        ]
        let indices: [UInt32] = [
            0, 1, 2,
            1, 3, 2,
            4, 5, 6,
            5, 7, 6,
        ]
        var params = MeshDenoiseParameters()
        params.backend = .nativeCPU
        params.meshUpdateDisplacementEps = -1

        let result = try await MeshDenoiser.denoise(
            positions: positions,
            indices: indices,
            parameters: params
        )

        XCTAssertLessThan(simd.length(result[1] - result[4]), 1e-5)
        XCTAssertLessThan(simd.length(result[3] - result[6]), 1e-5)
    }

    func testNativeCPUStaysCloseToReferenceOnFixture() async throws {
        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let input = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        let positions = input.positions.map { SIMD3<Float>(Float($0.x), Float($0.y), Float($0.z)) }

        var referenceParams = MeshDenoiseParameters()
        referenceParams.backend = .reference
        referenceParams.deterministic = true
        let reference = try await MeshDenoiser.denoise(
            positions: positions,
            indices: input.indices,
            parameters: referenceParams
        )

        var nativeParams = referenceParams
        nativeParams.backend = .nativeCPU
        let native = try await MeshDenoiser.denoise(
            positions: positions,
            indices: input.indices,
            parameters: nativeParams
        )

        var maxError = 0.0
        var sumError = 0.0
        for (got, want) in zip(native, reference) {
            let error = Double(simd.length(got - want))
            maxError = max(maxError, error)
            sumError += error
        }
        let meanError = sumError / Double(native.count)

        XCTAssertLessThan(maxError, 2e-3, "max native/reference error: \(maxError), mean: \(meanError)")
        XCTAssertLessThan(meanError, 2e-4, "max native/reference error: \(maxError), mean: \(meanError)")
    }

    func testNativeGPUStaysCloseToNativeCPUWhenMetalIsAvailable() async throws {
        guard NativeDenoiser.isGPUAvailable else {
            throw XCTSkip("Metal is not available on this host")
        }

        let mesh = DenoiseContractTests.noisyOctahedron()
        var cpuParams = MeshDenoiseParameters()
        cpuParams.backend = .nativeCPU
        let cpu = try await MeshDenoiser.denoise(
            positions: mesh.positions,
            indices: mesh.indices,
            parameters: cpuParams
        )

        var gpuParams = cpuParams
        gpuParams.backend = .nativeGPU
        let gpu = try await MeshDenoiser.denoise(
            positions: mesh.positions,
            indices: mesh.indices,
            parameters: gpuParams
        )

        XCTAssertEqual(gpu.count, cpu.count)
        for (got, want) in zip(gpu, cpu) {
            XCTAssertEqual(got.x, want.x, accuracy: 2e-5)
            XCTAssertEqual(got.y, want.y, accuracy: 2e-5)
            XCTAssertEqual(got.z, want.z, accuracy: 2e-5)
        }
    }

    func testNativeGPUStaysCloseToNativeCPUOnFixtureWhenMetalIsAvailable() async throws {
        guard NativeDenoiser.isGPUAvailable else {
            throw XCTSkip("Metal is not available on this host")
        }

        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let input = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        let positions = input.positions.map { SIMD3<Float>(Float($0.x), Float($0.y), Float($0.z)) }

        var cpuParams = MeshDenoiseParameters()
        cpuParams.backend = .nativeCPU
        let cpu = try await MeshDenoiser.denoise(
            positions: positions,
            indices: input.indices,
            parameters: cpuParams
        )

        var gpuParams = cpuParams
        gpuParams.backend = .nativeGPU
        let gpu = try await MeshDenoiser.denoise(
            positions: positions,
            indices: input.indices,
            parameters: gpuParams
        )

        var maxError: Float = 0
        for (got, want) in zip(gpu, cpu) {
            maxError = max(maxError, simd.length(got - want))
        }

        XCTAssertLessThan(maxError, 2e-3)
    }
}
