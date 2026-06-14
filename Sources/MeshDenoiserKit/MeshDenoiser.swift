import CMeshDenoiserCore
import Dispatch
import Foundation
import MeshDenoiserNative
import os

/// Bridges Task cancellation and progress callbacks across the C boundary.
private final class DenoiseSession: @unchecked Sendable {
    let progressHandler: (@Sendable (Double) -> Void)?
    private let cancelledLock = OSAllocatedUnfairLock(initialState: false)

    init(progressHandler: (@Sendable (Double) -> Void)?) {
        self.progressHandler = progressHandler
    }

    func cancel() { cancelledLock.withLock { $0 = true } }
    var isCancelled: Bool { cancelledLock.withLock { $0 } }
}

public enum MeshDenoiser {

    /// Denoises a triangle mesh, preserving vertex count and order.
    ///
    /// Runs the synchronous C++ filter on a global queue; supports Task cancellation
    /// (checked after each outer iteration) and reports fractional progress.
    /// - Returns: Denoised vertex positions, same count and order as `positions`.
    public static func denoise(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: MeshDenoiseParameters = MeshDenoiseParameters(),
        progress: (@Sendable (Double) -> Void)? = nil
    ) async throws -> [SIMD3<Float>] {
        let session = DenoiseSession(progressHandler: progress)
        let denoisedPositions = try await withTaskCancellationHandler {
            try await withCheckedThrowingContinuation { continuation in
                DispatchQueue.global(qos: .userInitiated).async {
                    continuation.resume(with: Result {
                        try runSync(positions: positions, indices: indices,
                                    parameters: parameters, session: session)
                    })
                }
            }
        } onCancel: {
            session.cancel()
        }
        try validateDenoisedPositions(denoisedPositions, expectedCount: positions.count)
        return denoisedPositions
    }

    private static func runSync(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: MeshDenoiseParameters,
        session: DenoiseSession
    ) throws -> [SIMD3<Float>] {
        guard parameters.isValid else {
            throw MeshDenoiseError.invalidParameters
        }

        switch parameters.backend {
        case .reference:
            return preserveCoincidentInputVertices(
                try runReferenceSync(
                    positions: positions,
                    indices: indices,
                    parameters: parameters,
                    session: session
                ),
                originalPositions: positions
            )
        case .nativeGPU where !NativeDenoiser.isGPUAvailable:
            throw MeshDenoiseError.gpuUnavailable
        case .automatic, .nativeGPU, .nativeCPU:
            let useGPU = parameters.backend != .nativeCPU && NativeDenoiser.isGPUAvailable
            var native = NativeDenoiseParameters()
            native.lambda = parameters.lambda
            native.eta = parameters.eta
            native.mu = parameters.mu
            native.nu = parameters.nu
            native.meshUpdateClosenessWeight = parameters.meshUpdateClosenessWeight
            native.meshUpdateIterations = parameters.meshUpdateIterations
            native.meshUpdateDisplacementEps = parameters.meshUpdateDisplacementEps
            native.outerIterations = parameters.outerIterations

            do {
                return preserveCoincidentInputVertices(
                    try NativeDenoiser.denoise(
                        positions: positions,
                        indices: indices,
                        parameters: native,
                        useGPU: useGPU
                    ) { completed, total in
                        if session.isCancelled { return false }
                        session.progressHandler?(Double(completed) / Double(total))
                        return true
                    },
                    originalPositions: positions
                )
            } catch NativeDenoiseError.cancelled {
                throw CancellationError()
            } catch NativeDenoiseError.invalidInput {
                throw MeshDenoiseError.invalidInput
            } catch NativeDenoiseError.invalidParameters {
                throw MeshDenoiseError.invalidParameters
            } catch NativeDenoiseError.solverFailed {
                throw MeshDenoiseError.solverFailed
            } catch NativeDenoiseError.notImplemented {
                throw MeshDenoiseError.solverFailed
            }
        }
    }

    private static func preserveCoincidentInputVertices(
        _ denoisedPositions: [SIMD3<Float>],
        originalPositions: [SIMD3<Float>],
        epsilon: Float = 1e-6
    ) -> [SIMD3<Float>] {
        guard denoisedPositions.count == originalPositions.count else {
            return denoisedPositions
        }

        var groups = [CoincidentPositionKey: [Int]]()
        groups.reserveCapacity(originalPositions.count)
        for (index, position) in originalPositions.enumerated() {
            groups[CoincidentPositionKey(position: position, epsilon: epsilon), default: []].append(index)
        }

        var output = denoisedPositions
        for indices in groups.values where indices.count > 1 {
            var average = SIMD3<Float>(repeating: 0)
            for index in indices {
                average += denoisedPositions[index]
            }
            average /= Float(indices.count)
            for index in indices {
                output[index] = average
            }
        }
        return output
    }

    private static func validateDenoisedPositions(_ positions: [SIMD3<Float>], expectedCount: Int) throws {
        guard positions.count == expectedCount else {
            throw MeshDenoiseError.solverFailed
        }
        for position in positions {
            guard position.x.isFinite, position.y.isFinite, position.z.isFinite else {
                throw MeshDenoiseError.solverFailed
            }
        }
    }

    private static func runReferenceSync(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: MeshDenoiseParameters,
        session: DenoiseSession
    ) throws -> [SIMD3<Float>] {
        var flat = [Float](repeating: 0, count: positions.count * 3)
        for (i, p) in positions.enumerated() {
            flat[3 * i] = p.x
            flat[3 * i + 1] = p.y
            flat[3 * i + 2] = p.z
        }
        var out = [Float](repeating: 0, count: flat.count)
        var cParams = parameters.cParams

        let callback: md_progress_fn = { completed, total, rawCtx in
            let session = Unmanaged<DenoiseSession>.fromOpaque(rawCtx!).takeUnretainedValue()
            if session.isCancelled { return false }
            session.progressHandler?(Double(completed) / Double(total))
            return true
        }

        let status = withExtendedLifetime(session) {
            md_denoise(flat, positions.count,
                       indices, indices.count / 3,
                       &cParams, &out,
                       callback, Unmanaged.passUnretained(session).toOpaque())
        }

        switch status {
        case MD_OK:
            return (0..<positions.count).map {
                SIMD3<Float>(out[3 * $0], out[3 * $0 + 1], out[3 * $0 + 2])
            }
        case MD_CANCELLED:
            throw CancellationError()
        case MD_ERR_INVALID_INPUT:
            throw MeshDenoiseError.invalidInput
        case MD_ERR_INVALID_PARAMS:
            throw MeshDenoiseError.invalidParameters
        default:
            throw MeshDenoiseError.solverFailed
        }
    }
}

private struct CoincidentPositionKey: Hashable {
    var x: Int64
    var y: Int64
    var z: Int64

    init(position: SIMD3<Float>, epsilon: Float) {
        x = Int64((position.x / epsilon).rounded())
        y = Int64((position.y / epsilon).rounded())
        z = Int64((position.z / epsilon).rounded())
    }
}
