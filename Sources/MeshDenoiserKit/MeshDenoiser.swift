import CMeshDenoiserCore
import Dispatch
import Foundation
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
        return try await withTaskCancellationHandler {
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
    }

    private static func runSync(
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
