import Foundation

/// Errors thrown by ``MeshDenoiser/denoise(positions:indices:parameters:progress:)``.
/// Cancellation is reported as `CancellationError`, not through this enum.
public enum MeshDenoiseError: LocalizedError, Sendable, Equatable {
    /// Empty buffers, NaN/Inf coordinates, out-of-range indices, or a degenerate/non-manifold face.
    case invalidInput
    /// A parameter is out of its documented range.
    case invalidParameters
    /// The sparse solver failed to factorize or converge.
    case solverFailed
    /// `.nativeGPU` was requested but no Metal device is available.
    case gpuUnavailable

    public var errorDescription: String? {
        switch self {
        case .invalidInput:
            return "Mesh input is empty or invalid."
        case .invalidParameters:
            return "Mesh denoising parameters are invalid."
        case .solverFailed:
            return "The mesh denoising solver failed."
        case .gpuUnavailable:
            return "A Metal device is not available."
        }
    }
}
