/// Errors thrown by ``MeshDenoiser/denoise(positions:indices:parameters:progress:)``.
/// Cancellation is reported as `CancellationError`, not through this enum.
public enum MeshDenoiseError: Error, Equatable {
    /// Empty buffers, NaN/Inf coordinates, out-of-range indices, or a degenerate/non-manifold face.
    case invalidInput
    /// A parameter is out of its documented range.
    case invalidParameters
    /// The sparse solver failed to factorize or converge.
    case solverFailed
}
