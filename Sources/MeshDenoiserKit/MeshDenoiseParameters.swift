import CMeshDenoiserCore

/// SD-filter denoising parameters. Defaults are the tuned values from MeshDenoiserDefaults.txt.
public struct MeshDenoiseParameters: Sendable, Equatable {

    public enum Backend: Sendable, Equatable, CaseIterable {
        /// Native GPU if a Metal device exists, else native CPU.
        case automatic
        /// Native Metal implementation; throws `.gpuUnavailable` without a Metal device.
        case nativeGPU
        /// Native CPU implementation.
        case nativeCPU
        /// Wrapped C++ implementation; current test oracle.
        case reference
    }

    public enum LinearSolver: Int32, Sendable {
        /// Iterative conjugate gradient — lower memory, use for very large meshes (>~500k vertices).
        case conjugateGradient = 0
        /// Direct sparse Cholesky (default) — fastest for typical mesh sizes.
        case ldlt = 1
    }

    /// Regularization weight. Higher = more smoothing per iteration. Must be > 0.
    public var lambda: Double = 0.15
    /// Spatial Gaussian sigma, scaled by average face-centroid distance. Must be > 0.
    public var eta: Double = 2.2
    /// Guidance normal difference weight. Must be > 0.
    public var mu: Double = 0.2
    /// Signal normal difference weight. Must be > 0.
    public var nu: Double = 0.25
    /// Vertex position fidelity during mesh update. Must be >= 0.
    public var meshUpdateClosenessWeight: Double = 0.001
    /// Max vertex-update iterations per outer iteration. Must be > 0.
    public var meshUpdateIterations: Int = 20
    /// Early-stop RMS displacement threshold for mesh update; <= 0 disables.
    public var meshUpdateDisplacementEps: Double = 0.1
    /// Number of full filtering passes. More = more smoothing. Must be > 0.
    public var outerIterations: Int = 1
    public var linearSolver: LinearSolver = .ldlt
    /// Forces single-threaded execution for bit-reproducible output.
    public var deterministic: Bool = false
    /// Which implementation runs the filter. Kept on `.reference` until native parity gates pass.
    public var backend: Backend = .reference

    public init() {}

    var isValid: Bool {
        lambda > 0 && eta > 0 && mu > 0 && nu > 0
            && meshUpdateClosenessWeight >= 0
            && meshUpdateIterations > 0
            && meshUpdateDisplacementEps.isFinite
            && outerIterations > 0
    }

    var cParams: md_params {
        var c = md_params()
        md_params_init_defaults(&c)
        c.lambda = lambda
        c.eta = eta
        c.mu = mu
        c.nu = nu
        c.mesh_update_closeness_weight = meshUpdateClosenessWeight
        c.mesh_update_iterations = Int32(meshUpdateIterations)
        c.mesh_update_displacement_eps = meshUpdateDisplacementEps
        c.outer_iterations = Int32(outerIterations)
        c.linear_solver_type = linearSolver.rawValue
        c.deterministic = deterministic ? 1 : 0
        return c
    }
}
