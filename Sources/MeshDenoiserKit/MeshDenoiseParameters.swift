import CMeshDenoiserCore

/// SD-filter denoising parameters. Defaults are the tuned values from MeshDenoiserDefaults.txt.
public struct MeshDenoiseParameters: Sendable, Equatable {

    public enum Backend: Sendable, Equatable, CaseIterable {
        /// Native CPU by default; use `.nativeGPU` explicitly for benchmarked GPU runs.
        case automatic
        /// Native Metal filter with a CPU conjugate-gradient vertex update.
        /// Throws `.gpuUnavailable` when its Metal pipeline cannot be created.
        case nativeGPU
        /// Native CPU implementation with a conjugate-gradient vertex update.
        case nativeCPU
        /// Wrapped C++ implementation; current test oracle.
        case reference
    }

    public enum LinearSolver: Int32, Sendable {
        /// Iterative conjugate gradient for the reference backend.
        /// Native backends always use their built-in conjugate-gradient vertex update.
        case conjugateGradient = 0
        /// Direct sparse Cholesky for the reference backend (default).
        /// Native backends always use their built-in conjugate-gradient vertex update.
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
    /// Reference-backend solver. Native backends use their built-in conjugate-gradient update.
    public var linearSolver: LinearSolver = .ldlt
    /// Forces single-threaded execution for bit-reproducible output.
    public var deterministic: Bool = false
    /// Which implementation runs the filter. Kept on `.reference` until native parity gates pass.
    public var backend: Backend = .reference

    public init() {}

    var isValid: Bool {
        let filterParameters = [lambda, eta, mu, nu]
        let nativeFilterParametersAreRepresentable = filterParameters.allSatisfy { value in
            let nativeValue = Float(value)
            return nativeValue.isFinite && nativeValue > 0
        }

        return filterParameters.allSatisfy { $0.isFinite && $0 > 0 }
            && (backend == .reference || nativeFilterParametersAreRepresentable)
            && meshUpdateClosenessWeight.isFinite
            && meshUpdateClosenessWeight >= 0
            && Int32(exactly: meshUpdateIterations) != nil
            && meshUpdateIterations > 0
            && meshUpdateDisplacementEps.isFinite
            && Int32(exactly: outerIterations) != nil
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
        c.mesh_update_iterations = Int32(exactly: meshUpdateIterations) ?? 0
        c.mesh_update_displacement_eps = meshUpdateDisplacementEps
        c.outer_iterations = Int32(exactly: outerIterations) ?? 0
        c.linear_solver_type = linearSolver.rawValue
        c.deterministic = deterministic ? 1 : 0
        return c
    }
}
