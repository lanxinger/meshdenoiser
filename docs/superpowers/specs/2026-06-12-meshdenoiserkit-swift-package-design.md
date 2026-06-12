# MeshDenoiserKit — Swift Package Design

**Date:** 2026-06-12
**Status:** Approved
**Goal:** Make the MeshDenoiser SD-filter algorithm usable from the Plinth Capture iOS/macOS app as a user-triggered "denoise model" tool, by wrapping the existing C++ core in a Swift Package.

## Decision summary

- **Approach:** Wrap the existing C++17 algorithm core directly (no Swift/Metal port). A Metal port was evaluated and rejected: the dominant cost is Eigen's sparse Cholesky factorization (`SimplicialLDLT`) in the vertex-update step, which is a sequential direct solver with no Metal/MPS equivalent. A pure Swift + Accelerate port (1–2 weeks + numerical validation) is not justified when the C++ compiles cleanly for arm64.
- **Usage context:** User-triggered action in the Plinth model viewer, run on a loaded model. Not part of the automatic capture/upload pipeline (may be revisited later).
- **Package boundary:** Format-agnostic — raw vertex/index buffers in, denoised vertex buffer out. All USDZ/file handling stays on the app side.

## Architecture

A SwiftPM package defined at the root of this repo (`Package.swift`), with two targets:

### Target 1: `CMeshDenoiserCore` (C++17)

Vendors the algorithm and its two real dependencies:

- `src/EigenTypes.h`, `src/MeshTypes.h`, `src/SDFilter.h`, `src/MeshNormalFilter.h`, `src/MeshNormalDenoising.h` (unchanged except the patches below)
- **Eigen** (header-only, MPL2)
- **OpenMesh Core** (BSD-3) — compiled sources; only the Core module, no IO plugins needed beyond what TriMesh requires

Explicitly excluded: `MeshDenoiser.cpp` (CLI), tinygltf, tinyusdz, AppMetrics, CHOLMOD, OpenMP.

Public C API (umbrella header `include/CMeshDenoiserCore.h`):

```c
typedef struct {
    double lambda;                  // default 0.15
    double eta;                     // default 2.2
    double mu;                      // default 0.2
    double nu;                      // default 0.25
    double mesh_update_closeness_weight;   // default 0.001
    int    mesh_update_iterations;         // default 20
    double mesh_update_displacement_eps;   // default 0.1
    int    outer_iterations;               // default 1
    int    linear_solver_type;             // 0 = CG, 1 = LDLT (default)
} md_params;

typedef enum {
    MD_OK = 0,
    MD_ERR_INVALID_INPUT,    // NaN/Inf coords, degenerate counts, bad indices
    MD_ERR_INVALID_PARAMS,
    MD_ERR_SOLVER_FAILED,
    MD_CANCELLED
} md_status;

// Return false from the callback to request cancellation.
// completed_outer / total_outer report outer-iteration progress.
typedef bool (*md_progress_fn)(int completed_outer, int total_outer, void *ctx);

void md_params_init_defaults(md_params *params);

md_status md_denoise(const float *vertices, size_t vertex_count,   // xyz triples
                     const uint32_t *indices, size_t triangle_count, // 3 per tri
                     const md_params *params,
                     float *out_vertices,                          // caller-allocated, same size as input
                     md_progress_fn progress, void *ctx);          // progress may be NULL
```

Inside, the implementation builds an OpenMesh `TriMesh` from the buffers, runs `SDFilter::MeshNormalDenoising::denoise`, and copies positions back **in the original vertex order** (the algorithm only moves vertices; it never adds, removes, or reorders them).

Required patches to the vendored core (kept minimal):

1. **Cancellation + progress hook:** `MeshNormalDenoising::denoise` gains an optional callback invoked after each outer iteration; returning false aborts and surfaces `MD_CANCELLED`. No mid-iteration cancellation in v1.
2. **Silence stdout:** progress/diagnostic printing already has flags; ensure nothing writes to stdout/stderr from the library path.

### Target 2: `MeshDenoiserKit` (Swift)

Thin wrapper over the C API:

```swift
public struct MeshDenoiseParameters: Sendable {
    public var lambda: Double = 0.15
    // … mirrors md_params, documented with the tuned defaults
    public static let `default` = MeshDenoiseParameters()
}

public enum MeshDenoiser {
    /// Runs off the calling actor; supports Task cancellation; reports
    /// per-outer-iteration progress via the optional handler.
    public static func denoise(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: MeshDenoiseParameters = .default,
        progress: (@Sendable (Double) -> Void)? = nil
    ) async throws -> [SIMD3<Float>]
}
```

- Errors map `md_status` to a Swift `MeshDenoiseError` enum.
- Task cancellation is bridged to the C callback's cancel return.
- The denoise call runs on a background thread (the C call is synchronous and long-running).

## App integration (Plinth side — separate plan, not in this repo)

User-triggered flow in the model viewer:

1. Load the project's USDZ via SceneKit/ModelIO (both read USDZ natively on iOS/macOS).
2. Extract position buffer + triangle indices per geometry.
3. `await MeshDenoiser.denoise(...)` with progress UI and cancel button.
4. Rebuild geometry with identical vertex order — original UVs, materials, and textures carry over unchanged. Recompute normals (stored normals are stale after vertex movement).
5. Preview in the existing viewer; export via `SCNScene.write(to:)`, which supports `.usdz` output on iOS/macOS.

Vertex count/order preservation is the contract that keeps the texture pipeline safe.

## Build details

- `-march=native`/`-mtune=native` from the CMake build are not applied in the package (break arm64 cross-compilation; irrelevant on Apple Silicon).
- OpenMP is omitted: the `OMP_*` macros compile to no-ops without `USE_OPENMP`, and the bottleneck (LDLT factorization) is single-threaded regardless.
- CHOLMOD is omitted; `LinearSolverType 2` is not exposed.
- Large meshes: the wrapper exposes the existing CG solver (`linear_solver_type = 0`) as the documented fallback for meshes where LDLT memory becomes a concern (~>500k vertices on iPhone-class devices).
- Platforms: iOS 17+, macOS 14+ (matching the app's targets; no API in the package requires anything newer).
- The existing CMake CLI build remains untouched and continues to work.

## Testing

- **Golden-file parity test** (package test target): run a small fixture mesh (subsampled from existing test data) through `MeshDenoiserKit` in deterministic mode and compare vertex positions against committed output produced by the already-validated CLI binary. Tolerance 1e-6.
- **Contract tests:** vertex count/order preservation, cancellation surfaces `CancellationError`/`MD_CANCELLED`, invalid input (NaN coords, out-of-range indices) rejected with the right error.
- CI: add an `xcodebuild`/`swift test` job for the package alongside the existing CMake builds.

## Licensing

All App Store–compatible; ship license texts in the package resources:

- MeshSDFilter algorithm: BSD-3-Clause
- OpenMesh 11: BSD-3-Clause
- Eigen: MPL2 (header-only, file-level copyleft — unmodified use is fine)
- This repo: MPL2

## Out of scope (v1)

- Metal/GPU acceleration of the normal-filtering passes
- Automatic denoising in the capture/upload pipeline
- USDZ/glTF reading inside the package (app owns file I/O)
- Mid-iteration cancellation granularity
- Exposing CHOLMOD or OpenMP builds
