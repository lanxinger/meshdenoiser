# Native Swift + Metal Port — Design

**Date:** 2026-06-12
**Status:** Approved (decisions made interactively with Markus)
**Goal:** Reimplement the SD-filter denoiser natively in Swift + Metal + Accelerate so it performs well on iOS/iPadOS/macOS, behind the existing `MeshDenoiserKit` public API.

## Motivation (measured, M-series Mac, single-threaded C++ CLI)

| Faces | Total | Preprocessing | Filtering | Mesh update | Peak RSS |
|---|---|---|---|---|---|
| 5,120 | 0.03s | 0.02 | 0.01 | 0.00 | — |
| 81,920 | 0.63s | 0.31 | 0.16 | 0.16 | — |
| 327,680 | 5.2s | 2.7 (53%) | 1.0 (19%) | 1.4 (28%) | **6.0 GB** |
| 1,310,720 | killed after ~5 min (thrashing) | — | — | — | >5.1 GB |

Conclusions:
1. **Memory is the wall.** ~40M neighbor pairs stored as doubles + 64-bit indices with bidirectional adjacency ≈ 60–70 B/pair. A float32/uint32 compact layout is ~12–16 B/pair (4–5×), the difference between jetsam and feasible on iPhone.
2. **Hot loops are embarrassingly parallel and currently serial** (the Apple build has no OpenMP). Preprocessing and filtering are per-element loops — Metal compute territory; even CPU parallelism gives ~8×.
3. **The sparse solve is not the bottleneck** (28%, factorize-once + cheap triangular solves). It stays on CPU via Accelerate `SparseFactor` (Cholesky); no GPU solver.

## Decisions

- **Precision:** float32 throughout the GPU/CPU filter path; the vertex-update solve runs in double (normal equations square the condition number; the stage is cheap). Correctness = **tolerance parity** against the existing CLI golden files + cross-backend comparison, not bit parity.
- **C++ backend fate:** kept as `.reference` backend (test oracle + fallback) until the native backend is validated on-device, then removed in a follow-up.
- **Public API unchanged:** `MeshDenoiser.denoise(positions:indices:parameters:progress:)`. `MeshDenoiseParameters` gains a `backend` field (`.automatic` = native GPU→CPU fallback, `.nativeGPU`, `.nativeCPU`, `.reference`). The Plinth integration plan is unaffected.
- **Scope cuts (match what the CLI actually exercises):** only `normalize_iterates == true` and the ITERATIVE mesh-update path are ported (the CLI never uses the homogeneous or Poisson variants). CHOLMOD/OpenMP irrelevant. The native backend uses Accelerate Cholesky regardless of `linearSolver` (documented; CG-on-GPU is a follow-up if profiling ever shows the solve dominating).

## Architecture

New Swift target `MeshDenoiserNative` (no third-party deps; Metal shaders compiled by SwiftPM into the module bundle):

```
Pipeline per outer iteration (mirrors C++ semantics exactly):
  connectivity  (CPU, parallel)   faces → normals/areas/centroids, edges+adjacency CSR
  guidance      (CPU, parallel)   edge saliency → patch consistency → guidance normals
  neighbor search (CPU, parallel) uniform grid over centroids, radius 3η′; compact pair list
  static weights (CPU, parallel)  (aᵢ+aⱼ)·exp(h_g‖gᵢ−gⱼ‖² + h_s d²); λ rescale
  fixed-point filter (GPU, ≤100 iters)   per-pair dynamic weights → per-face gather+normalize
                                          → convergence reduction   [CPU fallback path]
  vertex update (CPU, Accelerate)  AᵀA + wI Cholesky (factor once, cached) +
                                   ≤20 iters of per-face projection + scatter + solve
```

Key formulas ported verbatim (from `src/SDFilter.h` / `MeshNormalFilter.h` / `MeshNormalDenoising.h`):
- η′ = η · mean adjacent-face-centroid distance; search radius = 3η′
- h_spatial = −1/(2η′²), h_guidance = −1/(2μ²), h_ν = −1/(2ν²)
- λ′ = λ · Σa / Σ((aᵢ+aⱼ)·exp(h_spatial·d²)); weighted-init scale per face = aᵢ·2ν²/λ′
- filter convergence: Σaᵢ‖Δsᵢ‖² ≤ Σa·ε², ε = 2·sin(0.1·π/180), max 100 iterations
- mesh update: w = closeness·F/V; per-face AᵀA contribution = I−J/3 on the face's vertex block (since (I−J/3)ᵀ(I−J/3) = I−J/3); RMS-displacement early stop
- areas normalized to mean 1; meshes normalized to bbox-diag 1 around the centroid before filtering and restored after

## Testing

- Unit tests per stage (connectivity counts, neighbor search vs brute force, solver on known system, projection math).
- End-to-end tolerance parity vs the committed CLI goldens (initial gates: max vertex error < 2e-3, mean < 2e-4 on the unit-scale fixture; calibrate from measured values and record them).
- Cross-backend test: native CPU vs `.reference` on the fixtures.
- GPU-vs-CPU parity < 1e-4 (same float32 math modulo reduction order), gated on Metal device availability.
- Existing contract tests (errors, cancellation, progress) parameterized over all backends — identical observable behavior.
- Benchmark executable (macOS) comparing backends at 20k/82k/328k faces; results recorded in the repo.

## Expected outcome

At 328k faces: ~0.5–1s total (vs 5.2s) and well under 1 GB peak (vs 6 GB); 1M+ faces becomes feasible on iPad/Mac and plausible on recent iPhones.

## Out of scope

- GPU vertex-update solver (CG); Poisson update; homogeneous (non-normalized) iterates
- Removing the C++ backend (follow-up after on-device validation)
- Mid-filter-iteration progress reporting (progress contract stays per-outer-iteration)
