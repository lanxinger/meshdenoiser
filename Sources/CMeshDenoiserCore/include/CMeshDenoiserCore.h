// C API for the SD-filter mesh denoiser (algorithm: Zhang et al., arXiv:1712.03574).
// Buffers in / buffers out; vertex count and order are preserved.
#ifndef CMESHDENOISERCORE_H
#define CMESHDENOISERCORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double lambda;                        // regularization weight (> 0)
    double eta;                           // spatial Gaussian sigma, scaled by avg face-centroid distance (> 0)
    double mu;                            // guidance normal difference weight (> 0)
    double nu;                            // signal normal difference weight (> 0)
    double mesh_update_closeness_weight;  // vertex position fidelity (>= 0)
    int    mesh_update_iterations;        // max vertex-update iterations per outer iteration (> 0)
    double mesh_update_displacement_eps;  // early-stop RMS displacement threshold (<= 0 disables)
    int    outer_iterations;              // full filtering passes (> 0)
    int    linear_solver_type;            // 0 = conjugate gradient, 1 = LDLT (default)
    int    deterministic;                 // nonzero forces single-threaded Eigen for reproducible output
} md_params;

typedef enum {
    MD_OK = 0,
    MD_ERR_INVALID_INPUT = 1,   // NULL/empty buffers, NaN/Inf coords, bad indices, non-manifold face
    MD_ERR_INVALID_PARAMS = 2,
    MD_ERR_SOLVER_FAILED = 3,
    MD_CANCELLED = 4
} md_status;

// Called after each completed outer iteration. Return false to cancel.
typedef bool (*md_progress_fn)(int completed_outer, int total_outer, void *ctx);

// Fills params with the tuned defaults from MeshDenoiserDefaults.txt.
void md_params_init_defaults(md_params *params);

// vertices: xyz triples, vertex_count points.
// indices: 3 per triangle, triangle_count triangles, each index < vertex_count.
// out_vertices: caller-allocated, 3 * vertex_count floats. Written only on MD_OK.
// progress may be NULL; ctx is passed through to it.
md_status md_denoise(const float *vertices, size_t vertex_count,
                     const uint32_t *indices, size_t triangle_count,
                     const md_params *params,
                     float *out_vertices,
                     md_progress_fn progress, void *ctx);

#ifdef __cplusplus
}
#endif

#endif
