#include "CMeshDenoiserCore.h"

void md_params_init_defaults(md_params *params)
{
    params->lambda = 0.15;
    params->eta = 2.2;
    params->mu = 0.2;
    params->nu = 0.25;
    params->mesh_update_closeness_weight = 0.001;
    params->mesh_update_iterations = 20;
    params->mesh_update_displacement_eps = 0.1;
    params->outer_iterations = 1;
    params->linear_solver_type = 1;
    params->deterministic = 0;
}

md_status md_denoise(const float *, size_t, const uint32_t *, size_t,
                     const md_params *, float *, md_progress_fn, void *)
{
    return MD_ERR_SOLVER_FAILED; // stub — replaced in Task 4
}
