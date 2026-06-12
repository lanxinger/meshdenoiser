#include "CMeshDenoiserCore.h"

#include "MeshTypes.h"
#include "MeshNormalDenoising.h"

#include <Eigen/Core>
#include <cmath>
#include <vector>

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

md_status md_denoise(const float *vertices, size_t vertex_count,
                     const uint32_t *indices, size_t triangle_count,
                     const md_params *params,
                     float *out_vertices,
                     md_progress_fn progress, void *ctx)
{
    if(vertices == nullptr || indices == nullptr || params == nullptr || out_vertices == nullptr ||
       vertex_count == 0 || triangle_count == 0){
        return MD_ERR_INVALID_INPUT;
    }

    for(size_t i = 0; i < vertex_count * 3; ++i){
        if(!std::isfinite(vertices[i])){
            return MD_ERR_INVALID_INPUT;
        }
    }
    for(size_t i = 0; i < triangle_count * 3; ++i){
        if(indices[i] >= vertex_count){
            return MD_ERR_INVALID_INPUT;
        }
    }

    SDFilter::MeshDenoisingParameters param;
    param.lambda = params->lambda;
    param.eta = params->eta;
    param.mu = params->mu;
    param.nu = params->nu;
    param.mesh_update_closeness_weight = params->mesh_update_closeness_weight;
    param.mesh_update_iter = params->mesh_update_iterations;
    param.mesh_update_disp_eps = params->mesh_update_displacement_eps;
    param.outer_iterations = params->outer_iterations;
    param.deterministic_mode = (params->deterministic != 0);
    switch(params->linear_solver_type){
    case 0: param.linear_solver_type = SDFilter::Parameters::CG; break;
    case 1: param.linear_solver_type = SDFilter::Parameters::LDLT; break;
    default: return MD_ERR_INVALID_PARAMS;
    }
    if(!param.valid_parameters()){
        return MD_ERR_INVALID_PARAMS;
    }

    if(param.deterministic_mode){
        Eigen::setNbThreads(1);
    }

    TriMesh mesh;
    std::vector<TriMesh::VertexHandle> vhandles;
    vhandles.reserve(vertex_count);
    for(size_t i = 0; i < vertex_count; ++i){
        vhandles.push_back(mesh.add_vertex(TriMesh::Point(
            static_cast<double>(vertices[3 * i]),
            static_cast<double>(vertices[3 * i + 1]),
            static_cast<double>(vertices[3 * i + 2]))));
    }
    for(size_t i = 0; i < triangle_count; ++i){
        TriMesh::FaceHandle fh = mesh.add_face(
            vhandles[indices[3 * i]],
            vhandles[indices[3 * i + 1]],
            vhandles[indices[3 * i + 2]]);
        if(!fh.is_valid()){
            return MD_ERR_INVALID_INPUT; // degenerate or non-manifold face
        }
    }

    // Same pipeline as the CLI: normalize -> denoise -> restore.
    Eigen::Vector3d original_center;
    double original_scale = 0.0;
    SDFilter::normalize_mesh(mesh, original_center, original_scale);

    SDFilter::MeshNormalDenoising denoiser(mesh);
    if(progress != nullptr){
        denoiser.set_progress_callback([progress, ctx](int completed, int total) -> bool {
            return progress(completed, total, ctx);
        });
    }

    TriMesh output_mesh;
    if(!denoiser.denoise(param, output_mesh)){
        return denoiser.cancelled() ? MD_CANCELLED : MD_ERR_SOLVER_FAILED;
    }

    SDFilter::restore_mesh(output_mesh, original_center, original_scale);

    for(TriMesh::ConstVertexIter cv_it = output_mesh.vertices_begin();
        cv_it != output_mesh.vertices_end(); ++cv_it){
        const TriMesh::Point &pt = output_mesh.point(*cv_it);
        const int idx = cv_it->idx();
        out_vertices[3 * idx] = static_cast<float>(pt[0]);
        out_vertices[3 * idx + 1] = static_cast<float>(pt[1]);
        out_vertices[3 * idx + 2] = static_cast<float>(pt[2]);
    }

    return MD_OK;
}
