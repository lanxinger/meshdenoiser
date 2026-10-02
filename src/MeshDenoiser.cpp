// BSD 3-Clause License
//
// Copyright (c) 2017, Bailin Deng
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include "MeshTypes.h"
#include "MeshNormalDenoising.h"
#include "AppMetrics.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef USE_OPENMP
#include <omp.h>
#endif

#include <tiny_gltf_v3.h>

#include "tinyusdz.hh"
#include "tydra/render-data.hh"
#include "tydra/scene-access.hh"

namespace
{

using namespace tinyusdz;  // For tydra namespace access

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

std::string to_lower_copy(std::string str)
{
	std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return str;
}

bool has_extension(const std::string &path, const std::string &ext)
{
	if(path.size() < ext.size()){
		return false;
	}
	return to_lower_copy(path.substr(path.size() - ext.size())) == ext;
}

// ---------------------------------------------------------------------------
// CLI helpers
// ---------------------------------------------------------------------------

void configure_deterministic_runtime(bool deterministic)
{
	if(!deterministic){
		return;
	}
#ifdef USE_OPENMP
	omp_set_dynamic(0);
	omp_set_num_threads(1);
#endif
	Eigen::setNbThreads(1);
}

const char* default_options_text()
{
	return
R"(#### Option file for SD filter based mesh denoising
#### Lines starting with '#' are comments

## Regularization weight, must be positive.
## Higher values = more smoothing per iteration.
Lambda  0.15

## Gaussian standard deviation for spatial weight, scaled by the average
## distance between adjacent face centroids. Must be positive.
## Higher values = wider spatial neighborhood.
Eta 2.2

## Gaussian standard deviation for guidance weight, must be positive.
Mu  0.2

## Gaussian standard deviation for signal weight, must be positive.
Nu  0.25

## Closeness weight for mesh update, must be positive.
MeshUpdateClosenessWeight  0.001

## Iterations for mesh update, must be positive.
MeshUpdateIterations  20

## Early-stop threshold for per-iteration mesh update RMS displacement (<=0 disables).
MeshUpdateDisplacementEps 1e-1

## Outer iterations for denoising, must be positive integer.
## More iterations = more smoothing overall.
OuterIterations   1

## Force deterministic mode (single-threaded runtime) for reproducible outputs (0/1).
DeterministicMode 0

## Linear solver backend: 0=CG, 1=Eigen LDLT, 2=CHOLMOD (falls back to 1 if unavailable).
LinearSolverType 1
)";
}

bool write_default_options_file(const std::string &path)
{
	std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
	if(!out.is_open()){
		return false;
	}

	out << default_options_text();
	return out.good();
}

void apply_default_options(SDFilter::MeshDenoisingParameters &param)
{
	param.lambda = 0.15;
	param.eta = 2.2;
	param.mu = 0.2;
	param.nu = 0.25;
	param.mesh_update_closeness_weight = 0.001;
	param.mesh_update_iter = 20;
	param.mesh_update_disp_eps = 1e-1;
	param.outer_iterations = 1;
	param.deterministic_mode = false;
	param.linear_solver_type = static_cast<SDFilter::Parameters::LinearSolverType>(SDFilter::DEFAULT_LINEAR_SOLVER_TYPE);
}

void print_help(const char *exe_name)
{
	std::cout << "MeshDenoiser - Mesh normal denoising filter" << std::endl;
	std::cout << std::endl;
	std::cout << "Usage:" << std::endl;
	std::cout << "  " << exe_name << " INPUT_MESH OUTPUT_MESH [optional flags]" << std::endl;
	std::cout << "  " << exe_name << " OPTION_FILE INPUT_MESH OUTPUT_MESH [optional flags]" << std::endl;
	std::cout << "  " << exe_name << " --write-default-options PATH" << std::endl;
	std::cout << "  " << exe_name << " --help" << std::endl;
	std::cout << std::endl;
	std::cout << "Positional arguments:" << std::endl;
	std::cout << "  OPTION_FILE               Optional. Path to denoising options text file." << std::endl;
	std::cout << "                            If omitted, built-in defaults are used." << std::endl;
	std::cout << "  INPUT_MESH                Required. Input mesh to denoise." << std::endl;
	std::cout << "  OUTPUT_MESH               Required. Output mesh path." << std::endl;
	std::cout << std::endl;
	std::cout << "Supported input formats:" << std::endl;
	std::cout << "  OBJ, PLY, OFF, STL (via OpenMesh)" << std::endl;
	std::cout << "  glTF (.gltf, .glb)" << std::endl;
	std::cout << "  USD (.usd, .usda, .usdc, .usdz)" << std::endl;
	std::cout << std::endl;
	std::cout << "Optional flags:" << std::endl;
	std::cout << "  --export-precision N      Vertex coordinate precision for output. Default: 16." << std::endl;
	std::cout << "  --metrics-json PATH       Write JSON timing and solver metrics." << std::endl;
	std::cout << "  --metrics-csv PATH        Append CSV timing and solver metrics." << std::endl;
	std::cout << "  --deterministic           Force deterministic single-threaded runtime." << std::endl;
	std::cout << "  --write-default-options PATH" << std::endl;
	std::cout << "                            Write the default options template to PATH and exit." << std::endl;
	std::cout << "  --help                    Show this help text and exit." << std::endl;
}

// ---------------------------------------------------------------------------
// Input validation
// ---------------------------------------------------------------------------

bool validate_mesh_vertices(const TriMesh &mesh)
{
	for(auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it){
		const auto &pt = mesh.point(*v_it);
		if(!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2])){
			std::cerr << "Error: non-finite vertex coordinate at index " << v_it->idx() << std::endl;
			return false;
		}
	}
	return true;
}

// ---------------------------------------------------------------------------
// glTF loader
// ---------------------------------------------------------------------------

struct PrimitiveInstance
{
	const tg3_primitive *primitive;
	Eigen::Matrix4d transform;
};

Eigen::Matrix4d compose_transform(const tg3_node &node)
{
	if(node.has_matrix){
		Eigen::Matrix4d mat;
		for(int r = 0; r < 4; ++r){
			for(int c = 0; c < 4; ++c){
				mat(r, c) = node.matrix[r + 4 * c];
			}
		}
		return mat;
	}

	Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
	translation(0, 3) = node.translation[0];
	translation(1, 3) = node.translation[1];
	translation(2, 3) = node.translation[2];

	Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
	Eigen::Quaterniond q(node.rotation[3], node.rotation[0], node.rotation[1], node.rotation[2]);
	q.normalize();
	rotation.block<3,3>(0,0) = q.toRotationMatrix();

	Eigen::Matrix4d scale = Eigen::Matrix4d::Identity();
	scale(0,0) = node.scale[0];
	scale(1,1) = node.scale[1];
	scale(2,2) = node.scale[2];

	return translation * rotation * scale;
}

bool collect_primitives(const tg3_model &model,
                        const std::vector<int32_t> &roots,
                        std::vector<PrimitiveInstance> &instances,
                        std::string &error)
{
	struct NodeVisit {
		int32_t index;
		Eigen::Matrix4d parent;
	};
	std::vector<NodeVisit> pending;
	std::vector<bool> visited(model.nodes_count, false);
	for(auto root = roots.rbegin(); root != roots.rend(); ++root){
		pending.push_back({*root, Eigen::Matrix4d::Identity()});
	}
	while(!pending.empty()){
		NodeVisit visit = pending.back();
		pending.pop_back();
		if(visit.index < 0 || static_cast<uint32_t>(visit.index) >= model.nodes_count){
			error = "Invalid node index in glTF scene.";
			return false;
		}
		if(visited[visit.index]){
			error = "Cyclic or repeated node in glTF scene.";
			return false;
		}
		visited[visit.index] = true;
		const tg3_node &node = model.nodes[visit.index];
		Eigen::Matrix4d world = visit.parent * compose_transform(node);
		if(node.mesh >= 0 && static_cast<uint32_t>(node.mesh) < model.meshes_count){
			const tg3_mesh &node_mesh = model.meshes[node.mesh];
			for(uint32_t i = 0; i < node_mesh.primitives_count; ++i){
				const tg3_primitive &primitive = node_mesh.primitives[i];
				if(primitive.mode == TG3_MODE_TRIANGLES || primitive.mode == -1){
					instances.push_back({&primitive, world});
				}
			}
		}
		for(uint32_t i = node.children_count; i > 0; --i){
			pending.push_back({node.children[i - 1], world});
		}
	}
	return true;
}

const uint8_t *accessor_element_ptr(const tg3_model &model,
                                    const tg3_accessor &accessor,
                                    uint64_t index)
{
	if(accessor.sparse.is_sparse || index >= accessor.count || accessor.buffer_view < 0 ||
	   static_cast<uint32_t>(accessor.buffer_view) >= model.buffer_views_count){
		return nullptr;
	}

	const tg3_buffer_view &view = model.buffer_views[accessor.buffer_view];
	if(view.buffer < 0 || static_cast<uint32_t>(view.buffer) >= model.buffers_count){
		return nullptr;
	}
	const tg3_buffer &buffer = model.buffers[view.buffer];
	int32_t component_size = tg3_component_size(accessor.component_type);
	int32_t components = tg3_num_components(accessor.type);
	int32_t stride = tg3_accessor_byte_stride(&accessor, &view);
	if(component_size <= 0 || components <= 0 || stride < component_size * components ||
	   !buffer.data.data || view.byte_offset > buffer.data.count ||
	   view.byte_length > buffer.data.count - view.byte_offset ||
	   accessor.byte_offset > view.byte_length){
		return nullptr;
	}

	uint64_t available = view.byte_length - accessor.byte_offset;
	uint64_t element_size = static_cast<uint64_t>(component_size * components);
	if(available < element_size || index > (available - element_size) / static_cast<uint64_t>(stride)){
		return nullptr;
	}
	return buffer.data.data + view.byte_offset + accessor.byte_offset + index * static_cast<uint64_t>(stride);
}

bool read_position(const tg3_model &model,
                   const tg3_accessor &accessor,
                   uint64_t index,
                   Eigen::Vector3d &out)
{
	const uint8_t *ptr = accessor_element_ptr(model, accessor, index);
	if(!ptr){
		return false;
	}

	float values[3];
	std::memcpy(values, ptr, sizeof(values));
	out = Eigen::Vector3d(static_cast<double>(values[0]),
	                      static_cast<double>(values[1]),
	                      static_cast<double>(values[2]));
	return true;
}

uint32_t read_index_element(const unsigned char *ptr, int component_type)
{
	switch(component_type){
		case TG3_COMPONENT_TYPE_UNSIGNED_BYTE:
			return ptr[0];
		case TG3_COMPONENT_TYPE_UNSIGNED_SHORT:
			return uint32_t(ptr[0]) | (uint32_t(ptr[1]) << 8);
		case TG3_COMPONENT_TYPE_UNSIGNED_INT:
			return uint32_t(ptr[0]) | (uint32_t(ptr[1]) << 8) | (uint32_t(ptr[2]) << 16) | (uint32_t(ptr[3]) << 24);
		default:
			return 0;
	}
}

bool load_gltf_mesh(const std::string &filename, TriMesh &mesh, std::string &warning, std::string &error)
{
	tinygltf3::Model parsed;
	tinygltf3::ErrorStack errors;
	tg3_parse_options options;
	tg3_parse_options_init(&options);
	options.images_as_is = 1;
	tg3_error_code result = tinygltf3::parse_file(parsed, errors, filename.c_str(), &options);
	for(uint32_t i = 0; i < errors.count(); ++i){
		const tg3_error_entry *entry = errors.entry(i);
		if(entry->message){
			std::string message = std::string(entry->message) + "\n";
			if(entry->severity == TG3_SEVERITY_ERROR){
				error += message;
			}
			else {
				warning += message;
			}
		}
	}
	if(result != TG3_OK){
		if(error.empty()){
			error = "Unable to parse glTF file.";
		}
		return false;
	}
	const tg3_model &model = *parsed.get();

	std::vector<PrimitiveInstance> instances;
	std::vector<int32_t> roots;
	if(model.scenes_count == 0){
		std::vector<bool> is_child(model.nodes_count, false);
		for(uint32_t i = 0; i < model.nodes_count; ++i){
			for(uint32_t j = 0; j < model.nodes[i].children_count; ++j){
				int32_t child = model.nodes[i].children[j];
				if(child >= 0 && static_cast<uint32_t>(child) < model.nodes_count){
					is_child[child] = true;
				}
			}
		}
		for(uint32_t i = 0; i < model.nodes_count; ++i){
			if(!is_child[i]){
				roots.push_back(static_cast<int32_t>(i));
			}
		}
	}
	else{
		int32_t scene_index = model.default_scene >= 0 ? model.default_scene : 0;
		if(static_cast<uint32_t>(scene_index) >= model.scenes_count){
			scene_index = 0;
		}
		const tg3_scene &scene = model.scenes[scene_index];
		for(uint32_t i = 0; i < scene.nodes_count; ++i){
			roots.push_back(scene.nodes[i]);
		}
	}
	if(!collect_primitives(model, roots, instances, error)){
		return false;
	}

	if(instances.empty()){
		error = "No triangulated primitives found in glTF file.";
		return false;
	}

	mesh.clear();
	mesh.request_face_normals();
	mesh.request_vertex_normals();

	using VertexKey = std::pair<size_t, uint32_t>;
	struct KeyHash
	{
		size_t operator()(const VertexKey &key) const noexcept
		{
			return std::hash<size_t>()(key.first) ^ (static_cast<size_t>(key.second) << 1);
		}
	};

	std::unordered_map<VertexKey, TriMesh::VertexHandle, KeyHash> vertex_cache;

	size_t instance_id = 0;
	for(const PrimitiveInstance &instance : instances){
		const tg3_primitive &primitive = *instance.primitive;
		int32_t position_accessor_index = -1;
		for(uint32_t i = 0; i < primitive.attributes_count; ++i){
			if(tg3_str_equals_cstr(primitive.attributes[i].key, "POSITION")){
				position_accessor_index = primitive.attributes[i].value;
				break;
			}
		}
		if(position_accessor_index < 0 || static_cast<uint32_t>(position_accessor_index) >= model.accessors_count){
			continue;
		}
		const tg3_accessor &position_accessor = model.accessors[position_accessor_index];

		if(position_accessor.type != TG3_TYPE_VEC3 ||
		   position_accessor.component_type != TG3_COMPONENT_TYPE_FLOAT || position_accessor.sparse.is_sparse){
			error = "Unsupported POSITION accessor format in glTF.";
			return false;
		}

		if(primitive.indices < 0 || static_cast<uint32_t>(primitive.indices) >= model.accessors_count){
			error = "Indexed primitives are required in glTF meshes.";
			return false;
		}
		const tg3_accessor &index_accessor = model.accessors[primitive.indices];

		if(index_accessor.type != TG3_TYPE_SCALAR || index_accessor.sparse.is_sparse ||
		   (index_accessor.component_type != TG3_COMPONENT_TYPE_UNSIGNED_BYTE &&
		    index_accessor.component_type != TG3_COMPONENT_TYPE_UNSIGNED_SHORT &&
		    index_accessor.component_type != TG3_COMPONENT_TYPE_UNSIGNED_INT)){
			error = "Unsupported index component type in glTF primitive.";
			return false;
		}

		if(!accessor_element_ptr(model, index_accessor, 0)){
			error = "Unable to read index data from glTF primitive.";
			return false;
		}
		if(index_accessor.count % 3 != 0){
			warning += "glTF primitive indices are not a multiple of three; trailing vertices will be ignored.\n";
		}

		for(size_t tri = 0; tri + 2 < index_accessor.count; tri += 3){
			std::array<TriMesh::VertexHandle, 3> face_vertices;

			for(int k = 0; k < 3; ++k){
				const uint8_t *index_ptr = accessor_element_ptr(model, index_accessor, tri + k);
				if(!index_ptr){
					error = "Truncated index data in glTF primitive.";
					return false;
				}
				uint32_t vertex_index = read_index_element(index_ptr, index_accessor.component_type);

				if(vertex_index >= position_accessor.count){
					error = "glTF index points outside POSITION accessor range.";
					return false;
				}

				VertexKey key(instance_id, vertex_index);
				auto cache_it = vertex_cache.find(key);
				if(cache_it == vertex_cache.end()){
					Eigen::Vector3d position;
					if(!read_position(model, position_accessor, vertex_index, position)){
						error = "Truncated POSITION data in glTF accessor.";
						return false;
					}

					Eigen::Vector4d p(position.x(), position.y(), position.z(), 1.0);
					Eigen::Vector4d transformed = instance.transform * p;
					TriMesh::Point point(transformed.x(), transformed.y(), transformed.z());

					TriMesh::VertexHandle vh = mesh.add_vertex(point);
					cache_it = vertex_cache.emplace(key, vh).first;
				}

				face_vertices[k] = cache_it->second;
			}

			if(!mesh.add_face(face_vertices[0], face_vertices[1], face_vertices[2]).is_valid()){
				warning += "Skipped a degenerate triangle during glTF import.\n";
			}
		}

		++instance_id;
	}

	if(mesh.n_faces() == 0){
		error = "No valid triangles found in glTF mesh.";
		return false;
	}

	mesh.garbage_collection();
	return true;
}

// ---------------------------------------------------------------------------
// USD loader
// ---------------------------------------------------------------------------

bool load_usd_mesh(const std::string &filename, TriMesh &mesh, std::string &warning, std::string &error)
{
	tinyusdz::Stage stage;
	std::string warn, err;

	bool success = false;
	if(has_extension(filename, ".usdz")){
		success = tinyusdz::LoadUSDZFromFile(filename, &stage, &warn, &err);
	}
	else if(has_extension(filename, ".usdc")){
		success = tinyusdz::LoadUSDCFromFile(filename, &stage, &warn, &err);
	}
	else if(has_extension(filename, ".usda")){
		success = tinyusdz::LoadUSDAFromFile(filename, &stage, &warn, &err);
	}
	else if(has_extension(filename, ".usd")){
		success = tinyusdz::LoadUSDFromFile(filename, &stage, &warn, &err);
	}
	else{
		error = "Unsupported USD file extension. Supported: .usd, .usda, .usdc, .usdz";
		return false;
	}

	if(!warn.empty()){
		warning += warn;
	}
	if(!err.empty()){
		error += err;
	}
	if(!success){
		if(error.empty()){
			error = "Unable to parse USD file.";
		}
		return false;
	}

	tydra::RenderScene render_scene;
	tydra::RenderSceneConverter converter;
	tydra::RenderSceneConverterEnv env(stage);

	if(!converter.ConvertToRenderScene(env, &render_scene)){
		error = "Failed to convert USD stage to render scene.";
		return false;
	}

	if(render_scene.meshes.empty()){
		error = "No meshes found in USD file.";
		return false;
	}

	mesh.clear();
	mesh.request_face_normals();
	mesh.request_vertex_normals();

	for(const auto &render_mesh : render_scene.meshes){
		if(render_mesh.points.empty()){
			continue;
		}

		std::vector<TriMesh::VertexHandle> vertex_handles;
		vertex_handles.reserve(render_mesh.points.size());

		for(const auto &point : render_mesh.points){
			TriMesh::Point p(point[0], point[1], point[2]);
			vertex_handles.push_back(mesh.add_vertex(p));
		}

		const std::vector<uint32_t> *face_indices = nullptr;
		const std::vector<uint32_t> *face_counts = nullptr;

		if(!render_mesh.triangulatedFaceVertexIndices.empty()){
			face_indices = &render_mesh.triangulatedFaceVertexIndices;
			face_counts = &render_mesh.triangulatedFaceVertexCounts;
		}
		else{
			face_indices = &render_mesh.usdFaceVertexIndices;
			face_counts = &render_mesh.usdFaceVertexCounts;
		}

		if(face_indices->empty() || face_counts->empty()){
			continue;
		}

		size_t index_offset = 0;
		for(uint32_t count : *face_counts){
			if(count == 3){
				std::vector<TriMesh::VertexHandle> face_verts;
				face_verts.reserve(3);

				bool valid = true;
				for(uint32_t i = 0; i < 3; ++i){
					uint32_t idx = (*face_indices)[index_offset + i];
					if(idx >= vertex_handles.size()){
						valid = false;
						break;
					}
					face_verts.push_back(vertex_handles[idx]);
				}

				if(valid){
					mesh.add_face(face_verts);
				}
			}
			else if(count > 3){
				std::vector<uint32_t> poly_indices;
				poly_indices.reserve(count);

				bool valid = true;
				for(uint32_t i = 0; i < count; ++i){
					uint32_t idx = (*face_indices)[index_offset + i];
					if(idx >= vertex_handles.size()){
						valid = false;
						break;
					}
					poly_indices.push_back(idx);
				}

				if(valid){
					for(uint32_t i = 1; i + 1 < count; ++i){
						std::vector<TriMesh::VertexHandle> face_verts;
						face_verts.push_back(vertex_handles[poly_indices[0]]);
						face_verts.push_back(vertex_handles[poly_indices[i]]);
						face_verts.push_back(vertex_handles[poly_indices[i + 1]]);
						mesh.add_face(face_verts);
					}
				}
			}

			index_offset += count;
		}
	}

	if(mesh.n_faces() == 0){
		error = "No valid triangles found in USD mesh.";
		return false;
	}

	mesh.garbage_collection();
	return true;
}

// ---------------------------------------------------------------------------
// Multi-format mesh loading
// ---------------------------------------------------------------------------

bool load_mesh(const std::string &filename, TriMesh &mesh)
{
	std::string warning;
	std::string error;
	bool success = false;

	if(has_extension(filename, ".gltf") || has_extension(filename, ".glb")){
		success = load_gltf_mesh(filename, mesh, warning, error);
	}
	else if(has_extension(filename, ".usd") || has_extension(filename, ".usda") ||
	        has_extension(filename, ".usdc") || has_extension(filename, ".usdz")){
		success = load_usd_mesh(filename, mesh, warning, error);
	}
	else{
		success = OpenMesh::IO::read_mesh(mesh, filename);
		if(!success){
			error = "unable to read input mesh with OpenMesh";
		}
	}

	if(!warning.empty()){
		std::cout << "Warning while loading mesh: " << warning << std::endl;
	}

	if(!success){
		std::cerr << "Error: " << error << std::endl;
	}

	return success;
}

} // anonymous namespace


int main(int argc, char **argv)
{
	if(argc == 1)
	{
		print_help(argv[0]);
		return 0;
	}

	if(argc == 2)
	{
		const std::string arg = argv[1];
		if(arg == "--help" || arg == "-h"){
			print_help(argv[0]);
			return 0;
		}
	}

	if(argc == 3)
	{
		const std::string arg = argv[1];
		if(arg == "--write-default-options")
		{
			if(!write_default_options_file(argv[2])){
				std::cerr << "Error: unable to write default options file " << argv[2] << std::endl;
				return 1;
			}

			std::cout << "Wrote default options to " << argv[2] << std::endl;
			return 0;
		}
	}

	// Count positional arguments (everything before the first --)
	int positional_argc = 0;
	for(int i = 1; i < argc; ++i)
	{
		const std::string arg = argv[i];
		if(arg.rfind("--", 0) == 0){
			break;
		}
		++positional_argc;
	}

	if(positional_argc != 2 && positional_argc != 3)
	{
		print_help(argv[0]);
		return 1;
	}

	std::string options_path;
	std::string input_mesh_path;
	std::string output_mesh_path;
	int flag_index = 1;
	bool using_builtin_defaults = false;

	if(positional_argc == 2)
	{
		using_builtin_defaults = true;
		input_mesh_path = argv[1];
		output_mesh_path = argv[2];
		flag_index = 3;
	}
	else
	{
		options_path = argv[1];
		input_mesh_path = argv[2];
		output_mesh_path = argv[3];
		flag_index = 4;
	}

	int export_precision = 16;
	std::string metrics_json_path;
	std::string metrics_csv_path;
	bool deterministic_cli = false;

	for(int i = flag_index; i < argc; ++i)
	{
		std::string arg = argv[i];
		if(arg == "--export-precision" && i + 1 < argc){
			export_precision = std::stoi(argv[++i]);
		}
		else if(arg == "--metrics-json" && i + 1 < argc){
			metrics_json_path = argv[++i];
		}
		else if(arg == "--metrics-csv" && i + 1 < argc){
			metrics_csv_path = argv[++i];
		}
		else if(arg == "--deterministic"){
			deterministic_cli = true;
		}
		else{
			std::cerr << "Unknown or incomplete argument: " << arg << std::endl;
			return 1;
		}
	}

	using Clock = std::chrono::steady_clock;
	auto app_begin = Clock::now();

	// Load input mesh
	TriMesh mesh;
	auto import_begin = Clock::now();
	if(!load_mesh(input_mesh_path, mesh))
	{
		return 1;
	}
	auto import_end = Clock::now();

	std::cout << "Loaded " << mesh.n_vertices() << " vertices, " << mesh.n_faces() << " faces." << std::endl;

	// Validate vertex coordinates
	if(!validate_mesh_vertices(mesh)){
		return 1;
	}

#ifdef USE_OPENMP
	Eigen::initParallel();
#endif

	// Load options
	SDFilter::MeshDenoisingParameters param;
	if(using_builtin_defaults)
	{
		apply_default_options(param);
		std::cout << "No option file specified; using built-in default denoising options." << std::endl;
	}
	else if(!param.load(options_path.c_str())){
		std::cerr << "Error: unable to load option file " << options_path << std::endl;
		return 1;
	}
	if(!param.valid_parameters()){
		std::cerr << "Invalid filter options. Aborting..." << std::endl;
		return 1;
	}
	if(deterministic_cli){
		param.deterministic_mode = true;
	}
	configure_deterministic_runtime(param.deterministic_mode);
	param.output();

	// Normalize the input mesh
	Eigen::Vector3d original_center;
	double original_scale;
	auto normalize_begin = Clock::now();
	SDFilter::normalize_mesh(mesh, original_center, original_scale);
	auto normalize_end = Clock::now();

	// Filter the normals and construct the output mesh
	SDFilter::MeshNormalDenoising denoiser(mesh);
	denoiser.set_print_progress(true);
	TriMesh output_mesh;
	auto denoise_begin = Clock::now();
	if(!denoiser.denoise(param, output_mesh)){
		std::cerr << "Error: denoising failed." << std::endl;
		return 1;
	}
	auto denoise_end = Clock::now();

	auto restore_begin = Clock::now();
	SDFilter::restore_mesh(output_mesh, original_center, original_scale);
	auto restore_end = Clock::now();

	// Save output mesh
	auto export_begin = Clock::now();
	if(!SDFilter::write_mesh(output_mesh, output_mesh_path, export_precision)){
		std::cerr << "Error: unable to save the result mesh to file " << output_mesh_path << std::endl;
		return 1;
	}
	auto export_end = Clock::now();
	auto app_end = Clock::now();

	// Collect and report metrics
	SDFilter::PipelineMetrics metrics;
	metrics.mode = "denoise";
	metrics.input_mesh = input_mesh_path;
	metrics.output_mesh = output_mesh_path;
	metrics.import_secs = std::chrono::duration<double>(import_end - import_begin).count();
	metrics.normalize_secs = std::chrono::duration<double>(normalize_end - normalize_begin).count();
	metrics.algorithm_secs = std::chrono::duration<double>(denoise_end - denoise_begin).count();
	metrics.restore_secs = std::chrono::duration<double>(restore_end - restore_begin).count();
	metrics.export_secs = std::chrono::duration<double>(export_end - export_begin).count();
	metrics.total_secs = std::chrono::duration<double>(app_end - app_begin).count();
	metrics.obj_export_precision = export_precision;

	std::cout << std::endl;
	std::cout << "====== Timing =========" << std::endl;
	std::cout << "Import:    " << metrics.import_secs << " s" << std::endl;
	std::cout << "Normalize: " << metrics.normalize_secs << " s" << std::endl;
	std::cout << "Denoise:   " << metrics.algorithm_secs << " s" << std::endl;
	std::cout << "Restore:   " << metrics.restore_secs << " s" << std::endl;
	std::cout << "Export:    " << metrics.export_secs << " s" << std::endl;
	std::cout << "Total:     " << metrics.total_secs << " s" << std::endl;
	std::cout << "=======================" << std::endl;

	const SDFilter::RunStatistics &stats = denoiser.run_stats();
	if(!metrics_json_path.empty()){
		if(!SDFilter::write_metrics_json(metrics_json_path, metrics, stats)){
			std::cerr << "Warning: unable to write metrics JSON file " << metrics_json_path << std::endl;
		}
	}

	if(!metrics_csv_path.empty()){
		bool write_header = !std::filesystem::exists(metrics_csv_path);
		if(!SDFilter::write_metrics_csv(metrics_csv_path, metrics, stats, write_header)){
			std::cerr << "Warning: unable to write metrics CSV file " << metrics_csv_path << std::endl;
		}
	}

	return 0;
}
