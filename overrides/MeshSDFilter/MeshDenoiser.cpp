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

#include <Eigen/Dense>
#include <OpenMesh/Core/IO/MeshIO.hh>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_USE_CPP14
#include <tiny_gltf.h>

#include "tinyusdz.hh"
#include "tydra/render-data.hh"
#include "tydra/scene-access.hh"

namespace
{

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

struct PrimitiveInstance
{
	const tinygltf::Primitive *primitive;
	Eigen::Matrix4d transform;
};

Eigen::Matrix4d compose_transform(const tinygltf::Node &node)
{
	if(node.matrix.size() == 16){
		Eigen::Matrix4d mat;
		for(int r = 0; r < 4; ++r){
			for(int c = 0; c < 4; ++c){
				mat(r, c) = node.matrix[c + 4 * r];
			}
		}
		return mat;
	}

	Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
	if(node.translation.size() == 3){
		translation(0, 3) = node.translation[0];
		translation(1, 3) = node.translation[1];
		translation(2, 3) = node.translation[2];
	}

	Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
	if(node.rotation.size() == 4){
		Eigen::Quaterniond q(node.rotation[3], node.rotation[0], node.rotation[1], node.rotation[2]);
		q.normalize();
		rotation.block<3,3>(0,0) = q.toRotationMatrix();
	}

	Eigen::Matrix4d scale = Eigen::Matrix4d::Identity();
	if(node.scale.size() == 3){
		scale(0,0) = node.scale[0];
		scale(1,1) = node.scale[1];
		scale(2,2) = node.scale[2];
	}

	return translation * rotation * scale;
}

void collect_primitives(const tinygltf::Model &model,
                        int node_index,
                        const Eigen::Matrix4d &parent,
                        std::vector<PrimitiveInstance> &instances)
{
	const tinygltf::Node &node = model.nodes[node_index];
	Eigen::Matrix4d world = parent * compose_transform(node);

	if(node.mesh >= 0 && node.mesh < static_cast<int>(model.meshes.size())){
		const tinygltf::Mesh &mesh = model.meshes[node.mesh];
		for(const tinygltf::Primitive &primitive : mesh.primitives){
			if(primitive.mode == TINYGLTF_MODE_TRIANGLES){
				instances.push_back({&primitive, world});
			}
		}
	}

	for(int child : node.children){
		if(child >= 0 && child < static_cast<int>(model.nodes.size())){
			collect_primitives(model, child, world, instances);
		}
	}
}

Eigen::Matrix4d default_transform()
{
	return Eigen::Matrix4d::Identity();
}

template<typename T>
const unsigned char *accessor_element_ptr(const tinygltf::Model &model,
                                          const tinygltf::Accessor &accessor,
                                          size_t index,
                                          size_t &stride_bytes)
{
	if(accessor.bufferView < 0 || accessor.bufferView >= static_cast<int>(model.bufferViews.size())){
		return nullptr;
	}

	const tinygltf::BufferView &buffer_view = model.bufferViews[accessor.bufferView];
	if(buffer_view.buffer < 0 || buffer_view.buffer >= static_cast<int>(model.buffers.size())){
		return nullptr;
	}

	const tinygltf::Buffer &buffer = model.buffers[buffer_view.buffer];
	size_t component_size = tinygltf::GetComponentSizeInBytes(accessor.componentType);
	size_t num_components = tinygltf::GetNumComponentsInType(accessor.type);

	size_t expected_stride = component_size * num_components;
	stride_bytes = accessor.ByteStride(buffer_view);
	if(stride_bytes == 0){
		stride_bytes = expected_stride;
	}

	size_t offset = accessor.byteOffset + buffer_view.byteOffset + index * stride_bytes;
	if(offset + expected_stride > buffer.data.size()){
		return nullptr;
	}

	return buffer.data.data() + offset;
}

bool read_position(const tinygltf::Model &model,
                   const tinygltf::Accessor &accessor,
                   size_t index,
                   Eigen::Vector3d &out)
{
	size_t stride = 0;
	const unsigned char *ptr = accessor_element_ptr<float>(model, accessor, index, stride);
	if(!ptr){
		return false;
	}

	const float *values = reinterpret_cast<const float *>(ptr);
	out = Eigen::Vector3d(static_cast<double>(values[0]),
	                      static_cast<double>(values[1]),
	                      static_cast<double>(values[2]));
	return true;
}

uint32_t read_index_element(const unsigned char *ptr, int component_type)
{
	switch(component_type){
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
			return static_cast<uint32_t>(*reinterpret_cast<const uint8_t*>(ptr));
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
			return static_cast<uint32_t>(*reinterpret_cast<const uint16_t*>(ptr));
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
			return *reinterpret_cast<const uint32_t*>(ptr);
		default:
			return 0;
	}
}

bool DummyLoadImage(tinygltf::Image*, const int, std::string*, std::string*, int, int, const unsigned char*, int, void*)
{
	return true;
}

bool load_gltf_mesh(const std::string &filename, TriMesh &mesh, std::string &warning, std::string &error)
{
	tinygltf::TinyGLTF loader;
	loader.SetImageLoader(DummyLoadImage, nullptr);
	tinygltf::Model model;

	bool is_binary = has_extension(filename, ".glb");
	bool success = false;
	std::string warn;
	std::string err;
	if(is_binary){
		success = loader.LoadBinaryFromFile(&model, &err, &warn, filename);
	}
	else{
		success = loader.LoadASCIIFromFile(&model, &err, &warn, filename);
	}

	if(!warn.empty()){
		warning += warn;
	}
	if(!err.empty()){
		error += err;
	}
	if(!success){
		if(error.empty()){
			error = "Unable to parse glTF file.";
		}
		return false;
	}

	std::vector<PrimitiveInstance> instances;
	Eigen::Matrix4d identity = default_transform();

	if(model.scenes.empty()){
		for(size_t i = 0; i < model.nodes.size(); ++i){
			collect_primitives(model, static_cast<int>(i), identity, instances);
		}
	}
	else{
		int scene_index = model.defaultScene >= 0 ? model.defaultScene : 0;
		if(scene_index < 0 || scene_index >= static_cast<int>(model.scenes.size())){
			scene_index = 0;
		}
		const tinygltf::Scene &scene = model.scenes[scene_index];
		for(int node_index : scene.nodes){
			if(node_index >= 0 && node_index < static_cast<int>(model.nodes.size())){
				collect_primitives(model, node_index, identity, instances);
			}
		}
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
		const tinygltf::Primitive &primitive = *instance.primitive;

		auto pos_it = primitive.attributes.find("POSITION");
		if(pos_it == primitive.attributes.end()){
			continue;
		}

		int position_accessor_index = pos_it->second;
		if(position_accessor_index < 0 || position_accessor_index >= static_cast<int>(model.accessors.size())){
			continue;
		}
		const tinygltf::Accessor &position_accessor = model.accessors[position_accessor_index];

		if(position_accessor.type != TINYGLTF_TYPE_VEC3 ||
		   position_accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT){
			error = "Unsupported POSITION accessor format in glTF.";
			return false;
		}

		if(primitive.indices < 0 || primitive.indices >= static_cast<int>(model.accessors.size())){
			error = "Indexed primitives are required in glTF meshes.";
			return false;
		}
		const tinygltf::Accessor &index_accessor = model.accessors[primitive.indices];

		if(index_accessor.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE &&
		   index_accessor.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT &&
		   index_accessor.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT){
			error = "Unsupported index component type in glTF primitive.";
			return false;
		}

		size_t index_stride = 0;
		const unsigned char *index_base = accessor_element_ptr<uint8_t>(model, index_accessor, 0, index_stride);
		if(!index_base){
			error = "Unable to read index data from glTF primitive.";
			return false;
		}
		if(index_stride == 0){
			index_stride = tinygltf::GetComponentSizeInBytes(index_accessor.componentType);
		}

		if(index_accessor.count % 3 != 0){
			warning += "glTF primitive indices are not a multiple of three; trailing vertices will be ignored.\n";
		}

		for(size_t tri = 0; tri + 2 < index_accessor.count; tri += 3){
			std::array<TriMesh::VertexHandle, 3> face_vertices;
			bool valid_triangle = true;

			for(int k = 0; k < 3; ++k){
				size_t vertex_index_offset = (tri + k) * index_stride;
				const unsigned char *index_ptr = index_base + vertex_index_offset;
				uint32_t vertex_index = read_index_element(index_ptr, index_accessor.componentType);

				if(vertex_index >= position_accessor.count){
					warning += "glTF index points outside POSITION accessor range; triangle skipped.\n";
					valid_triangle = false;
					break;
				}

				VertexKey key(instance_id, vertex_index);
				auto cache_it = vertex_cache.find(key);
				if(cache_it == vertex_cache.end()){
					Eigen::Vector3d position;
					if(!read_position(model, position_accessor, vertex_index, position)){
						warning += "Failed reading POSITION data from glTF accessor; triangle skipped.\n";
						valid_triangle = false;
						break;
					}

					Eigen::Vector4d p(position.x(), position.y(), position.z(), 1.0);
					Eigen::Vector4d transformed = instance.transform * p;
					TriMesh::Point point(static_cast<float>(transformed.x()),
					                     static_cast<float>(transformed.y()),
					                     static_cast<float>(transformed.z()));

					TriMesh::VertexHandle vh = mesh.add_vertex(point);
					cache_it = vertex_cache.emplace(key, vh).first;
				}

				face_vertices[k] = cache_it->second;
			}

			if(!valid_triangle){
				continue;
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

bool load_usd_mesh(const std::string &filename, TriMesh &mesh, std::string &warning, std::string &error)
{
	tinyusdz::Stage stage;
	std::string warn, err;

	// Load USD file based on extension
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
	else{
		error = "Unsupported USD file extension. Supported: .usdz, .usdc, .usda";
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

	// Convert USD stage to RenderScene using Tydra
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

	// Clear and prepare the output mesh
	mesh.clear();
	mesh.request_face_normals();
	mesh.request_vertex_normals();

	// Process all meshes in the render scene
	for(const auto &render_mesh : render_scene.meshes){
		if(render_mesh.points.empty()){
			continue;
		}

		// Add vertices
		std::vector<TriMesh::VertexHandle> vertex_handles;
		vertex_handles.reserve(render_mesh.points.size());

		for(const auto &point : render_mesh.points){
			TriMesh::Point p(point[0], point[1], point[2]);
			vertex_handles.push_back(mesh.add_vertex(p));
		}

		// Determine which indices to use
		const std::vector<uint32_t> *face_indices = nullptr;
		const std::vector<uint32_t> *face_counts = nullptr;

		if(!render_mesh.triangulatedFaceVertexIndices.empty()){
			// Use triangulated indices if available
			face_indices = &render_mesh.triangulatedFaceVertexIndices;
			face_counts = &render_mesh.triangulatedFaceVertexCounts;
		}
		else{
			// Use original indices
			face_indices = &render_mesh.usdFaceVertexIndices;
			face_counts = &render_mesh.usdFaceVertexCounts;
		}

		if(face_indices->empty() || face_counts->empty()){
			continue;
		}

		// Add faces
		size_t index_offset = 0;
		for(uint32_t count : *face_counts){
			if(count == 3){
				// Triangle - add directly
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
				// Polygon - triangulate by fan method
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
					// Simple fan triangulation from first vertex
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

} // anonymous namespace


int main(int argc, char **argv)
{
	if(argc != 4)
	{
		std::cout << "Usage:\tMeshDenoiser  OPTION_FILE  INPUT_MESH  OUTPUT_MESH" << std::endl;
		return 1;
	}

	const std::string option_file = argv[1];
	const std::string input_mesh_path = argv[2];
	const std::string output_mesh_path = argv[3];

	TriMesh mesh;
	bool load_success = false;
	std::string warning;
	std::string error;

	if(has_extension(input_mesh_path, ".gltf") || has_extension(input_mesh_path, ".glb")){
		load_success = load_gltf_mesh(input_mesh_path, mesh, warning, error);
	}
	else if(has_extension(input_mesh_path, ".usd") || has_extension(input_mesh_path, ".usda") ||
	        has_extension(input_mesh_path, ".usdc") || has_extension(input_mesh_path, ".usdz")){
		load_success = load_usd_mesh(input_mesh_path, mesh, warning, error);
	}
	else{
		load_success = OpenMesh::IO::read_mesh(mesh, input_mesh_path);
		if(!load_success){
			error = "unable to read input mesh with OpenMesh";
		}
	}

	if(!warning.empty()){
		std::cout << "Warning while loading mesh: " << warning << std::endl;
	}

	if(!load_success)
	{
		std::cerr << "Error: " << error << std::endl;
		return 1;
	}

#ifdef USE_OPENMP
	Eigen::initParallel();
#endif

	// Load option file
	SDFilter::MeshDenoisingParameters param;
	if(!param.load(option_file.c_str())){
		std::cerr << "Error: unable to load option file " << option_file << std::endl;
		return 1;
	}
	if(!param.valid_parameters()){
		std::cerr << "Invalid filter options. Aborting..." << std::endl;
		return 1;
	}
	param.output();


	// Normalize the input mesh
	Eigen::Vector3d original_center;
	double original_scale;
	SDFilter::normalize_mesh(mesh, original_center, original_scale);

	// Filter the normals and construct the output mesh
	SDFilter::MeshNormalDenoising denoiser(mesh);
	TriMesh output_mesh;
	if(!denoiser.denoise(param, output_mesh)){
		return 1;
	}

	SDFilter::restore_mesh(output_mesh, original_center, original_scale);

	// Save output mesh
	if(!SDFilter::write_mesh_high_accuracy(output_mesh, output_mesh_path)){
		std::cerr << "Error: unable to save the result mesh to file " << output_mesh_path << std::endl;
		return 1;
	}

	return 0;
}
