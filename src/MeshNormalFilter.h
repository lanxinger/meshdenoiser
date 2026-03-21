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


#ifndef MESHNORMALFILTER_H_
#define MESHNORMALFILTER_H_

#include "MeshTypes.h"
#include "SDFilter.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <array>

namespace SDFilter
{

class MeshFilterParameters : public Parameters
{
public:
	MeshFilterParameters()
	:mesh_update_method(ITERATIVE_UPDATE), mesh_update_closeness_weight(0.001), mesh_update_iter(20),
	 mesh_update_disp_eps(0.0)
	{
		// Use an threshold value corresponding to eps_angle_degree degrees change of normal vectors between two iterations
		double eps_angle_degree = 0.2;
		avg_disp_eps = 2 * std::sin(eps_angle_degree * 0.5 * M_PI / 180);
	}

	virtual ~MeshFilterParameters(){}

	// Methods for mesh vertex update according to filtered normals
	enum MeshUpdateMethod
	{
		ITERATIVE_UPDATE,	// ShapeUp styled iterative solver
		POISSON_UPDATE,	// Poisson-based update from [Wang et al. 2015]
		MESH_UPDATE_METHOD_COUNT
	};

	MeshUpdateMethod mesh_update_method;

	double mesh_update_closeness_weight;	// Weight for the closeness term in mesh update

	int mesh_update_iter;	// Number of mesh update iterations

	double mesh_update_disp_eps;	// Early-stop threshold on RMS vertex displacement per mesh-update iteration; <=0 disables early stop

	virtual bool valid_parameters() const
	{
		if(!Parameters::valid_parameters()){
			return false;
		}

		if(mesh_update_iter <= 0){
			std::cerr << "Error: MeshUpdateIterations must be positive" << std::endl;
			return false;
		}

		if(mesh_update_closeness_weight < 0){
			std::cerr << "Error: MeshUpdateClosenessWeight must be positive" << std::endl;
			return false;
		}

		if(mesh_update_disp_eps < 0.0){
			std::cerr << "Error: MeshUpdateDisplacementEps must be non-negative" << std::endl;
			return false;
		}

		if(mesh_update_method < ITERATIVE_UPDATE || mesh_update_method >= MESH_UPDATE_METHOD_COUNT){
			std::cerr << "Error: invalid MeshUpdateMethod" << std::endl;
			return false;
		}

		return true;
	}

protected:
	virtual bool load_option(const OptionInterpreter &opt)
	{
		return Parameters::load_option(opt) ||
				opt.load_enum("MeshUpdateMethod", MESH_UPDATE_METHOD_COUNT, mesh_update_method) ||
				opt.load("MeshUpdateClosenessWeight", mesh_update_closeness_weight) ||
				opt.load("MeshUpdateIterations", mesh_update_iter) ||
				opt.load("MeshUpdateDisplacementEps", mesh_update_disp_eps);
	}

	virtual void output_options()
	{
		Parameters::output_options();
		std::cout << "Mesh update method: " << static_cast<int>(mesh_update_method) << std::endl;
		std::cout << "Mesh update closeness weight: " << mesh_update_closeness_weight << std::endl;
		std::cout << "Mesh update iterations: " << mesh_update_iter << std::endl;
		std::cout << "Mesh update displacement eps: " << mesh_update_disp_eps << std::endl;
	}
};

class MeshNormalFilter : public SDFilter
{
public:

	MeshNormalFilter(const TriMesh &mesh)
	:mesh_(mesh), print_error_evaluation_(false), linear_solver_(Parameters::LDLT), system_matrix_factorized_(false),
	 mesh_update_system_closeness_weight_(std::numeric_limits<double>::quiet_NaN()),
	 mesh_update_system_n_faces_(-1), mesh_update_system_n_vtx_(-1),
	 cached_face_vtx_idx_valid_(false), cached_face_vtx_n_faces_(-1), cached_face_vtx_n_vtx_(-1){}

	virtual ~MeshNormalFilter() {}

	// Pass param by value, to allow changing the value of eta
	bool filter(MeshFilterParameters param, TriMesh &output_mesh)
	{
		assert(param.valid_parameters());
		linear_solver_.set_solver_type(param.linear_solver_type);
		linear_solver_.set_solver_params(param.linear_solver_max_iterations, param.linear_solver_tolerance);

		Timer timer;
		Timer::EventID mesh_flter_begin_time = timer.get_time();

		// Rescale the eta parameter, according to average distance between neighboring face centroids
		param.eta *= average_neighbor_face_centroid_dist(mesh_);

		if(!SDFilter::filter(param)){
			std::cerr << "Error in performing SD filter" << std::endl;
			return false;
		}


		Timer::EventID update_begin_time = timer.get_time();

		// Normalize the filtered face normals
		Matrix3X target_normals = signals_.block(0, 0, 3, signals_.cols());
		target_normals.colwise().normalize();

		if(param.mesh_update_method == MeshFilterParameters::ITERATIVE_UPDATE){
			if(!iterative_mesh_update(param, target_normals, output_mesh)){
				std::cerr << "Error in iteative mesh update" << std::endl;
				return false;
			}
		}
		else{
			if(!Poisson_mesh_update(target_normals, output_mesh)){
				std::cerr << "Error in Poisson mesh update" << std::endl;
				return false;
			}
		}

		Timer::EventID update_end_time = timer.get_time();
		stats_.mesh_update_secs = timer.elapsed_time(update_begin_time, update_end_time);
		stats_.mesh_filter_total_secs = timer.elapsed_time(mesh_flter_begin_time, update_end_time);

		if(print_timing_){
			std::cout << "Mesh udpate timing: " << stats_.mesh_update_secs << " secs" << std::endl;
			std::cout << "Mesh filter total timing: " << stats_.mesh_filter_total_secs << " secs" << std::endl;
		}

		if(print_error_evaluation_)
		{
			std::cout << std::endl;
			show_normalized_mesh_displacement_norm(output_mesh);
			show_normal_error_statistics(output_mesh, target_normals, 2, 10);
		}

		return true;
	}

protected:

	TriMesh mesh_;

	bool print_error_evaluation_;	// The printing of mesh update error

	bool get_neighborhood(const Parameters &param, Eigen::Matrix2Xi &neighbor_pairs, Eigen::VectorXd &neighbor_dist)
	{
		const double radius = 3.0 * param.eta;
		const double radius_sqr = radius * radius;
		const int n_faces = mesh_.n_faces();
		if(n_faces <= 0){
			return false;
		}

		Matrix3X face_centroids(3, n_faces);
		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_faces; ++i){
				face_centroids.col(i) = to_eigen_vec3d(mesh_.calc_face_centroid(TriMesh::FaceHandle(i)));
			}
		}

		using CellKey = std::array<int, 3>;
		struct CellKeyHash
		{
			std::size_t operator()(const CellKey &k) const
			{
				std::size_t h = static_cast<std::size_t>(k[0]) * 73856093u;
				h ^= static_cast<std::size_t>(k[1]) * 19349663u;
				h ^= static_cast<std::size_t>(k[2]) * 83492791u;
				return h;
			}
		};

		Eigen::Vector3d min_c = face_centroids.rowwise().minCoeff();
		std::unordered_map<CellKey, std::vector<int>, CellKeyHash> cells;
		cells.reserve(static_cast<std::size_t>(n_faces));

		auto centroid_cell = [&](const Eigen::Vector3d &p) -> CellKey
		{
			Eigen::Vector3d q = (p - min_c) / radius;
			return CellKey{
				static_cast<int>(std::floor(q[0])),
				static_cast<int>(std::floor(q[1])),
				static_cast<int>(std::floor(q[2]))
			};
		};

		for(int i = 0; i < n_faces; ++i){
			cells[centroid_cell(face_centroids.col(i))].push_back(i);
		}

		std::vector<std::pair<int, int>> pairs;
		pairs.reserve(static_cast<std::size_t>(n_faces) * 12u);

		for(int i = 0; i < n_faces; ++i)
		{
			const Eigen::Vector3d ci = face_centroids.col(i);
			const CellKey c = centroid_cell(ci);

			for(int dx = -1; dx <= 1; ++dx)
			{
				for(int dy = -1; dy <= 1; ++dy)
				{
					for(int dz = -1; dz <= 1; ++dz)
					{
						CellKey nkey{c[0] + dx, c[1] + dy, c[2] + dz};
						auto it = cells.find(nkey);
						if(it == cells.end()){
							continue;
						}

						const std::vector<int> &bucket = it->second;
						for(std::size_t k = 0; k < bucket.size(); ++k)
						{
							const int j = bucket[k];
							if(j <= i){
								continue;
							}

							if((ci - face_centroids.col(j)).squaredNorm() < radius_sqr){
								pairs.emplace_back(i, j);
							}
						}
					}
				}
			}
		}

		const Eigen::Index n_neighbor_pairs = static_cast<Eigen::Index>(pairs.size());
		if(n_neighbor_pairs <= 0){
			return false;
		}

		neighbor_pairs.resize(2, n_neighbor_pairs);
		neighbor_dist.resize(n_neighbor_pairs);
		for(Eigen::Index p = 0; p < n_neighbor_pairs; ++p)
		{
			const int i = pairs[static_cast<std::size_t>(p)].first;
			const int j = pairs[static_cast<std::size_t>(p)].second;
			neighbor_pairs(0, p) = i;
			neighbor_pairs(1, p) = j;
			neighbor_dist(p) = (face_centroids.col(i) - face_centroids.col(j)).norm();
		}

		return true;
	}


	void get_initial_data(Eigen::MatrixXd &guidance, Eigen::MatrixXd &init_signals, Eigen::VectorXd &area_weights)
	{
		init_signals.resize(3, mesh_.n_faces());

		for(TriMesh::ConstFaceIter cf_it = mesh_.faces_begin(); cf_it != mesh_.faces_end(); ++ cf_it)
		{
			Eigen::Vector3d f_normal = to_eigen_vec3d(mesh_.calc_face_normal(*cf_it)).normalized();
			init_signals.col(cf_it->idx()) = f_normal;
		}

		guidance = init_signals;

		get_face_area_weights(mesh_, area_weights);
	}


	void reset_mesh_update_system()
	{
		system_matrix_factorized_ = false;
		mesh_update_system_closeness_weight_ = std::numeric_limits<double>::quiet_NaN();
		mesh_update_system_n_faces_ = -1;
		mesh_update_system_n_vtx_ = -1;
	}


	void set_mesh(const TriMesh &mesh, bool invalidate_update_system)
	{
		mesh_ = mesh;
		cached_face_vtx_idx_valid_ = false;
		cached_face_vtx_n_faces_ = -1;
		cached_face_vtx_n_vtx_ = -1;

		if(invalidate_update_system){
			reset_mesh_update_system();
		}
	}



private:

	// Pre-computed information for mesh update problem
	//	\min_X  ||AX - B||^2 + w ||X - X0||^2,
	// where X are new mesh vertex positions, A is a mean-centering matrix, B are the target mean-centered positions,
	// X0 are initial vertex positions, and w > 0 is a closeness weight.
	// This amounts to solving a linear system
	//	(A^T A + w I) X = A^T B + w X0.
	// We precompute matrix A^T, and pre-factorize A^T A + w I
	LinearSolver linear_solver_;	// Linear system solver for mesh update
	SparseMatrixXd At_;		// Transpose of part of the linear least squares matrix that corresponds to mean centering of face vertices
	bool system_matrix_factorized_;	// Whether the matrix
	double mesh_update_system_closeness_weight_;
	int mesh_update_system_n_faces_;
	int mesh_update_system_n_vtx_;
	Matrix3Xi cached_face_vtx_idx_;
	bool cached_face_vtx_idx_valid_;
	int cached_face_vtx_n_faces_;
	int cached_face_vtx_n_vtx_;




	// Set up and pre-factorize the linear system for iterative mesh update
	bool setup_mesh_udpate_system(const Matrix3Xi &face_vtx_idx, double w_closeness)
	{
		const int n_faces = mesh_.n_faces();
		const int n_vtx = mesh_.n_vertices();
		const bool same_problem_size = (mesh_update_system_n_faces_ == n_faces && mesh_update_system_n_vtx_ == n_vtx);
		const bool same_closeness = (std::isfinite(mesh_update_system_closeness_weight_) &&
			std::abs(mesh_update_system_closeness_weight_ - w_closeness) <= std::numeric_limits<double>::epsilon());
		if(system_matrix_factorized_ && same_problem_size && same_closeness)
		{
			return true;
		}

		std::vector<Triplet> A_triplets(9 * n_faces);
		std::vector<Triplet> I_triplets(n_vtx);

		// Matrix for mean centering of three vertices
		Eigen::Matrix3d mean_centering_mat;
		get_mean_centering_matrix(mean_centering_mat);

		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				Eigen::Vector3i vtx_idx = face_vtx_idx.col(i);

				int triplet_addr = 9 * i;
				int row_idx = 3 * i;
				for(int j = 0; j < 3; ++ j)
				{
					for(int k = 0; k < 3; ++ k){
						A_triplets[triplet_addr++] = Triplet(row_idx, vtx_idx(k), mean_centering_mat(j, k));
					}

					row_idx++;
				}
			}

			OMP_FOR
			for(int i = 0; i < n_vtx; ++ i)
			{
				I_triplets[i] = Triplet(i, i, w_closeness);
			}
		}


		SparseMatrixXd A(3 * n_faces, n_vtx);
		A.setFromTriplets(A_triplets.begin(), A_triplets.end());
		At_ = A.transpose();
		At_.makeCompressed();

		SparseMatrixXd wI(n_vtx, n_vtx);
		wI.setFromTriplets(I_triplets.begin(), I_triplets.end());
		SparseMatrixXd M = At_ * A  + wI;

		linear_solver_.reset_pattern();
		if(!linear_solver_.compute(M)){
			std::cerr << "Error: failed to pre-factorize mesh update system" << std::endl;
			return false;
		}

		system_matrix_factorized_ = true;
		mesh_update_system_closeness_weight_ = w_closeness;
		mesh_update_system_n_faces_ = n_faces;
		mesh_update_system_n_vtx_ = n_vtx;
		return true;
	}

	void get_cached_face_vertex_indices(const TriMesh &mesh, Matrix3Xi &face_vtx_idx)
	{
		const int n_faces = mesh.n_faces();
		const int n_vtx = mesh.n_vertices();
		if(!cached_face_vtx_idx_valid_ || cached_face_vtx_n_faces_ != n_faces || cached_face_vtx_n_vtx_ != n_vtx)
		{
			get_face_vertex_indices(mesh, cached_face_vtx_idx_);
			cached_face_vtx_idx_valid_ = true;
			cached_face_vtx_n_faces_ = n_faces;
			cached_face_vtx_n_vtx_ = n_vtx;
		}
		face_vtx_idx = cached_face_vtx_idx_;
	}


	void get_face_area_weights(const TriMesh &mesh, Eigen::VectorXd &face_area_weights) const
	{
		face_area_weights.resize(mesh.n_faces());

		for(TriMesh::ConstFaceIter cf_it = mesh.faces_begin(); cf_it != mesh.faces_end(); ++ cf_it)
		{
			face_area_weights(cf_it->idx()) = mesh.calc_sector_area(mesh.halfedge_handle(*cf_it));
		}

		face_area_weights /= face_area_weights.mean();
	}


	bool iterative_mesh_update(const MeshFilterParameters &param, const Matrix3X &target_normals, TriMesh &output_mesh)
	{
		// Rescale closeness weight using the ratio between face number and vertex number, and take its square root
		double w_closeness = param.mesh_update_closeness_weight * double(mesh_.n_faces()) / mesh_.n_vertices();

		output_mesh = mesh_;

		Matrix3Xi face_vtx_idx;
		get_cached_face_vertex_indices(output_mesh, face_vtx_idx);

		if(!setup_mesh_udpate_system(face_vtx_idx, w_closeness)){
			return false;
		}

		std::cout << "Starting iterative mesh update......" << std::endl;

		Matrix3X vtx_pos;
		get_vertex_points(output_mesh, vtx_pos);

		int n_faces = output_mesh.n_faces();
		Eigen::Matrix3Xd target_plane_local_frames(3, 2 * n_faces);	// Local frame for the target plane of each face
		std::vector<bool> local_frame_initialized(n_faces, false);

		using MatrixX3dRM = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
		MatrixX3dRM wX0 = vtx_pos.transpose() * w_closeness;	// Part of the linear system right-hand-side that corresponds to initial vertex positions
		MatrixX3dRM B(3 * n_faces, 3);	// Per-face target position of the new vertices

		int n_vtx = output_mesh.n_vertices();
		MatrixX3dRM rhs(n_vtx, 3), sol(n_vtx, 3);

		for(int iter = 0; iter < param.mesh_update_iter; ++ iter)
		{
			OMP_PARALLEL
			{
				OMP_FOR
				for(int i = 0; i < n_faces; ++ i)
				{
					Eigen::Vector3d current_normal = to_eigen_vec3d(output_mesh.calc_face_normal(TriMesh::FaceHandle(i)));
					Eigen::Vector3d target_normal = target_normals.col(i);

					Eigen::Matrix3d face_vtx_pos;
					get_mean_centered_face_vtx_pos(vtx_pos, face_vtx_idx.col(i), face_vtx_pos);

					Eigen::Matrix3d target_pos;

					// If the current normal is not pointing away from the target normal, simply project the points onto the target plane
					if(current_normal.dot(target_normal) >= 0){
						target_pos = face_vtx_pos - target_normal * (target_normal.transpose() * face_vtx_pos);
					}
					else{
						// Otherwise, project the points onto a line in the target plane
						typedef Eigen::Matrix<double, 3, 2> Matrix32d;
						Matrix32d current_local_frame;
						if(local_frame_initialized[i]){
							current_local_frame = target_plane_local_frames.block(0, 2*i, 3, 2);
						}
						else{
							Eigen::JacobiSVD<Eigen::Vector3d, Eigen::FullPivHouseholderQRPreconditioner> jSVD_normal(target_normal, Eigen::ComputeFullU);
							current_local_frame = jSVD_normal.matrixU().block(0, 1, 3, 2);
							target_plane_local_frames.block(0, 2*i, 3, 2) = current_local_frame;
							local_frame_initialized[i] = true;
						}

						Matrix32d local_coord = face_vtx_pos.transpose() * current_local_frame;
						Eigen::JacobiSVD<Matrix32d> jSVD_coord(local_coord, Eigen::ComputeFullV);
						Eigen::Vector2d fitting_line_direction = jSVD_coord.matrixV().col(0);
						Eigen::Vector3d line_direction_3d = current_local_frame * fitting_line_direction;
						target_pos = line_direction_3d * (line_direction_3d.transpose() * face_vtx_pos);
					}

					B.block(3 * i, 0, 3, 3) = target_pos.transpose();
				}
			}

			// Solver linear system
			rhs = wX0;
			rhs.noalias() += At_ * B;
			if(!linear_solver_.solve(rhs, sol)){
				std::cerr << "Error: failed to solve mesh update system" << std::endl;
				return false;
			}

			const Matrix3X prev_vtx_pos = vtx_pos;
			vtx_pos = sol.transpose();
			set_vertex_points(output_mesh, vtx_pos);

			if(param.mesh_update_disp_eps > 0.0)
			{
				const double rms_disp = std::sqrt((vtx_pos - prev_vtx_pos).colwise().squaredNorm().mean());
				if(rms_disp <= param.mesh_update_disp_eps)
				{
					if(print_progress_){
						std::cout << "Mesh update converged after " << (iter + 1)
								  << " iterations (rms displacement " << rms_disp << ")" << std::endl;
					}
					break;
				}
			}
		}

		return true;
	}


	bool Poisson_mesh_update(const Matrix3X &target_normals, TriMesh &output_mesh)
	{
		output_mesh = mesh_;

		Matrix3Xi face_vtx_idx;
		get_face_vertex_indices(output_mesh, face_vtx_idx);

		std::cout << "Starting Poisson mesh update......" << std::endl;

		Matrix3X vtx_pos;
		get_vertex_points(output_mesh, vtx_pos);

		int n_faces = output_mesh.n_faces();
		int n_vtx = output_mesh.n_vertices();
		Eigen::VectorXd face_area_weights;
		get_face_area_weights(output_mesh, face_area_weights);

		// Compute the initial centroid
		Eigen::Vector3d initial_centroid = compute_centroid(face_vtx_idx, face_area_weights, vtx_pos);

		// Set up the linear least squares system
		SparseMatrixXd A(3*n_faces + 1, n_vtx);
		Eigen::MatrixX3d B(3*n_faces + 1, 3);
		std::vector<Triplet> A_triplets(9 * n_faces + 1);

		// Set the target position of the first vertex at the origin
		A_triplets.back() = Triplet(3*n_faces, 0, 1.0);
		B.row(3*n_faces).setZero();

		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				// Compute rotation from current face to the target plane
				Eigen::Vector3d init_normal = to_eigen_vec3d(output_mesh.calc_face_normal(TriMesh::FaceHandle(i)));
				Eigen::Vector3d target_normal = target_normals.col(i);
				Eigen::Quaternion<double> rot = Eigen::Quaternion<double>::FromTwoVectors(init_normal, target_normal);

				Eigen::Vector3i vtx_idx = face_vtx_idx.col(i);
				Eigen::Matrix3d face_vtx_pos;
				for(int j = 0; j < 3; ++ j){
					face_vtx_pos.col(j) = vtx_pos.col(vtx_idx(j));
				}

				double area_weight = std::sqrt(face_area_weights(i));

				for(int j = 0; j < 3; ++ j)
				{
					// Search for coefficients such that
					// [v1 - (a * v2 + (1 - a) * v3)] * (v2 - v3) = 0
					// ==> a = [(v1 - v3) * (v2 - v3)] / ||v2 - v3||^2
					// Then the gradient coefficients become (1, -a, a - 1)

					int i1 = j, i2 = (j+1)%3, i3 = (j+2)%3;

					Eigen::Vector3d v1 = face_vtx_pos.col(i1),
							v2 = face_vtx_pos.col(i2),
							v3 = face_vtx_pos.col(i3);

					double a = 0.5;
					if((v2 - v3).norm() > 1e-12){
						a = (v1 - v3).dot(v2 - v3) / (v2 - v3).squaredNorm();
					}

					// Compute gradient coefficient w.r.t vertex positions
					Eigen::Vector3d current_grad_coef;
					current_grad_coef(i1) = 1.0;
					current_grad_coef(i2) = -a;
					current_grad_coef(i3) = a - 1.0;
					current_grad_coef *= area_weight;

					// Fill gradient coefficients into matrix
					int triplet_addr = i*9 + j*3;
					int row_idx = 3*i + j;
					A_triplets[triplet_addr++] = Triplet(row_idx, vtx_idx(i1), current_grad_coef(i1));
					A_triplets[triplet_addr++] = Triplet(row_idx, vtx_idx(i2), current_grad_coef(i2));
					A_triplets[triplet_addr] = Triplet(row_idx, vtx_idx(i3), current_grad_coef(i3));

					// Put the target gradient on the right-hand-side
					Eigen::Vector3d target_grad = rot * (face_vtx_pos * current_grad_coef);
					B.row(row_idx) = target_grad.transpose();
				}
			}
		}

		// Solver linear system
		A.setFromTriplets(A_triplets.begin(), A_triplets.end());
		SparseMatrixXd At = A.transpose();
		SparseMatrixXd M = At * A;
		Eigen::MatrixX3d rhs = At * B;
		Eigen::MatrixX3d sol(n_vtx, 3);

		linear_solver_.reset_pattern();
		if(!(linear_solver_.compute(M) && linear_solver_.solve(rhs, sol))){
			std::cerr << "Error: failed to solve linear system for Poisson mesh update" << std::endl;
			return false;
		}

		// Align the new mesh with the intial mesh
		vtx_pos = sol.transpose();
		Eigen::Vector3d new_centroid = compute_centroid(face_vtx_idx, area_weights_, vtx_pos);
		vtx_pos.colwise() += initial_centroid - new_centroid;
		set_vertex_points(output_mesh, vtx_pos);


		return true;
	}



	// Generate the matrix for mean-centering of the vertices of a triangle
	void get_mean_centering_matrix(Eigen::Matrix3d &mat)
	{
		mat = Eigen::Matrix3d::Identity() - Eigen::Matrix3d::Constant(1.0/3);
	}

	void get_mean_centered_face_vtx_pos(const Eigen::Matrix3Xd &vtx_pos, const Eigen::Vector3i &face_vtx, Eigen::Matrix3d &face_vtx_pos)
	{
		for(int i = 0; i < 3; ++ i){
			face_vtx_pos.col(i) = vtx_pos.col(face_vtx(i));
		}

		Eigen::Vector3d mean_pt = face_vtx_pos.rowwise().mean();
		face_vtx_pos.colwise() -= mean_pt;
	}

	// Compute the centroid of a mesh given its vertex positions and face areas
	Eigen::Vector3d compute_centroid(const Eigen::Matrix3Xi &face_vtx_idx, const Eigen::VectorXd &face_areas, const Eigen::Matrix3Xd &vtx_pos)
	{
		int n_faces = face_vtx_idx.cols();
		Eigen::Matrix3Xd face_centroids(3, n_faces);

		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				Eigen::Vector3d c = Eigen::Vector3d::Zero();
				Eigen::Vector3i face_vtx = face_vtx_idx.col(i);

				for(int j = 0; j < 3; ++ j){
					c += vtx_pos.col(face_vtx(j));
				}

				face_centroids.col(i) = c / 3.0;
			}
		}

		return (face_centroids * face_areas) / face_areas.sum();
	}


	////////// Methods for evaluating the quality of the updated mesh /////////////

	// Compute the L2 norm between the initial mesh and filtered mesh
	void show_normalized_mesh_displacement_norm(const TriMesh &filtered_mesh)
	{
		Eigen::Matrix3Xd init_vtx_pos, new_vtx_pos;
		get_vertex_points(mesh_, init_vtx_pos);
		get_vertex_points(filtered_mesh, new_vtx_pos);
		Eigen::VectorXd vtx_disp_sqr_norm = (init_vtx_pos - new_vtx_pos).colwise().squaredNorm();

		// Computer normalized vertex area weights from the original mesh
		Eigen::VectorXd face_area_weights;
		get_face_area_weights(mesh_, face_area_weights);
		Eigen::Matrix3Xi face_vtx_indices;
		get_face_vertex_indices(mesh_, face_vtx_indices);
		int n_faces = mesh_.n_faces();

		Eigen::VectorXd vtx_area(mesh_.n_vertices());
		vtx_area.setZero();
		for(int i = 0; i < n_faces; ++ i){
			for(int j = 0; j < 3; ++ j){
				vtx_area(face_vtx_indices(j, i)) += face_area_weights(i);
			}
		}
		vtx_area /= vtx_area.sum();

		std::cout << "Normalized mesh displacement norm: " <<
				std::sqrt(vtx_area.dot(vtx_disp_sqr_norm)) / average_edge_length(mesh_) << std::endl;
	}


	void show_error_statistics(const Eigen::VectorXd &err_values, double bin_size, int n_bins)
	{
		int n_elems = err_values.size();

		Eigen::VectorXi error_bin_idx(n_elems);
		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_elems; ++ i){
				error_bin_idx(i) = std::min(n_bins, static_cast<int>(std::floor(err_values(i) / bin_size)));
			}
		}

		Eigen::VectorXd bin_count(n_bins + 1);
		bin_count.setZero();

		for(int i = 0; i < n_elems; ++ i)
		{
			bin_count( error_bin_idx(i) ) += 1;
		}

		bin_count /= bin_count.sum();

		for(int i = 0; i < n_bins; ++ i)
		{
			double lower_val = bin_size * i;
			double upper_val = bin_size * (i+1);
			std::cout << lower_val << " to " << upper_val << ": " << bin_count(i) * 100 << "%" << std::endl;
		}

		std::cout << "Over " << bin_size * n_bins << ": " << bin_count(n_bins) * 100 << "%" << std::endl;
	}

	// Show statistics of the deviation between the new normals and target normals (in degrees)
	void show_normal_error_statistics(const TriMesh &mesh, const Matrix3X &target_normals, int bin_size_in_degrees, int n_bins)
	{
		// Compute the normal deviation angle, and the number of flipped normals
		int n_faces = mesh.n_faces();
		Eigen::VectorXd face_normal_error_angle(n_faces);

		for(int i = 0; i < n_faces; ++ i)
		{
			Eigen::Vector3d normal = to_eigen_vec3d(mesh.calc_face_normal(TriMesh::FaceHandle(i)));
			double error_angle_cos = std::max(-1.0, std::min(1.0, normal.dot(target_normals.col(i))));
			face_normal_error_angle(i) = std::acos(error_angle_cos);
		}

		face_normal_error_angle *= (180 / M_PI);

		std::cout << "Statistics of deviation between new normals and target normals:" << std::endl;
		std::cout << "===============================================================" << std::endl;
		show_error_statistics(face_normal_error_angle, bin_size_in_degrees, n_bins);
	}
};

}



#endif /* MESHNORMALFILTER_H_ */
