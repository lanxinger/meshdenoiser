#ifndef APPMETRICS_H
#define APPMETRICS_H

#include "SDFilter.h"

#include <fstream>
#include <string>

namespace SDFilter
{

inline std::string json_escape(const std::string &s)
{
	std::string out;
	out.reserve(s.size());
	for(char c : s)
	{
		switch(c)
		{
		case '\\': out += "\\\\"; break;
		case '\"': out += "\\\""; break;
		case '\n': out += "\\n"; break;
		case '\r': out += "\\r"; break;
		case '\t': out += "\\t"; break;
		default: out.push_back(c); break;
		}
	}
	return out;
}

struct PipelineMetrics
{
	std::string mode;
	std::string input_mesh;
	std::string output_mesh;
	double import_secs = 0.0;
	double normalize_secs = 0.0;
	double algorithm_secs = 0.0;
	double restore_secs = 0.0;
	double export_secs = 0.0;
	double total_secs = 0.0;
	int obj_export_precision = 16;
};

inline bool write_metrics_json(const std::string &path, const PipelineMetrics &m, const RunStatistics &s)
{
	std::ofstream out(path.c_str());
	if(!out.is_open()){
		return false;
	}

	out << "{\n";
	out << "  \"mode\": \"" << json_escape(m.mode) << "\",\n";
	out << "  \"input_mesh\": \"" << json_escape(m.input_mesh) << "\",\n";
	out << "  \"output_mesh\": \"" << json_escape(m.output_mesh) << "\",\n";
	out << "  \"timing\": {\n";
	out << "    \"import_secs\": " << m.import_secs << ",\n";
	out << "    \"normalize_secs\": " << m.normalize_secs << ",\n";
	out << "    \"algorithm_secs\": " << m.algorithm_secs << ",\n";
	out << "    \"restore_secs\": " << m.restore_secs << ",\n";
	out << "    \"export_secs\": " << m.export_secs << ",\n";
	out << "    \"total_secs\": " << m.total_secs << ",\n";
	out << "    \"preprocessing_secs\": " << s.preprocessing_secs << ",\n";
	out << "    \"filtering_secs\": " << s.filtering_secs << ",\n";
	out << "    \"mesh_update_secs\": " << s.mesh_update_secs << ",\n";
	out << "    \"mesh_filter_total_secs\": " << s.mesh_filter_total_secs << ",\n";
	out << "    \"denoise_total_secs\": " << s.denoise_total_secs << "\n";
	out << "  },\n";
	out << "  \"solver\": {\n";
	out << "    \"iterations\": " << s.solver_iterations << ",\n";
	out << "    \"converged\": " << (s.solver_converged ? "true" : "false") << ",\n";
	out << "    \"outer_iterations\": " << s.outer_iterations << "\n";
	out << "  },\n";
	out << "  \"io\": {\n";
	out << "    \"obj_export_precision\": " << m.obj_export_precision << "\n";
	out << "  }\n";
	out << "}\n";

	return out.good();
}

inline bool write_metrics_csv(
	const std::string &path,
	const PipelineMetrics &m,
	const RunStatistics &s,
	bool write_header)
{
	std::ofstream out;
	if(write_header){
		out.open(path.c_str(), std::ios::out | std::ios::trunc);
	}
	else{
		out.open(path.c_str(), std::ios::out | std::ios::app);
	}

	if(!out.is_open()){
		return false;
	}

	if(write_header){
		out << "mode,input_mesh,output_mesh,import_secs,normalize_secs,algorithm_secs,restore_secs,export_secs,total_secs,"
			<< "preprocessing_secs,filtering_secs,mesh_update_secs,mesh_filter_total_secs,denoise_total_secs,"
			<< "solver_iterations,solver_converged,outer_iterations,"
			<< "obj_export_precision\n";
	}

	out << m.mode << ","
		<< m.input_mesh << ","
		<< m.output_mesh << ","
		<< m.import_secs << ","
		<< m.normalize_secs << ","
		<< m.algorithm_secs << ","
		<< m.restore_secs << ","
		<< m.export_secs << ","
		<< m.total_secs << ","
		<< s.preprocessing_secs << ","
		<< s.filtering_secs << ","
		<< s.mesh_update_secs << ","
		<< s.mesh_filter_total_secs << ","
		<< s.denoise_total_secs << ","
		<< s.solver_iterations << ","
		<< (s.solver_converged ? 1 : 0) << ","
		<< s.outer_iterations << ","
		<< m.obj_export_precision << "\n";

	return out.good();
}

}

#endif // APPMETRICS_H
