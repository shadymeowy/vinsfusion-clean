#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <onnxruntime_cxx_api.h>

#define NETVLAD_DIM 4096

class NetVLADDB {
    public:
	NetVLADDB(const std::string &model_path, int history_size = 1024);
	~NetVLADDB();

	// Runs the model on `image` (HWC, CV_8UC3, 224x224), returns top-k indices/values,
	// then inserts the descriptor into the cyclic buffer (no normalization).
	void query(const cv::Mat &image, std::vector<int> &best_idx,
		   std::vector<float> &best_values, bool add_to_history = true);

    private:
	// ORT objects
	Ort::Env env_;
	Ort::SessionOptions so_;
	Ort::Session session_;
	Ort::AllocatorWithDefaultOptions alloc_;
	Ort::MemoryInfo mem_info_cpu_;

	// Input / output names (discovered from the model)
	std::vector<std::string> input_names_;
	std::vector<std::string> output_names_;

	// Cyclic descriptor buffer (fixed size)
	const int history_size_;
	std::vector<float> descs_; // history_size_ x NETVLAD_DIM, row-major
	int write_pos_; // next row to overwrite [0, history_size_-1]
};
