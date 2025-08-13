#include "netvladdb.h"

#include <cstring>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <random>
#include <stdexcept>

NetVLADDB::NetVLADDB(const std::string &model_path, int history_size)
	: env_(ORT_LOGGING_LEVEL_WARNING, "NetVLADDB")
	, so_()
	, session_(nullptr)
	, mem_info_cpu_(Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator,
						   OrtMemTypeDefault))
	, history_size_(history_size)
	, descs_(static_cast<size_t>(history_size) * NETVLAD_DIM,
		 0.0f)
	, // no normalization
	write_pos_(0)
{
	if (history_size_ <= 0) {
		throw std::runtime_error("NetVLADDB: history_size must be > 0");
	}

	// Session options
	so_.SetIntraOpNumThreads(1);
	so_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

	// CUDA EP (ORT 1.22)
	OrtCUDAProviderOptions cuda_options{};
	cuda_options.device_id = 0;
	cuda_options.arena_extend_strategy = 0;
	cuda_options.gpu_mem_limit = std::numeric_limits<size_t>::max();
	cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchDefault;
	cuda_options.do_copy_in_default_stream = 1;
	so_.AppendExecutionProvider_CUDA(cuda_options);

	// Create session
	session_ = Ort::Session(env_, model_path.c_str(), so_);

	// Discover input/output names
	size_t num_inputs = session_.GetInputCount();
	size_t num_outputs = session_.GetOutputCount();
	input_names_.reserve(num_inputs);
	output_names_.reserve(num_outputs);

	for (size_t i = 0; i < num_inputs; ++i) {
		auto name = session_.GetInputNameAllocated(i, alloc_);
		input_names_.emplace_back(name.get());
	}
	for (size_t i = 0; i < num_outputs; ++i) {
		auto name = session_.GetOutputNameAllocated(i, alloc_);
		output_names_.emplace_back(name.get());
	}

	if (num_inputs != 2) {
		throw std::runtime_error(
			"NetVLADDB: model should have exactly 2 inputs (image, descs).");
	}
	if (num_outputs != 3) {
		throw std::runtime_error(
			"NetVLADDB: model should have exactly 3 outputs (desc, values, idx).");
	}
}

NetVLADDB::~NetVLADDB() = default;

void NetVLADDB::query(const cv::Mat &image, std::vector<int> &best_idx,
		      std::vector<float> &best_values, bool add_to_history)
{
	best_idx.clear();
	best_values.clear();

	// Expect HWC CV_8UC3 224x224; pass-through (no color swap, no normalization)
	if (image.empty() || image.type() != CV_8UC3) {
		throw std::runtime_error(
			"NetVLADDB: expected CV_8UC3 image of size 224x224 (HWC).");
	}
	cv::Mat resized_image;
	if (image.rows != 224 || image.cols != 224) {
		cv::resize(image, resized_image, cv::Size(224, 224),
			   cv::INTER_LINEAR);
	} else {
		// Use original image if already 224x224
		resized_image = image;
	}
	cv::Mat hwc_u8 = image.isContinuous() ? image : image.clone();

	const int64_t H = 224, W = 224, C = 3;
	std::vector<int64_t> image_shape{ H, W, C }; // HWC

	// Descriptors tensor (fixed-size cyclic buffer), shape [history_size_,
	// NETVLAD_DIM]
	std::vector<int64_t> descs_shape{ static_cast<int64_t>(history_size_),
					  NETVLAD_DIM };

	// Create Ort::Value tensors (CPU)
	Ort::Value image_tensor = Ort::Value::CreateTensor<uint8_t>(
		mem_info_cpu_,
		const_cast<uint8_t *>(hwc_u8.ptr<uint8_t>(0)), // data
		static_cast<size_t>(
			hwc_u8.total() *
			hwc_u8.elemSize()), // bytes count used internally to deduce len
		image_shape.data(), image_shape.size());

	Ort::Value descs_tensor = Ort::Value::CreateTensor<float>(
		mem_info_cpu_, descs_.data(), descs_.size(), descs_shape.data(),
		descs_shape.size());

	// Prepare names (in discovered order)
	std::vector<const char *> input_name_ptrs;
	input_name_ptrs.reserve(input_names_.size());
	for (auto &s : input_names_)
		input_name_ptrs.push_back(s.c_str());

	std::vector<const char *> output_name_ptrs;
	output_name_ptrs.reserve(output_names_.size());
	for (auto &s : output_names_)
		output_name_ptrs.push_back(s.c_str());

	// Run inference
	auto outputs = session_.Run(
		Ort::RunOptions{ nullptr }, input_name_ptrs.data(),
		std::array<Ort::Value, 2>{ std::move(image_tensor),
					   std::move(descs_tensor) }
			.data(),
		2, output_name_ptrs.data(), output_name_ptrs.size());

	if (outputs.size() != 3) {
		throw std::runtime_error(
			"NetVLADDB: unexpected number of outputs.");
	}

	// Unpack outputs
	auto &desc_out_val = outputs[0];
	auto &values_val = outputs[1];
	auto &idx_val = outputs[2];

	// Top-k
	const float *values_ptr = values_val.GetTensorData<float>();
	const int64_t *idx_ptr = idx_val.GetTensorData<int64_t>();
	auto values_shape = values_val.GetTensorTypeAndShapeInfo().GetShape();
	if (values_shape.empty())
		throw std::runtime_error(
			"NetVLADDB: values output has empty shape.");
	int k = static_cast<int>(values_shape.back());

	best_values.assign(values_ptr, values_ptr + k);
	best_idx.resize(k);
	for (int i = 0; i < k; ++i)
		best_idx[i] = static_cast<int>(idx_ptr[i]);

	if (add_to_history) {
		// Insert descriptor into cyclic buffer (no normalization)
		const float *desc_ptr = desc_out_val.GetTensorData<float>();
		float *dst = descs_.data() +
			     static_cast<size_t>(write_pos_) * NETVLAD_DIM;
		std::memcpy(dst, desc_ptr, sizeof(float) * NETVLAD_DIM);

		// Advance ring pointer
		write_pos_ = (write_pos_ + 1) % history_size_;
	}
}
