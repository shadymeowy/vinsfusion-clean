#include <loop_fusion/matcher.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

Matcher::Matcher(const std::string &model_path, int width, int height, int n1,
                 int n2)
    : width_(width),
      height_(height),
      n1(n1),
      n2(n2),
      env_(ORT_LOGGING_LEVEL_WARNING, "Matcher"),
      so_(),
      session_(nullptr),
      mem_info_cpu_(
          Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault)) {
  if (width_ <= 0 || height_ <= 0) {
    throw std::runtime_error("Matcher: width and height must be > 0");
  }
  if (n1 <= 0) {
    throw std::runtime_error("Matcher: n1 must be > 0");
  }
  if (n2 <= 0) {
    throw std::runtime_error("Matcher: n2 must be > 0");
  }

  // Session options (match your style).
  so_.SetIntraOpNumThreads(1);
  so_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  // CUDA EP (as in your sample; optional if using a GPU build of ORT).
  {
    OrtCUDAProviderOptions cuda_options{};
    cuda_options.device_id = 0;
    cuda_options.arena_extend_strategy = 0;
    cuda_options.gpu_mem_limit = std::numeric_limits<size_t>::max();
    cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchDefault;
    cuda_options.do_copy_in_default_stream = 1;
    so_.AppendExecutionProvider_CUDA(cuda_options);
  }

  // Create session
  session_ = Ort::Session(env_, model_path.c_str(), so_);

  // Discover input/output names (follow your pattern)
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

  if (num_inputs != 5) {
    throw std::runtime_error(
        "Matcher: model should have exactly 5 inputs (image1, image2, pts1, "
        "pts2, threshold).");
  }
  if (num_outputs != 1) {
    throw std::runtime_error(
        "Matcher: model should have exactly 1 output (matches).");
  }
}

Matcher::~Matcher() = default;

cv::Mat Matcher::preprocessImage(const cv::Mat &src, double &out_scale) const {
  if (src.empty()) {
    throw std::runtime_error("Matcher: input image is empty.");
  }

  // Convert to grayscale if needed
  cv::Mat gray;
  if (src.channels() == 1) {
    gray = src;
  } else {
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  }

  // Output buffer (H, W) with zeros
  cv::Mat buffer(cv::Size(width_, height_), CV_8UC1, cv::Scalar(0));

  // Compute scale = min(H/h, W/w) (Python parity)
  const int h = gray.rows;
  const int w = gray.cols;
  out_scale = std::min(static_cast<double>(height_) / static_cast<double>(h),
                       static_cast<double>(width_) / static_cast<double>(w));

  const int new_w = static_cast<int>(std::round(w * out_scale));
  const int new_h = static_cast<int>(std::round(h * out_scale));

  cv::Mat resized;
  cv::resize(gray, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  // Top-left paste
  assert(new_w <= width_ && new_h <= height_);
  resized.copyTo(buffer(cv::Rect(0, 0, resized.cols, resized.rows)));

  return buffer;
}

Ort::Value Matcher::makeImageTensor(const cv::Mat &img) const {
  assert(img.type() == CV_8UC1);
  assert(img.isContinuous());
  // Shape [H, W]
  std::vector<int64_t> shape = {img.rows, img.cols};
  // Use the single pre-created mem_info_cpu_ (NetVLADDB style).
  return Ort::Value::CreateTensor<uint8_t>(
      mem_info_cpu_, const_cast<uint8_t *>(img.ptr<uint8_t>(0)),
      static_cast<size_t>(img.total() * img.elemSize()), shape.data(),
      shape.size());
}

Ort::Value Matcher::makePtsTensor(const std::vector<int64_t> &flat_xy) const {
  const int64_t N = static_cast<int64_t>(flat_xy.size() / 2);
  std::vector<int64_t> shape = {N, 2};
  // Again reuse mem_info_cpu_ instead of creating a new MemoryInfo.
  return Ort::Value::CreateTensor<int64_t>(
      mem_info_cpu_, const_cast<int64_t *>(flat_xy.data()), flat_xy.size(),
      shape.data(), shape.size());
}

Ort::Value Matcher::makeScalarTensor(float &scalar) const {
  std::vector<int64_t> shape = {1};
  return Ort::Value::CreateTensor<float>(mem_info_cpu_, &scalar, 1,
                                         shape.data(), shape.size());
}

void Matcher::toScaledInt64XY(const std::vector<cv::Point2f> &in, double scale,
                              std::vector<int64_t> &out_xy, int n) {
  out_xy.clear();
  out_xy.reserve(n * 2);
  for (int i = 0; i < std::min(n, static_cast<int>(in.size())); ++i) {
    const double xs = in[i].x * scale;
    const double ys = in[i].y * scale;
    out_xy.push_back(static_cast<int64_t>(std::llround(xs)));
    out_xy.push_back(static_cast<int64_t>(std::llround(ys)));
  }
}

void Matcher::match(const cv::Mat &image1, const cv::Mat &image2,
                    const std::vector<cv::Point2f> &pts1,
                    const std::vector<cv::Point2f> &pts2,
                    std::vector<int> &id1_to_2, float threshold) {
  id1_to_2.clear();
  if (static_cast<int>(pts1.size()) > n1) {
    std::cout << "Matcher: number of keypoints exceeds n1" << pts1.size()
              << " > " << n1 << std::endl;
  }
  if (static_cast<int>(pts2.size()) > n2) {
    std::cout << "Matcher: number of keypoints exceeds n2" << pts2.size()
              << " > " << n2 << std::endl;
  }

  // 1) Preprocess both images and record scales
  cv::Mat pre1 = preprocessImage(image1, scale1_);
  cv::Mat pre2 = preprocessImage(image2, scale2_);

  // 2) Scale & round keypoints into network frame
  std::vector<int64_t> pts1_xy, pts2_xy;
  toScaledInt64XY(pts1, scale1_, pts1_xy, n1);
  toScaledInt64XY(pts2, scale2_, pts2_xy, n2);

  // if pts1_xy size is less than n_keypoint_max_, pad with zeros
  if (pts1_xy.size() < 2 * n1) {
    std::cout << "Matcher: padding pts1_xy from " << pts1_xy.size() << " to "
              << 2 * n1 << std::endl;
    pts1_xy.resize(2 * n1, 0);
  }
  if (pts2_xy.size() < 2 * n2) {
    std::cout << "Matcher: padding pts2_xy from " << pts2_xy.size() << " to "
              << 2 * n2 << std::endl;
    pts2_xy.resize(2 * n2, 0);
  }

  // 3) Create tensors (all using the same mem_info_cpu_)
  static const std::array<const char *, 5> kInputNames = {
      "image1", "image2", "pts1", "pts2", "threshold"};
  static const std::array<const char *, 1> kOutputNames = {"matches"};

  // Build tensors (types/shapes must match the model)
  Ort::Value image1_tensor = makeImageTensor(pre1);  // CV_8UC1 -> uint8 [H1,W1]
  Ort::Value image2_tensor = makeImageTensor(pre2);  // CV_8UC1 -> uint8 [H2,W2]
  Ort::Value pts1_tensor = makePtsTensor(pts1_xy);   // int64  -> [256,2]
  Ort::Value pts2_tensor = makePtsTensor(pts2_xy);   // int64  -> [2048,2]
  Ort::Value threshold_tensor = makeScalarTensor(threshold);  // float32-> [1]

  std::array<Ort::Value, 5> input_vals = {
      std::move(image1_tensor), std::move(image2_tensor),
      std::move(pts1_tensor), std::move(pts2_tensor),
      std::move(threshold_tensor)};

  auto outputs = session_.Run(Ort::RunOptions{nullptr}, kInputNames.data(),
                              input_vals.data(), input_vals.size(),
                              kOutputNames.data(), kOutputNames.size());

  if (outputs.size() != 1) {
    throw std::runtime_error("Matcher: unexpected number of outputs.");
  }

  // We only need the matches (idx mapping). By convention, output[0] is
  // "matches".
  auto &matches_val = outputs[0];

  const int64_t *mptr = matches_val.GetTensorData<int64_t>();
  auto mshape = matches_val.GetTensorTypeAndShapeInfo().GetShape();
  if (mshape.empty()) {
    throw std::runtime_error("Matcher: matches output has empty shape.");
  }

  size_t mcount = 1;
  for (auto d : mshape) mcount *= static_cast<size_t>(d < 0 ? 0 : d);

  id1_to_2.resize(mcount);
  for (size_t i = 0; i < mcount; ++i) {
    id1_to_2[i] = static_cast<int>(mptr[i]);
  }
  // trim to original pts1 size
  id1_to_2.resize(pts1.size());

  // replace out of bounds pts2 with -1
  for (size_t i = 0; i < id1_to_2.size(); ++i) {
    if (id1_to_2[i] < 0 || id1_to_2[i] >= static_cast<int>(pts2.size())) {
      id1_to_2[i] = -1;
    }
  }
}
