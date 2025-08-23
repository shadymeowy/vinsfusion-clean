#pragma once

#include <onnxruntime_cxx_api.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

class Matcher {
 public:
  // Construct with fixed network input size (width, height) and a sanity cap on
  // keypoints.
  Matcher(const std::string &model_path, int width, int height, int n1, int n2);
  ~Matcher();

  // image1/image2: ORIGINAL images (any size, BGR or GRAY).
  // pts1/pts2: keypoints in ORIGINAL image coordinates (float); will be scaled
  // internally. Output: id1_to_2 with -1 for unmatched, length == pts1.size().
  void match(const cv::Mat &image1, const cv::Mat &image2,
             const std::vector<cv::Point2f> &pts1,
             const std::vector<cv::Point2f> &pts2, std::vector<int> &id1_to_2,
             float threshold);

 private:
  // Fixed network input geometry.
  int width_;
  int height_;
  int n1;
  int n2;

  // Per-call scales (set during preprocess).
  double scale1_ = 1.0;
  double scale2_ = 1.0;

  // ORT objects and helpers (CUDA EP integration like your sample).
  Ort::Env env_;
  Ort::SessionOptions so_;
  Ort::Session session_;
  Ort::MemoryInfo mem_info_cpu_;
  Ort::AllocatorWithDefaultOptions alloc_;

  // Discovered I/O names (order as returned by the model).
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;

  // ---- Helpers ----
  // Preprocess: convert to GRAY if needed, resize with scale=min(H/h, W/w),
  // and top-left paste into (height_, width_) uint8 buffer. Writes out_scale.
  cv::Mat preprocessImage(const cv::Mat &src, double &out_scale) const;

  // Tensors (both use the single, pre-created mem_info_cpu_).
  Ort::Value makeImageTensor(const cv::Mat &img) const;  // uint8 [H, W]
  Ort::Value makePtsTensor(
      const std::vector<int64_t> &flat_xy) const;    // int64 [N, 2]
  Ort::Value makeScalarTensor(float &scalar) const;  // double [1]

  // Convert original points -> scaled (network frame) int64 XY (numpy-like
  // rounding).
  static void toScaledInt64XY(const std::vector<cv::Point2f> &in, double scale,
                              std::vector<int64_t> &out_xy, int n);
};
