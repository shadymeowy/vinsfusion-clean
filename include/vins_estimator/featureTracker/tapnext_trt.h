#pragma once
#include <NvInfer.h>
#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

class TAPNextTRT {
public:
  TAPNextTRT(const std::string &onnxPath, const std::string &enginePath,
             int nTracks = 256, int imgW = 256, int imgH = 256,
             int modelW = 256, int modelH = 256);

  ~TAPNextTRT();

  // reset ALWAYS expects query points as a flat float32 buffer of shape [1, N,
  // 3]
  void reset(const std::vector<float> &query);
  void reset(const std::vector<float> &xs, const std::vector<float> &ys);

  // Run one frame (cv::Mat BGR8) -> (N points, N visibility flags)
  std::pair<std::vector<cv::Point2f>, std::vector<uint8_t>>
  run(const cv::Mat &bgrFrame);

  struct Logger;
  struct TensorBuf;
  struct Buffers;

private:
  void bindAll();
  void zeroStates();
  void rotateStates();
  void buildCudaGraph();
  void destroyCudaGraph();

  void buildEngineFromOnnx(const std::string &onnxPath,
                           const std::string &enginePath);
  void allocateBuffers();
  void loadEngine(const std::string &enginePath);

private:
  Logger *logger_{};
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::unique_ptr<Buffers> bufs_;

  int nTracks_ = 256;
  int modelW_ = 256, modelH_ = 256;
  int imageW_ = 256, imageH_ = 256;
  float sx_ = 1.0f, sy_ = 1.0f;
  int64_t step_ = 0;

  cudaGraph_t graph_ = nullptr;
  cudaGraphExec_t graphExec_ = nullptr;
};
