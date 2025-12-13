#include <vins_estimator/featureTracker/tapnext_trt.h>

#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace trt = nvinfer1;

inline static void checkCuda(cudaError_t e, const char *file, int line) {
  if (e != cudaSuccess) {
    std::cerr << "CUDA error " << cudaGetErrorString(e) << " at " << file << ":"
              << line << std::endl;
    std::exit(1);
  }
}
#define CHECK_CUDA(x) checkCuda((x), __FILE__, __LINE__)

inline static size_t volume(const trt::Dims &d) {
  size_t v = 1;
  for (int i = 0; i < d.nbDims; ++i)
    v *= static_cast<size_t>(d.d[i]);
  return v;
}

static inline size_t dtypeSize(trt::DataType t) {
  switch (t) {
  case trt::DataType::kFLOAT:
    return 4;
  case trt::DataType::kHALF:
    return 2;
  case trt::DataType::kINT8:
    return 1;
  case trt::DataType::kINT32:
    return 4;
  case trt::DataType::kINT64:
    return 8;
  case trt::DataType::kUINT8:
    return 1;
  case trt::DataType::kBOOL:
    return 1;
  case trt::DataType::kFP8:
    return 1;
  case trt::DataType::kBF16:
    return 2;
  default:
    return 0;
  }
}

inline static bool containsCI(std::string s, std::string n) {
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  std::transform(n.begin(), n.end(), n.begin(), ::tolower);
  return s.find(n) != std::string::npos;
}

struct TAPNextTRT::Logger : public trt::ILogger {
  trt::ILogger::Severity reportableSeverity = trt::ILogger::Severity::kINFO;

  void log(trt::ILogger::Severity severity,
           trt::AsciiChar const *msg) noexcept override {
    if (severity <= reportableSeverity)
      std::cerr << "[TRT] " << msg << std::endl;
  }
};

struct TAPNextTRT::TensorBuf {
  void *dptr = nullptr;
  void *hptr = nullptr;
  size_t nbytes = 0;
  trt::Dims shape{};
  trt::DataType dtype{};
};

struct TAPNextTRT::Buffers {
  std::unordered_map<std::string, TensorBuf> tensor;
  cudaStream_t stream = nullptr;
  ~Buffers() {
    for (auto &kv : tensor) {
      if (kv.second.dptr)
        cudaFree(kv.second.dptr);
      if (kv.second.hptr)
        cudaFreeHost(kv.second.hptr);
    }
    if (stream)
      cudaStreamDestroy(stream);
  }
};

TAPNextTRT::~TAPNextTRT() { destroyCudaGraph(); }

// ---------- class impl ----------
TAPNextTRT::TAPNextTRT(const std::string &onnxPath,
                       const std::string &enginePath, int nTracks, int imgW,
                       int imgH, int modelW, int modelH)
    : logger_(new Logger()), nTracks_(nTracks), imageW_(imgW), imageH_(imgH),
      modelW_(modelW), modelH_(modelH),
      sx_(static_cast<float>(imgW) / static_cast<float>(modelW)),
      sy_(static_cast<float>(imgH) / static_cast<float>(modelH)), step_(0) {

  // Build if engine missing
  runtime_ = std::unique_ptr<trt::IRuntime>(trt::createInferRuntime(*logger_));
  if (!std::ifstream(enginePath).good()) {
    std::cout << "Building engine from ONNX: " << onnxPath << std::endl;
    buildEngineFromOnnx(onnxPath, enginePath);
  }
  loadEngine(enginePath);
  if (!engine_)
    throw std::runtime_error("Failed to load engine.");

  context_.reset(engine_->createExecutionContext());
  if (!context_)
    throw std::runtime_error("Failed to create execution context.");

  allocateBuffers();
  bindAll();
  zeroStates();
  buildCudaGraph();
}

void TAPNextTRT::reset(const std::vector<float> &queryPointsB1xNx3) {
  auto it = bufs_->tensor.find("query_points_in");
  if (it == bufs_->tensor.end())
    throw std::runtime_error("Missing 'query_points_in' tensor.");
  const auto &qpb = it->second;

  if (queryPointsB1xNx3.size() * sizeof(float) != qpb.nbytes)
    throw std::runtime_error("query_points size mismatch.");

  if (qpb.hptr) {
    std::memcpy(qpb.hptr, queryPointsB1xNx3.data(), qpb.nbytes);
    CHECK_CUDA(cudaMemcpyAsync(qpb.dptr, qpb.hptr, qpb.nbytes,
                               cudaMemcpyHostToDevice, bufs_->stream));
  } else {
    CHECK_CUDA(cudaMemcpyAsync(qpb.dptr, queryPointsB1xNx3.data(), qpb.nbytes,
                               cudaMemcpyHostToDevice, bufs_->stream));
  }

  zeroStates();
  step_ = 0;
  CHECK_CUDA(cudaStreamSynchronize(bufs_->stream));
}

void TAPNextTRT::reset(const std::vector<float> &xs,
                       const std::vector<float> &ys) {
  if (xs.size() != ys.size())
    throw std::runtime_error("reset(x,y): size mismatch");

  const size_t N = xs.size();
  std::vector<float> qp(N * 3);
  size_t idx = 0;
  for (size_t i = 0; i < N; ++i) {
    // Model expects [0.0, y, x] in model coordinates
    qp[idx + 0] = 0.0F;
    qp[idx + 1] = ys[i] / sy_;
    qp[idx + 2] = xs[i] / sx_;
    idx += 3;
  }
  reset(qp);
}

std::pair<std::vector<cv::Point2f>, std::vector<uint8_t>>
TAPNextTRT::run(const cv::Mat &bgrFrame) {
  if (bgrFrame.empty())
    throw std::runtime_error("Empty frame");

  int inW = bgrFrame.cols;
  int inH = bgrFrame.rows;
  assert(inW == imageW_ && inH == imageH_);

  // preprocess
  cv::Mat rsz;
  cv::Mat rgb;
  cv::Mat f32;
  cv::resize(bgrFrame, rsz, cv::Size(modelW_, modelH_));
  cv::cvtColor(rsz, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(f32, CV_32F, 1.0);
  f32 = (f32 / 255.0F) * 2.0F - 1.0F;

  std::vector<float> videoFlat(static_cast<size_t>(modelW_ * modelH_ * 3));
  std::memcpy(videoFlat.data(), f32.data, videoFlat.size() * sizeof(float));

  auto &vid = bufs_->tensor.at("video");
  auto &stepIn = bufs_->tensor.at("step_in");
  auto &tracks = bufs_->tensor.at("tracks");
  auto &vislog = bufs_->tensor.at("visible_logits");

  if (!vid.hptr || vid.nbytes != videoFlat.size() * sizeof(float))
    throw std::runtime_error("'video' tensor mismatch or not pinned");

  std::memcpy(vid.hptr, videoFlat.data(), vid.nbytes);

  if (dtypeSize(stepIn.dtype) != sizeof(int64_t))
    throw std::runtime_error("'step_in' expected int32");
  if (!stepIn.hptr)
    throw std::runtime_error("'step_in' not pinned");
  std::memcpy(stepIn.hptr, &step_, sizeof(int64_t));

  // Launch captured sequence (H2D -> enqueue -> D2H -> rotate states)
  CHECK_CUDA(cudaGraphLaunch(graphExec_, bufs_->stream));
  CHECK_CUDA(
      cudaStreamSynchronize(bufs_->stream)); // preserve original sync semantics

  auto N = static_cast<size_t>(nTracks_);
  const auto *trk = reinterpret_cast<const float *>(tracks.hptr);
  const auto *vlg = reinterpret_cast<const float *>(vislog.hptr);

  std::vector<cv::Point2f> pts;
  pts.reserve(N);
  std::vector<uint8_t> visible(N, 0);

  for (size_t i = 0; i < N; ++i) {
    float y = trk[i * 2 + 0] * sy_;
    float x = trk[i * 2 + 1] * sx_;
    pts.emplace_back(x, y);
    visible[i] = vlg[i] > 0.0F ? 1 : 0;
  }

  ++step_;
  return {pts, visible};
}

void TAPNextTRT::bindAll() {
  for (auto &kv : bufs_->tensor) {
    context_->setTensorAddress(kv.first.c_str(), kv.second.dptr);
  }
}

void TAPNextTRT::zeroStates() {
  auto it1 = bufs_->tensor.find("conv1d_states_in");
  auto it2 = bufs_->tensor.find("rg_lru_states_in");
  if (it1 != bufs_->tensor.end())
    CHECK_CUDA(cudaMemsetAsync(it1->second.dptr, 0, it1->second.nbytes,
                               bufs_->stream));
  if (it2 != bufs_->tensor.end())
    CHECK_CUDA(cudaMemsetAsync(it2->second.dptr, 0, it2->second.nbytes,
                               bufs_->stream));
  CHECK_CUDA(cudaStreamSynchronize(bufs_->stream));
}

void TAPNextTRT::rotateStates() {
  auto in1 = bufs_->tensor.find("conv1d_states_in");
  auto out1 = bufs_->tensor.find("conv1d_states_out");
  auto in2 = bufs_->tensor.find("rg_lru_states_in");
  auto out2 = bufs_->tensor.find("rg_lru_states_out");
  if (in1 != bufs_->tensor.end() && out1 != bufs_->tensor.end())
    CHECK_CUDA(cudaMemcpyAsync(in1->second.dptr, out1->second.dptr,
                               in1->second.nbytes, cudaMemcpyDeviceToDevice,
                               bufs_->stream));
  if (in2 != bufs_->tensor.end() && out2 != bufs_->tensor.end())
    CHECK_CUDA(cudaMemcpyAsync(in2->second.dptr, out2->second.dptr,
                               in2->second.nbytes, cudaMemcpyDeviceToDevice,
                               bufs_->stream));
  CHECK_CUDA(cudaStreamSynchronize(bufs_->stream));
}

void TAPNextTRT::destroyCudaGraph() {
  if (graphExec_) {
    cudaGraphExecDestroy(graphExec_);
    graphExec_ = nullptr;
  }
  if (graph_) {
    cudaGraphDestroy(graph_);
    graph_ = nullptr;
  }
}

void TAPNextTRT::buildCudaGraph() {
  destroyCudaGraph();

  // Required tensors
  auto &vid = bufs_->tensor.at("video");
  auto &stepIn = bufs_->tensor.at("step_in");
  auto &tracks = bufs_->tensor.at("tracks");
  auto &vislog = bufs_->tensor.at("visible_logits");

  // Optional state tensors
  auto in1 = bufs_->tensor.find("conv1d_states_in");
  auto out1 = bufs_->tensor.find("conv1d_states_out");
  auto in2 = bufs_->tensor.find("rg_lru_states_in");
  auto out2 = bufs_->tensor.find("rg_lru_states_out");

  // Capture exactly your steady-state sequence on the same stream
  CHECK_CUDA(
      cudaStreamBeginCapture(bufs_->stream, cudaStreamCaptureModeGlobal));

  // H2D fixed pointers (you will update contents before each launch)
  CHECK_CUDA(cudaMemcpyAsync(vid.dptr, vid.hptr, vid.nbytes,
                             cudaMemcpyHostToDevice, bufs_->stream));
  CHECK_CUDA(cudaMemcpyAsync(stepIn.dptr, stepIn.hptr, sizeof(int64_t),
                             cudaMemcpyHostToDevice, bufs_->stream));

  // Inference
  if (!context_->enqueueV3(bufs_->stream))
    throw std::runtime_error("enqueueV3 failed during graph capture");

  // D2H outputs
  CHECK_CUDA(cudaMemcpyAsync(tracks.hptr, tracks.dptr, tracks.nbytes,
                             cudaMemcpyDeviceToHost, bufs_->stream));
  CHECK_CUDA(cudaMemcpyAsync(vislog.hptr, vislog.dptr, vislog.nbytes,
                             cudaMemcpyDeviceToHost, bufs_->stream));

  // Rotate recurrent states (if present)
  if (in1 != bufs_->tensor.end() && out1 != bufs_->tensor.end())
    CHECK_CUDA(cudaMemcpyAsync(in1->second.dptr, out1->second.dptr,
                               in1->second.nbytes, cudaMemcpyDeviceToDevice,
                               bufs_->stream));
  if (in2 != bufs_->tensor.end() && out2 != bufs_->tensor.end())
    CHECK_CUDA(cudaMemcpyAsync(in2->second.dptr, out2->second.dptr,
                               in2->second.nbytes, cudaMemcpyDeviceToDevice,
                               bufs_->stream));

  CHECK_CUDA(cudaStreamEndCapture(bufs_->stream, &graph_));
  CHECK_CUDA(cudaGraphInstantiate(&graphExec_, graph_, 0ULL));
}

void TAPNextTRT::loadEngine(const std::string &enginePath) {
  std::ifstream ifs(enginePath, std::ios::binary);
  if (!ifs)
    throw std::runtime_error("Could not open engine: " + enginePath);
  std::vector<char> blob((std::istreambuf_iterator<char>(ifs)),
                         std::istreambuf_iterator<char>());
  engine_ = std::unique_ptr<trt::ICudaEngine>(
      runtime_->deserializeCudaEngine(blob.data(), blob.size()));
}

void TAPNextTRT::allocateBuffers() {
  bufs_ = std::make_unique<TAPNextTRT::Buffers>();
  CHECK_CUDA(cudaStreamCreate(&bufs_->stream));

  int nIO = engine_->getNbIOTensors();
  for (int i = 0; i < nIO; ++i) {
    const char *nm = engine_->getIOTensorName(i);
    std::string name = nm ? nm : std::string("tensor_") + std::to_string(i);

    trt::Dims shape = engine_->getTensorShape(name.c_str());
    trt::DataType dtype = engine_->getTensorDataType(name.c_str());
    size_t nbytes = volume(shape) * dtypeSize(dtype);

    TAPNextTRT::TensorBuf tb;
    tb.shape = shape;
    tb.dtype = dtype;
    tb.nbytes = nbytes;

    CHECK_CUDA(cudaMalloc(&tb.dptr, nbytes));

    // Host-pinned for common I/O used from CPU
    if (name == "video" || name == "step_in" || name == "tracks" ||
        name == "visible_logits" || name == "query_points_in") {
      CHECK_CUDA(cudaMallocHost(&tb.hptr, nbytes));
    }

    bufs_->tensor.emplace(name, tb);
  }
}

void TAPNextTRT::buildEngineFromOnnx(const std::string &onnxPath,
                                     const std::string &enginePath) {
  auto builder =
      std::unique_ptr<trt::IBuilder>(trt::createInferBuilder(*logger_));
  if (!builder)
    throw std::runtime_error("Failed to create builder");

  auto network =
      std::unique_ptr<trt::INetworkDefinition>(builder->createNetworkV2(0));
  auto config =
      std::unique_ptr<trt::IBuilderConfig>(builder->createBuilderConfig());
  if (!network || !config)
    throw std::runtime_error("Failed to create network/config");
  config->setMemoryPoolLimit(trt::MemoryPoolType::kWORKSPACE, 1ULL << 30);
  config->setFlag(trt::BuilderFlag::kFP16);

  auto parser = std::unique_ptr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, *logger_));
  std::ifstream ifs(onnxPath, std::ios::binary);
  if (!ifs)
    throw std::runtime_error("ONNX not found: " + onnxPath);
  std::string onnxData((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

  std::cout << "Parsing ONNX: " << onnxPath << "..." << std::endl;
  if (!parser->parse(onnxData.data(), onnxData.size())) {
    std::cerr << "ERROR: parse failed.\n";
    for (int i = 0; i < parser->getNbErrors(); ++i)
      std::cerr << parser->getError(i)->desc() << std::endl;
    return;
  }
  std::cout << "ONNX parsing complete.\n";

  // Per-layer precision
  std::vector<std::string> fp16Names = {"vit_block", "ssm_block"};
  std::vector<std::string> fp32Names = {"rg_lru", "reduce", "temporal_pre_norm",
                                        "channel_pre_norm"};
  std::vector<std::string> ignoreNames = {"Gather", "Cast"};

  std::cout << "Applying mixed precision settings..." << std::endl;
  for (int i = 0; i < network->getNbLayers(); ++i) {
    auto *layer = network->getLayer(i);
    std::string lname =
        layer->getName() ? layer->getName() : ("layer_" + std::to_string(i));

    auto unsupported = [](trt::DataType t) {
      return t != trt::DataType::kFLOAT && t != trt::DataType::kHALF;
    };

    bool ignore = false;
    for (const auto &s : ignoreNames)
      if (containsCI(lname, s)) {
        ignore = true;
        break;
      }
    if (ignore) {
      std::cout << "    -> '" << lname << "' ignored.\n";
      continue;
    }

    if (unsupported(layer->getPrecision())) {
      std::cout << "    -> '" << lname << "' unsupported precision. Skip.\n";
      continue;
    }
    bool outBad = false;
    for (int j = 0; j < layer->getNbOutputs(); ++j) {
      auto t = layer->getOutputType(j);
      if (t != trt::DataType::kFLOAT && t != trt::DataType::kHALF) {
        outBad = true;
        break;
      }
    }
    if (outBad) {
      std::cout << "    -> '" << lname << "' bad output types. Skip.\n";
      continue;
    }

    bool wantsFP16 = false;
    bool forcedFP32 = false;
    for (const auto &s : fp16Names)
      if (containsCI(lname, s)) {
        wantsFP16 = true;
        break;
      }
    for (const auto &s : fp32Names)
      if (containsCI(lname, s)) {
        forcedFP32 = true;
        break;
      }

    if (wantsFP16 && !forcedFP32) {
      layer->setPrecision(trt::DataType::kHALF);
      for (int j = 0; j < layer->getNbOutputs(); ++j)
        layer->setOutputType(j, trt::DataType::kHALF);
      std::cout << "    -> '" << lname << "' -> FP16\n";
    } else {
      layer->setPrecision(trt::DataType::kFLOAT);
      for (int j = 0; j < layer->getNbOutputs(); ++j)
        layer->setOutputType(j, trt::DataType::kFLOAT);
      std::cout << "    -> '" << lname << "' -> FP32\n";
    }
  }

  std::cout << "Building TensorRT engine..." << std::endl;
  auto plan = std::unique_ptr<trt::IHostMemory>(
      builder->buildSerializedNetwork(*network, *config));
  if (!plan) {
    std::cerr << "ERROR: engine build failed.\n";
    return;
  }

  // Save engine
  {
    std::ofstream ofs(enginePath, std::ios::binary);
    ofs.write(reinterpret_cast<const char *>(plan->data()), plan->size());
  }
  std::cout << "Engine saved to " << enginePath << ".\n";

  engine_ = std::unique_ptr<trt::ICudaEngine>(
      runtime_->deserializeCudaEngine(plan->data(), plan->size()));
}