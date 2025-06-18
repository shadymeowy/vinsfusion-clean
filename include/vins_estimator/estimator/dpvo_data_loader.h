#pragma once

#include <H5Cpp.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace vins::estimator {

struct DpvoData {
  Eigen::MatrixX2f target_distorted;   // (E, 2) full-res distorted pixels
  Eigen::MatrixX2f weight;             // (E, 2) per-dim confidence
  Eigen::MatrixX2f patch_centers;      // (n*M, 2) full-res distorted pixels
  Eigen::VectorXi ii, jj, kk;         // (E,) edge indices
  Eigen::VectorXi tstamps;            // (N,) DPVO window frame -> counter
  int n = 0;
  int M = 0;
  int RES = 0;
};

class DpvoDataLoader {
 public:
  DpvoDataLoader() = default;

  void init(const std::string& info_csv_path, const std::string& ba_args_dir) {
    ba_args_dir_ = ba_args_dir;
    loadInfoCSV(info_csv_path);  // optional, not needed for counter-based sync
    loadNonKeyframes(ba_args_dir + "/non_keyframes.csv");
    printf("[DpvoDataLoader] init: dir=%s, csv_entries=%d, non_keyframes=%d\n",
           ba_args_dir_.c_str(), (int)counter_to_ts_.size(),
           (int)non_keyframes_.size());
  }

  bool isInitialized() const { return !ba_args_dir_.empty(); }

  // Returns true if the given DPVO counter was removed (non-keyframe) by DPVO
  bool isNonKeyframe(int counter) const {
    return non_keyframes_.count(counter) > 0;
  }

  int findImageIndex(double vins_ts) const {
    int best_counter = -1;
    double best_diff = 1e18;
    for (const auto& [counter, ts] : counter_to_ts_) {
      double diff = std::abs(ts - vins_ts);
      if (diff < best_diff) {
        best_diff = diff;
        best_counter = counter;
      }
    }
    return best_counter;
  }

  double resolveTimestamp(int dpvo_frame, const DpvoData& data) const {
    if (dpvo_frame < 0 || dpvo_frame >= data.tstamps.size()) return -1.0;
    int counter = data.tstamps(dpvo_frame);
    auto it = counter_to_ts_.find(counter);
    if (it == counter_to_ts_.end()) return -1.0;
    return it->second;
  }

  bool loadH5(int image_index, DpvoData& data) const {
    std::string path = ba_args_dir_ + "/" + std::to_string(image_index) + ".h5";
    try {
      H5::H5File file(path, H5F_ACC_RDONLY);

      // Read scalar integers
      data.n = readScalarInt(file, "n");
      data.M = readScalarInt(file, "M");
      data.RES = readScalarInt(file, "RES");

      // Read target_distorted (E, 2)
      readMatrix2f(file, "target_distorted", data.target_distorted);

      // Read weight (1, E, 2) -> squeeze to (E, 2)
      readWeight(file, "weight", data.weight);

      // Read patch_centers_distorted (n*M, 2)
      readMatrix2f(file, "patch_centers_distorted", data.patch_centers);

      // Read edge indices
      readVectorInt(file, "ii", data.ii);
      readVectorInt(file, "jj", data.jj);
      readVectorInt(file, "kk", data.kk);

      // Read tstamps
      readTstamps(file, "tstamps", data.tstamps);

      return true;
    } catch (const H5::Exception& e) {
      std::cerr << "[DpvoDataLoader] Failed to load " << path << ": "
                << e.getDetailMsg() << std::endl;
      return false;
    }
  }

 private:
  void loadInfoCSV(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      std::cerr << "[DpvoDataLoader] Cannot open " << path << std::endl;
      return;
    }
    std::string line;
    std::getline(ifs, line);  // skip header
    while (std::getline(ifs, line)) {
      std::istringstream ss(line);
      std::string idx_str, ts_str;
      if (!std::getline(ss, idx_str, ',') || !std::getline(ss, ts_str, ','))
        continue;
      int idx = std::stoi(idx_str);
      double ts = std::stod(ts_str);
      counter_to_ts_[idx] = ts;
    }
  }

  static int readScalarInt(const H5::H5File& file, const std::string& name) {
    H5::DataSet ds = file.openDataSet(name);
    H5::DataSpace space = ds.getSpace();
    int rank = space.getSimpleExtentNdims();

    if (rank == 0) {
      // scalar dataset
      int val = 0;
      H5::DataType dtype = ds.getDataType();
      if (dtype.getClass() == H5T_FLOAT) {
        double dval;
        ds.read(&dval, H5::PredType::NATIVE_DOUBLE);
        val = static_cast<int>(dval);
      } else {
        ds.read(&val, H5::PredType::NATIVE_INT);
      }
      return val;
    }

    // 1-element array
    hsize_t dims[1];
    space.getSimpleExtentDims(dims);
    if (dims[0] == 1) {
      H5::DataType dtype = ds.getDataType();
      if (dtype.getClass() == H5T_FLOAT) {
        double dval;
        ds.read(&dval, H5::PredType::NATIVE_DOUBLE);
        return static_cast<int>(dval);
      }
      int val;
      ds.read(&val, H5::PredType::NATIVE_INT);
      return val;
    }
    return 0;
  }

  static void readMatrix2f(const H5::H5File& file, const std::string& name,
                           Eigen::MatrixX2f& mat) {
    H5::DataSet ds = file.openDataSet(name);
    H5::DataSpace space = ds.getSpace();
    hsize_t dims[2];
    space.getSimpleExtentDims(dims);
    int rows = static_cast<int>(dims[0]);
    mat.resize(rows, 2);
    // HDF5/numpy data is row-major: [x0, y0, x1, y1, ...]
    // Eigen MatrixX2f is column-major, so read into temp buffer first
    std::vector<float> buf(rows * 2);
    ds.read(buf.data(), H5::PredType::NATIVE_FLOAT);
    for (int i = 0; i < rows; i++) {
      mat(i, 0) = buf[i * 2];
      mat(i, 1) = buf[i * 2 + 1];
    }
  }

  static void readWeight(const H5::H5File& file, const std::string& name,
                         Eigen::MatrixX2f& mat) {
    H5::DataSet ds = file.openDataSet(name);
    H5::DataSpace space = ds.getSpace();
    int rank = space.getSimpleExtentNdims();
    hsize_t dims[3];
    space.getSimpleExtentDims(dims);

    if (rank == 3) {
      // shape (1, E, 2) -> read as (E, 2)
      int E = static_cast<int>(dims[1]);
      mat.resize(E, 2);
      std::vector<float> buf(dims[0] * dims[1] * dims[2]);
      ds.read(buf.data(), H5::PredType::NATIVE_FLOAT);
      for (int i = 0; i < E; i++) {
        mat(i, 0) = buf[i * 2];
        mat(i, 1) = buf[i * 2 + 1];
      }
    } else if (rank == 2) {
      int rows = static_cast<int>(dims[0]);
      mat.resize(rows, 2);
      std::vector<float> buf(rows * 2);
      ds.read(buf.data(), H5::PredType::NATIVE_FLOAT);
      for (int i = 0; i < rows; i++) {
        mat(i, 0) = buf[i * 2];
        mat(i, 1) = buf[i * 2 + 1];
      }
    }
  }

  static void readVectorInt(const H5::H5File& file, const std::string& name,
                            Eigen::VectorXi& vec) {
    H5::DataSet ds = file.openDataSet(name);
    H5::DataSpace space = ds.getSpace();
    hsize_t dims[1];
    space.getSimpleExtentDims(dims);
    int n = static_cast<int>(dims[0]);
    vec.resize(n);

    H5::DataType dtype = ds.getDataType();
    if (dtype.getSize() == 8) {
      // int64 -> read as long long then cast
      std::vector<long long> buf(n);
      ds.read(buf.data(), H5::PredType::NATIVE_LLONG);
      for (int i = 0; i < n; i++) vec(i) = static_cast<int>(buf[i]);
    } else {
      ds.read(vec.data(), H5::PredType::NATIVE_INT);
    }
  }

  static void readTstamps(const H5::H5File& file, const std::string& name,
                          Eigen::VectorXi& vec) {
    H5::DataSet ds = file.openDataSet(name);
    H5::DataSpace space = ds.getSpace();
    hsize_t dims[1];
    space.getSimpleExtentDims(dims);
    int n = static_cast<int>(dims[0]);
    vec.resize(n);

    H5::DataType dtype = ds.getDataType();
    if (dtype.getClass() == H5T_FLOAT) {
      std::vector<double> buf(n);
      ds.read(buf.data(), H5::PredType::NATIVE_DOUBLE);
      for (int i = 0; i < n; i++) vec(i) = static_cast<int>(buf[i]);
    } else if (dtype.getSize() == 8) {
      std::vector<long long> buf(n);
      ds.read(buf.data(), H5::PredType::NATIVE_LLONG);
      for (int i = 0; i < n; i++) vec(i) = static_cast<int>(buf[i]);
    } else {
      ds.read(vec.data(), H5::PredType::NATIVE_INT);
    }
  }

  void loadNonKeyframes(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      printf("[DpvoDataLoader] no non_keyframes file: %s\n", path.c_str());
      return;
    }
    std::string line;
    std::getline(ifs, line);  // skip header
    while (std::getline(ifs, line)) {
      if (line.empty()) continue;
      int counter = std::stoi(line);
      non_keyframes_.insert(counter);
    }
  }

  std::string ba_args_dir_;
  std::map<int, double> counter_to_ts_;
  std::set<int> non_keyframes_;
};

}  // namespace vins::estimator
