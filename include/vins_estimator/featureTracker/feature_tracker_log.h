#pragma once

#include <vins_estimator/estimator/parameters.h>
#include <vins_estimator/featureTracker/feature_tracker.h>

#include <Eigen/Dense>
#include <msgpack/adaptor/define_decl.hpp>
#include <opencv2/opencv.hpp>
#include <msgpack.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace vins::estimator {

struct FeatureTrackerMethodLog {
  int call_id;
  std::string method_name;
  double timestamp;

  std::string image0_path;
  std::string image1_path;
  std::vector<std::string> calib_files;
  std::map<int, std::vector<double>> predict_pts;
  std::vector<int> remove_outlier_ids;

  std::map<int, std::vector<std::pair<int, std::vector<double>>>> track_output;
  std::string track_image_path;

  MSGPACK_DEFINE_MAP(call_id, method_name, timestamp, image0_path, image1_path,
                 calib_files, predict_pts, remove_outlier_ids, track_output,
                 track_image_path);
};

class FeatureTrackerLogger {
 public:
  explicit FeatureTrackerLogger(Parameters& params);

  std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
  trackImage(double _cur_time, const cv::Mat& _img,
             const cv::Mat& _img1 = cv::Mat());

  void readIntrinsicParameter(const std::vector<std::string>& calib_file);
  void setPrediction(std::map<int, Eigen::Vector3d>& predictPts);
  void removeOutliers(std::set<int>& removePtsIds);
  cv::Mat getTrackImage();

 private:
  FeatureTracker tracker_;
  int call_id_;
  std::string log_prefix_;

  void saveLog(const FeatureTrackerMethodLog& log);
  std::string formatLogPath(int call_id) const;
  std::string formatImagePath(int call_id, int img_idx) const;
  std::string formatTrackImagePath(int call_id) const;
};

}  // namespace vins::estimator
