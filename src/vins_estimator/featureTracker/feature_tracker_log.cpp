#include <vins_estimator/featureTracker/feature_tracker_log.h>

#include <fstream>
#include <sstream>

namespace vins::estimator {

FeatureTrackerLogger::FeatureTrackerLogger(Parameters &params)
    : log_prefix_("log"), call_id_{0}, tracker_(params) {}

std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
FeatureTrackerLogger::trackImage(double _cur_time, const cv::Mat &_img,
                                 const cv::Mat &_img1) {
  FeatureTrackerMethodLog log;
  log.call_id = call_id_++;
  log.method_name = "trackImage";
  log.timestamp = _cur_time;

  log.image0_path = formatImagePath(log.call_id, 0);
  cv::imwrite(log.image0_path, _img);
  if (!_img1.empty()) {
    log.image1_path = formatImagePath(log.call_id, 1);
    cv::imwrite(log.image1_path, _img1);
  }

  auto result = tracker_.trackImage(_cur_time, _img, _img1);

  for (const auto &[id, vec] : result) {
    std::vector<std::pair<int, std::vector<double>>> out_vec;
    for (const auto &[frame_id, mat] : vec) {
      std::vector<double> v7(7);
      for (int i = 0; i < 7; ++i) v7[i] = mat(i);
      out_vec.emplace_back(frame_id, v7);
    }
    log.track_output[id] = out_vec;
  }

  cv::Mat track_img = tracker_.getTrackImage();
  log.track_image_path = formatTrackImagePath(log.call_id);
  cv::imwrite(log.track_image_path, track_img);

  saveLog(log);
  return result;
}

void FeatureTrackerLogger::readIntrinsicParameter(
    const std::vector<std::string> &calib_file) {
  FeatureTrackerMethodLog log;
  log.call_id = call_id_++;
  log.method_name = "readIntrinsicParameter";
  log.timestamp = 0;
  log.calib_files = calib_file;

  tracker_.readIntrinsicParameter(calib_file);
  saveLog(log);
}

void FeatureTrackerLogger::setPrediction(
    std::map<int, Eigen::Vector3d> &predictPts) {
  FeatureTrackerMethodLog log;
  log.call_id = call_id_++;
  log.method_name = "setPrediction";
  log.timestamp = 0;

  for (const auto &[id, pt] : predictPts) {
    log.predict_pts[id] = {pt(0), pt(1), pt(2)};
  }

  tracker_.setPrediction(predictPts);
  saveLog(log);
}

void FeatureTrackerLogger::removeOutliers(std::set<int> &removePtsIds) {
  FeatureTrackerMethodLog log;
  log.call_id = call_id_++;
  log.method_name = "removeOutliers";
  log.timestamp = 0;

  log.remove_outlier_ids.assign(removePtsIds.begin(), removePtsIds.end());

  tracker_.removeOutliers(removePtsIds);
  saveLog(log);
}

cv::Mat FeatureTrackerLogger::getTrackImage() {
  FeatureTrackerMethodLog log;
  log.call_id = call_id_++;
  log.method_name = "getTrackImage";
  log.timestamp = 0;

  cv::Mat img = tracker_.getTrackImage();

  log.track_image_path = formatTrackImagePath(log.call_id);
  cv::imwrite(log.track_image_path, img);

  saveLog(log);
  return img;
}

void FeatureTrackerLogger::saveLog(const FeatureTrackerMethodLog &log) {
  std::stringstream buffer;
  msgpack::pack(buffer, log);

  std::string file = formatLogPath(log.call_id);
  std::ofstream out(file, std::ios::binary);
  out << buffer.str();
}

std::string FeatureTrackerLogger::formatLogPath(int call_id) const {
  char buf[256];
  snprintf(buf, sizeof(buf), "%s_%05d.msgpack", log_prefix_.c_str(), call_id);
  return std::string(buf);
}

std::string FeatureTrackerLogger::formatImagePath(int call_id,
                                                  int img_idx) const {
  char buf[256];
  snprintf(buf, sizeof(buf), "%s_%05d_img%d.png", log_prefix_.c_str(), call_id,
           img_idx);
  return std::string(buf);
}

std::string FeatureTrackerLogger::formatTrackImagePath(int call_id) const {
  char buf[256];
  snprintf(buf, sizeof(buf), "%s_%05d_track.png", log_prefix_.c_str(), call_id);
  return std::string(buf);
}

}  // namespace vins::estimator
