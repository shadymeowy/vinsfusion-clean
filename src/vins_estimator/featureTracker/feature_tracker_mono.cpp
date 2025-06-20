/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <vins_estimator/featureTracker/feature_tracker_mono.h>
#include <vins_estimator/featureTracker/klt_tracker.h>

extern "C" int track_klt_cy(const unsigned char *img, int width, int height, float *x_out, float *y_out,
               int *ids_out, int *cnt_out,
               int min_dist, int max_cnt, char flow_back);
extern "C" void mymodule_init();

namespace vins::estimator {

FeatureTrackerMono::FeatureTrackerMono(Parameters &params) : params(params) {}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
FeatureTrackerMono::trackImage(double cur_time, const cv::Mat &cur_img,
                               const cv::Mat &) {

  TicToc t_r;

  vector<float> cur_x(params.max_cnt, 0);
  vector<float> cur_y(params.max_cnt, 0);
  vector<int> ids(params.max_cnt, 0);
  vector<int> track_cnt(params.max_cnt, 0);

  vector<cv::Point2f> cur_pts;

  int width = cur_img.cols;
  int height = cur_img.rows;

  static bool init = false;
  if (!init) {
    mymodule_init();
    init = true;
  }
  int len = track_klt_cy(
      cur_img.data, width, height, cur_x.data(), cur_y.data(), ids.data(),
      track_cnt.data(), params.min_dist, params.max_cnt, params.flow_back);

  printf("track_klt_cy len: %d, cur_time: %f\n", len, cur_time);
  // resize vectors cur_x, cur_y, ids, track_cnt
  cur_x.resize(len);
  cur_y.resize(len);
  ids.resize(len);
  track_cnt.resize(len);

  cur_pts.reserve(len);
  for (int i = 0; i < len; i++) {
    cur_pts.emplace_back(cur_x[i], cur_y[i]);
  }

  vector<cv::Point2f> cur_un_pts;
  undistortedPts(cur_un_pts, cur_pts, m_camera_[0]);
  map<int, cv::Point2f> cur_un_pts_map;
  vector<cv::Point2f> pts_velocity;
  ptsVelocity(pts_velocity, cur_time - prev_time_, ids, cur_un_pts,
              cur_un_pts_map, prev_un_pts_map_);

  if (params.show_track) {
    im_track_ = cur_img.clone();
    drawTrack(im_track_, cur_img, ids, cur_pts, track_cnt, prev_pts_map_);
  }

  prev_un_pts_ = cur_un_pts;
  prev_un_pts_map_ = cur_un_pts_map;
  prev_time_ = cur_time;

  prev_pts_map_.clear();
  for (size_t i = 0; i < cur_pts.size(); i++)
    prev_pts_map_[ids[i]] = cur_pts[i];

  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
  for (size_t i = 0; i < ids.size(); i++) {
    int feature_id = ids[i];
    double x = cur_un_pts[i].x;
    double y = cur_un_pts[i].y;
    double z = 1;
    double p_u = cur_pts[i].x;
    double p_v = cur_pts[i].y;
    int camera_id = 0;
    double velocity_x = pts_velocity[i].x;
    double velocity_y = pts_velocity[i].y;

    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
  }
  // printf("feature track whole time %f\n", t_r.toc());
  return featureFrame;
}

void FeatureTrackerMono::readIntrinsicParameter(
    const vector<string> &calib_file) {
  for (size_t i = 0; i < calib_file.size(); i++) {
    ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
    camodocal::CameraPtr camera =
        CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    m_camera_.push_back(camera);
  }
  if (calib_file.size() == 2) {
    std::cerr << "Warning: Two cameras detected, but FeatureTrackerMono is "
                 "designed for single camera tracking."
              << std::endl;
    std::abort();
  }
  std::cerr << "Number of cameras: " << m_camera_.size() << std::endl;
}

void FeatureTrackerMono::undistortedPts(vector<cv::Point2f> &un_pts,
                                        vector<cv::Point2f> &pts,
                                        const camodocal::CameraPtr &cam) {
  for (auto &pt : pts) {
    Eigen::Vector2d a(pt.x, pt.y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.emplace_back(b.x() / b.z(), b.y() / b.z());
  }
}

void FeatureTrackerMono::ptsVelocity(vector<cv::Point2f> &pts_velocity,
                                     double dt, vector<int> &ids,
                                     vector<cv::Point2f> &pts,
                                     map<int, cv::Point2f> &cur_id_pts,
                                     map<int, cv::Point2f> &prev_id_pts) {
  pts_velocity.clear();
  cur_id_pts.clear();
  for (unsigned i = 0; i < ids.size(); i++) {
    cur_id_pts.insert(make_pair(ids[i], pts[i]));
  }

  // caculate points velocity
  if (!prev_id_pts.empty()) {
    for (unsigned i = 0; i < pts.size(); i++) {
      std::map<int, cv::Point2f>::iterator it;
      it = prev_id_pts.find(ids[i]);
      if (it != prev_id_pts.end()) {
        double v_x = (pts[i].x - it->second.x) / dt;
        double v_y = (pts[i].y - it->second.y) / dt;
        pts_velocity.emplace_back(v_x, v_y);
      } else
        pts_velocity.emplace_back(0, 0);
    }
  } else {
    for (unsigned i = 0; i < pts.size(); i++) {
      pts_velocity.emplace_back(0, 0);
    }
  }
}

void FeatureTrackerMono::drawTrack(cv::Mat &im_track, const cv::Mat &im,
                                   vector<int> &cur_ids,
                                   vector<cv::Point2f> &cur_pts,
                                   vector<int> &track_cnt,
                                   map<int, cv::Point2f> &prev_pts_map) {
  int cols = im.cols;
  cv::cvtColor(im_track, im_track, cv::COLOR_GRAY2RGB);

  for (size_t j = 0; j < cur_pts.size(); j++) {
    double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
    cv::circle(im_track, cur_pts[j], 2,
               cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
  }

  map<int, cv::Point2f>::iterator mapIt;
  for (size_t i = 0; i < cur_ids.size(); i++) {
    int id = cur_ids[i];
    mapIt = prev_pts_map.find(id);
    if (mapIt != prev_pts_map.end()) {
      cv::arrowedLine(im_track, cur_pts[i], mapIt->second,
                      cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
    }
  }
}

cv::Mat FeatureTrackerMono::getTrackImage() { return im_track_; }

} // namespace vins::estimator