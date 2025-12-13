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

#include "vins_estimator/featureTracker/tapnext_trt.h"

namespace vins::estimator {

FeatureTrackerMono::FeatureTrackerMono(Parameters &params) : params(params) {
  fast = cv::FastFeatureDetector::create();
  fast->setThreshold(0);
  fast->setNonmaxSuppression(true);
}

#include <camodocal/camera_models/Camera.h>
#include <camodocal/camera_models/CameraFactory.h>

#include <algorithm>

cv::Mat getOptimalRectifyMatrix(const camodocal::CameraPtr &cam,
                                const cv::Size &imgSize) {
  int w = imgSize.width;
  int h = imgSize.height;

  // initialize inner box limits
  double inner_left = -std::numeric_limits<double>::infinity();
  double inner_right = std::numeric_limits<double>::infinity();
  double inner_top = -std::numeric_limits<double>::infinity();
  double inner_bottom = std::numeric_limits<double>::infinity();

  // helper for getting normalized coordinates
  auto get_norm = [&](int u, int v) -> Eigen::Vector2d {
    Eigen::Vector2d p(u, v);
    Eigen::Vector3d ray;
    cam->liftProjective(p, ray);
    if (ray.z() <= 0) return Eigen::Vector2d(0, 0);  // Safety
    return Eigen::Vector2d(ray.x() / ray.z(), ray.y() / ray.z());
  };

  // scan left and right edges to get inner x limits
  for (int v = 0; v < h; ++v) {
    Eigen::Vector2d n_left = get_norm(0, v);
    Eigen::Vector2d n_right = get_norm(w - 1, v);
    if (n_left.x() > inner_left) inner_left = n_left.x();
    if (n_right.x() < inner_right) inner_right = n_right.x();
  }

  // scan top and bottom edges to get inner y limits
  for (int u = 0; u < w; ++u) {
    Eigen::Vector2d n_top = get_norm(u, 0);
    Eigen::Vector2d n_bot = get_norm(u, h - 1);
    if (n_top.y() > inner_top) inner_top = n_top.y();
    if (n_bot.y() < inner_bottom) inner_bottom = n_bot.y();
  }

  // calculate mid points and ranges
  double mid_x = (inner_left + inner_right) / 2.0;
  double mid_y = (inner_top + inner_bottom) / 2.0;

  // calculate focal length
  double fx = w / (inner_right - inner_left);
  double fy = h / (inner_bottom - inner_top);
  double f = std::max(fx, fy);

  // derive principal point
  double cx = (w / 2.0) - (f * mid_x);
  double cy = (h / 2.0) - (f * mid_y);

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = f;
  K.at<float>(1, 1) = f;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  return K;
}

cv::Point2f rectifiedToDistorted(const cv::Point2f &pt_rect,
                                 const camodocal::CameraPtr &cam,
                                 const cv::Mat &K_new, const cv::Mat &R) {
  float fx = K_new.at<float>(0, 0);
  float fy = K_new.at<float>(1, 1);
  float cx = K_new.at<float>(0, 2);
  float cy = K_new.at<float>(1, 2);

  double x_rect = (pt_rect.x - cx) / fx;
  double y_rect = (pt_rect.y - cy) / fy;
  double z_rect = 1.0;

  Eigen::Vector3d ray_rect(x_rect, y_rect, z_rect);
  Eigen::Matrix3d R_eigen;
  cv::cv2eigen(R, R_eigen);

  Eigen::Vector3d ray_cam = R_eigen.transpose() * ray_rect;
  Eigen::Vector2d pt_distorted_eigen;
  cam->spaceToPlane(ray_cam, pt_distorted_eigen);

  return cv::Point2f(static_cast<float>(pt_distorted_eigen.x()),
                     static_cast<float>(pt_distorted_eigen.y()));
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
FeatureTrackerMono::trackImage(double cur_time, const cv::Mat &cur_img,
                               const cv::Mat &) {
  TicToc t_r;

  // undistort image
  cv::Mat undist_img;
  cv::remap(cur_img, undist_img, undist_map1_, undist_map2_, cv::INTER_LINEAR);

  // if first frame or need to reset tracker
  static bool first_frame = true;
  if (first_frame) {
    resetTracker(undist_img);
    first_frame = false;
  }
  auto [pts, vis] = tapnext_trt_->run(undist_img);

  // draw track image
  if (params.show_track) {
    cv::Mat im_track_undist, im_track_dist;
    cvtColor(undist_img, im_track_undist, cv::COLOR_GRAY2BGR);
    cvtColor(cur_img, im_track_dist, cv::COLOR_GRAY2BGR);
    for (size_t k = 0; k < pts.size(); ++k) {
      if (vis[k]) {
        const auto &p = pts[k];
        if (p.x >= 0 && p.x < cur_img.cols && p.y >= 0 && p.y < cur_img.rows) {
          cv::circle(im_track_undist,
                     cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)), 3,
                     cv::Scalar(0, 255, 0), -1);
        }
        auto &camera = m_camera_[0];
        cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
        cv::Point2f p_dist = rectifiedToDistorted(p, camera, K_new_, R);
        if (p_dist.x >= 0 && p_dist.x < cur_img.cols && p_dist.y >= 0 &&
            p_dist.y < cur_img.rows) {
          cv::circle(
              im_track_dist,
              cv::Point(static_cast<int>(p_dist.x), static_cast<int>(p_dist.y)),
              3, cv::Scalar(0, 255, 0), -1);
        }
      }
    }
    cv::imshow("track image", im_track_undist);
    cv::imshow("track image distorted", im_track_dist);
  }

  // show undistorted image
  cv::imshow("undistorted image", undist_img);
  // show distorted image
  cv::imshow("distorted image", cur_img);
  cv::waitKey(1);

  vector<float> cur_x(params.max_cnt, 0);
  vector<float> cur_y(params.max_cnt, 0);
  vector<int> ids(params.max_cnt, 0);
  vector<int> track_cnt(params.max_cnt, 0);

  vector<cv::Point2f> cur_pts;

  //   int len = track_klt_cy(cur_img.data, width, height, cur_x.data(),
  //                          cur_y.data(), ids.data(), track_cnt.data(),
  //                          params.min_dist, params.max_cnt,
  //                          params.flow_back);
  int len = 0;

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

void FeatureTrackerMono::resetTracker(const cv::Mat &cur_img) {
  // Detect keypoints from FAST
  std::vector<cv::KeyPoint> kps;
  fast->detect(cur_img, kps);
  std::cout << "FAST keypoints detected: " << kps.size() << std::endl;

  // export keypoints to vectors
  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> response;
  xs.reserve(kps.size());
  ys.reserve(kps.size());
  response.reserve(kps.size());
  for (const auto &kp : kps) {
    xs.push_back(kp.pt.x);
    ys.push_back(kp.pt.y);
    response.push_back(kp.response);
  }

  std::vector<bool> anms_result;
  auto &camera = m_camera_[0];
  anms.run(xs, ys, response, anms_result, camera->imageHeight(),
           camera->imageWidth(), 256);

  std::vector<float> xs_model;
  std::vector<float> ys_model;
  for (size_t i = 0; i < anms_result.size(); ++i) {
    if (anms_result[i]) {
      xs_model.push_back(xs[i]);
      ys_model.push_back(ys[i]);
    }
  }
  std::cout << "ANMS keypoints selected: " << xs_model.size() << std::endl;

  tapnext_trt_->reset(xs_model, ys_model);
}

void FeatureTrackerMono::readIntrinsicParameter(
    const vector<string> &calib_file) {
  for (const auto &i : calib_file) {
    ROS_INFO("reading paramerter of camera %s", i.c_str());
    camodocal::CameraPtr camera =
        CameraFactory::instance()->generateCameraFromYamlFile(i);
    m_camera_.push_back(camera);
  }
  if (calib_file.size() == 2) {
    std::cerr << "Warning: Two cameras detected, but FeatureTrackerMono is "
                 "designed for single camera tracking."
              << std::endl;
    // std::abort();
  }
  std::cerr << "Number of cameras: " << m_camera_.size() << std::endl;

  // get the first camera's image size
  // TODO(shady): select camera from params
  // TODO(shady): select track count from params
  auto &camera = m_camera_[0];
  tapnext_trt_ = std::make_unique<TAPNextTRT>(
      params.tapnext_onnx_path, params.tapnext_engine_path, 256,
      camera->imageWidth(), camera->imageHeight());

  // precompute undistort maps
  auto imageSize = cv::Size(camera->imageWidth(), camera->imageHeight());
  K_new_ = getOptimalRectifyMatrix(camera, imageSize);

  float new_fx = K_new_.at<float>(0, 0);
  float new_fy = K_new_.at<float>(1, 1);
  float new_cx = K_new_.at<float>(0, 2);
  float new_cy = K_new_.at<float>(1, 2);

  // 2. Pass these to camodocal's function
  camera->initUndistortRectifyMap(undist_map1_, undist_map2_, new_fx, new_fy,
                                  imageSize, new_cx, new_cy,
                                  cv::Mat::eye(3, 3, CV_32F));
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

void FeatureTrackerMono::removeOutliers(set<int> & /*removePtsIds*/) {
  throw std::runtime_error(
      "FeatureTrackerMono::removeOutliers not implemented yet.");
}

}  // namespace vins::estimator