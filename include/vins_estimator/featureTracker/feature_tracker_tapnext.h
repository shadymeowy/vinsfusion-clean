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

#pragma once

#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <execinfo.h>
#include <vins_estimator/estimator/parameters.h>
#include <vins_estimator/featureTracker/anms.h>
#include <vins_estimator/featureTracker/id_counter.h>
#include <vins_estimator/featureTracker/tapnext_trt.h>
#include <vins_estimator/utility/tic_toc.h>

#include <csignal>
#include <cstddef>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace camodocal;
using namespace Eigen;

namespace vins::estimator {

class FeatureTrackerTAPNext {
 public:
  explicit FeatureTrackerTAPNext(Parameters &params);
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(
      double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
  void readIntrinsicParameter(const vector<string> &calib_file);

  static void setPrediction(map<int, Eigen::Vector3d> & /*predictPts*/) {
    std::cerr << "setPrediction is not implemented for FeatureTrackerMono."
              << std::endl;
    std::abort();
  }

  static void removeOutliers(set<int> &removePtsIds);
  cv::Mat getTrackImage();

 private:
  static void undistortedPts(vector<cv::Point2f> &un_pts,
                             vector<cv::Point2f> &pts,
                             const camodocal::CameraPtr &cam);
  static void ptsVelocity(vector<cv::Point2f> &pts_velocity, double dt,
                          vector<int> &ids, vector<cv::Point2f> &pts,
                          map<int, cv::Point2f> &cur_id_pts,
                          map<int, cv::Point2f> &prev_id_pts);
  static void drawTrack(cv::Mat &im_track, const cv::Mat &im,
                        vector<int> &cur_ids, vector<cv::Point2f> &cur_pts,
                        vector<int> &track_cnt,
                        map<int, cv::Point2f> &prev_pts_map);
  void resetTracker(const cv::Mat &cur_img);
  bool shouldResetTracker();

  Parameters &params;

  // lazy initialize TAPNextTRT
  std::unique_ptr<TAPNextTRT> tapnext_trt_{nullptr};
  // fast detector
  cv::Ptr<cv::FastFeatureDetector> fast;
  // adaptive non-maximal suppression
  AdaptiveNMS anms;
  // rectification maps
  cv::Mat K_new_;
  cv::Mat undist_map1_;
  cv::Mat undist_map2_;
  // current state
  vector<float> cur_model_x_;
  vector<float> cur_model_y_;
  vector<int> ids_;
  vector<int> track_cnt_;
  vector<bool> status_;
  int last_reset_counter_ = 0;

  cv::Mat im_track_;

  vector<cv::Point2f> prev_un_pts_;
  map<int, cv::Point2f> prev_un_pts_map_;
  map<int, cv::Point2f> prev_pts_map_;
  vector<camodocal::CameraPtr> m_camera_;
  double prev_time_;
};

}  // namespace vins::estimator