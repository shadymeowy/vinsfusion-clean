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
#include <vins_estimator/utility/tic_toc.h>

#include <csignal>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace camodocal;
using namespace Eigen;

namespace vins::estimator {

class FeatureTrackerMono {
public:
  explicit FeatureTrackerMono(Parameters &params);
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
  trackImage(double _cur_time, const cv::Mat &_img,
             const cv::Mat &_img1 = cv::Mat());
  void readIntrinsicParameter(const vector<string> &calib_file);

  void setPrediction(map<int, Eigen::Vector3d> &predictPts) {
    std::cerr << "setPrediction is not implemented for FeatureTrackerMono."
              << std::endl;
    std::abort();
  }

  void removeOutliers(set<int> &removePtsIds) {
    std::cerr << "removeOutliers is not implemented for FeatureTrackerMono."
              << std::endl;
    std::abort();
  }
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

  Parameters &params;

  cv::Mat im_track_;

  vector<cv::Point2f> prev_un_pts_;

  map<int, cv::Point2f> prev_un_pts_map_;
  map<int, cv::Point2f> prev_pts_map_;
  vector<camodocal::CameraPtr> m_camera_;
  double prev_time_;
};

} // namespace vins::estimator