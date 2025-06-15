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

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker {
 public:
  explicit FeatureTracker(Parameters &params);
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(
      double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
  void readIntrinsicParameter(const vector<string> &calib_file);
  void setPrediction(map<int, Eigen::Vector3d> &predictPts);
  void removeOutliers(set<int> &removePtsIds);
  cv::Mat getTrackImage();

 private:
  void setMask();
  void showUndistortion(const string &name);
  void rejectWithF();
  static vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts,
                                            const camodocal::CameraPtr &cam);
  vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                                  map<int, cv::Point2f> &cur_id_pts,
                                  map<int, cv::Point2f> &prev_id_pts);
  void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                 vector<int> &curLeftIds, vector<cv::Point2f> &curLeftPts,
                 vector<cv::Point2f> &curRightPts,
                 map<int, cv::Point2f> &prevLeftPtsMap);
  bool inBorder(const cv::Point2f &pt) const;
  static double distance(const cv::Point2f &pt1, const cv::Point2f &pt2);

  Parameters &params;

  int row, col;
  cv::Mat im_track_;
  cv::Mat mask_;
  cv::Mat fisheye_mask_;
  cv::Mat prev_img_, cur_img_;

  vector<cv::Point2f> predict_pts_;
  vector<cv::Point2f> predict_pts_debug_;
  vector<cv::Point2f> prev_pts_, cur_pts_, cur_right_pts_;
  vector<cv::Point2f> prev_un_pts_, cur_un_pts_, cur_un_right_pts_;
  vector<cv::Point2f> pts_velocity_, right_pts_velocity_;
  vector<int> ids_, ids_right_;
  vector<int> track_cnt_;
  map<int, cv::Point2f> cur_un_pts_map_, prev_un_pts_map_;
  map<int, cv::Point2f> cur_un_right_pts_map_, prev_un_right_pts_map_;
  map<int, cv::Point2f> prev_left_pts_map_;
  vector<camodocal::CameraPtr> m_camera_;
  double cur_time_;
  double prev_time_;
  bool stereo_cam_;
  int n_id_;
  bool has_prediction_;
};

}  // namespace vins::estimator