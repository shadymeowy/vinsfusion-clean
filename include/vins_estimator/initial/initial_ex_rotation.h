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

#include <vins_estimator/estimator/parameters.h>

#include <vector>
using namespace std;

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
using namespace Eigen;
#include <ros/console.h>

namespace vins::estimator {

/* This class help you to calibrate extrinsic rotation between imu and camera
 * when your totally don't konw the extrinsic parameter */
class InitialEXRotation {
 public:
  InitialEXRotation(Parameters &params);
  bool CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres,
                             Quaterniond delta_q_imu,
                             Matrix3d &calib_ric_result);

 private:
  Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);

  double testTriangulation(const vector<cv::Point2f> &l,
                           const vector<cv::Point2f> &r, cv::Mat_<double> R,
                           cv::Mat_<double> t);
  void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                  cv::Mat_<double> &t1, cv::Mat_<double> &t2);

  Parameters &params;

  int frame_count;

  vector<Matrix3d> Rc;
  vector<Matrix3d> Rimu;
  vector<Matrix3d> Rc_g;
  Matrix3d ric;
};

}  // namespace vins::estimator