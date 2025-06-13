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
#include <ros/ros.h>
#include <vins_estimator/estimator/feature_manager.h>
#include <vins_estimator/factor/imu_factor.h>
#include <vins_estimator/utility/utility.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>

using namespace Eigen;
using namespace std;

namespace vins::estimator {

class ImageFrame {
 public:
  ImageFrame() {};
  ImageFrame(
      const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_points,
      double _t)
      : t{_t}, is_key_frame{false} {
    points = _points;
  };
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;
  double t;
  Matrix3d R;
  Vector3d T;
  IntegrationBase *pre_integration;
  bool is_key_frame;
};
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame,
                        Vector3d *Bgs);
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d *Bgs,
                        Vector3d &g, VectorXd &x, const Eigen::Vector3d &G, const Eigen::Vector3d &tic);

} // namespace vins::estimator