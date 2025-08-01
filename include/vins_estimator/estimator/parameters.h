/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <vins_estimator/utility/utility.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace vins::estimator {

#define WINDOW_SIZE 10
#define NUM_OF_F 4096

enum LossType {
  LOSS_L2 = 0,
  LOSS_HUBER = 1,
  LOSS_TUKEY = 2,
  LOSS_CAUCHY = 3,
};

struct Parameters {
  double focal_length;

  double init_depth;
  double min_parallax;
  double acc_n, acc_w;
  double gyr_n, gyr_w;

  std::vector<Eigen::Matrix3d> ric;
  std::vector<Eigen::Vector3d> tic;

  Eigen::Vector3d g{0.0, 0.0, 9.8};

  double bias_acc_threshold;
  double bias_gyr_threshold;
  double solver_time;
  int num_iterations;
  int estimate_extrinsic;
  int estimate_td;
  int rolling_shutter;
  std::string ex_calib_result_path;
  std::string vins_result_path;
  std::string output_folder;
  std::string imu_topic;
  int row, col;
  double td;
  int num_of_cam;
  int stereo;
  int use_imu;
  int multiple_thread;

  std::string image0_topic, image1_topic;
  std::string fisheye_mask;
  std::vector<std::string> cam_names;
  int max_cnt;
  int min_dist;
  double f_threshold;
  int show_track;
  int flow_back;

  std::string pose_graph_save_path;
  int save_image;
  int load_previous_pose_graph;

  double terminate_t;

  int feature_debug;
  std::string feature_debug_path;

  LossType loss_type;
  double loss_parameter;
  LossType loss_type_initial;
  double loss_parameter_initial;

  void read_from_file(const std::string &config_file);
};

enum SIZE_PARAMETERIZATION {
  SIZE_POSE = 7,
  SIZE_SPEEDBIAS = 9,
  SIZE_FEATURE = 1
};

enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };

}  // namespace vins::estimator