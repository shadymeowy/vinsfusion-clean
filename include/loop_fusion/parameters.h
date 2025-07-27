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
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace vins::loop_fusion {

extern ros::Publisher pub_match_img;
extern camodocal::CameraPtr m_camera;
extern std::string BRIEF_PATTERN_FILE;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;

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

  float terminate_t;

  void read_from_file(const std::string &config_file);
};

}  // namespace vins::loop_fusion
