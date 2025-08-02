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

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <vins_estimator/estimator/feature_manager.h>
#include <vins_estimator/estimator/parameters.h>
#include <vins_estimator/factor/imu_factor.h>
#include <vins_estimator/factor/marginalization_factor.h>
#include <vins_estimator/factor/pose_local_parameterization.h>
#include <vins_estimator/factor/projectionOneFrameTwoCamFactor.h>
#include <vins_estimator/factor/projectionTwoFrameOneCamFactor.h>
#include <vins_estimator/factor/projectionTwoFrameTwoCamFactor.h>
#include <vins_estimator/featureTracker/feature_tracker_mono.h>
#include <vins_estimator/initial/initial_alignment.h>
#include <vins_estimator/initial/initial_ex_rotation.h>
#include <vins_estimator/initial/initial_sfm.h>
#include <vins_estimator/initial/solve_5pts.h>
#include <vins_estimator/utility/tic_toc.h>
#include <vins_estimator/utility/utility.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <thread>
#include <unordered_map>

namespace vins::estimator {

class Estimator {
 public:
  explicit Estimator(Parameters &params);
  ~Estimator();
  void setParameter();

  // interface
  void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
  void inputIMU(double t, const Vector3d &linearAcceleration,
                const Vector3d &angularVelocity);
  void inputFeature(
      double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
                    &featureFrame);
  void inputImage(double t, const cv::Mat &_img,
                  const cv::Mat &_img1 = cv::Mat());
  void processIMU(double t, double dt, const Vector3d &linear_acceleration,
                  const Vector3d &angular_velocity);
  void processImage(
      const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
      double header);
  void processMeasurements();
  void changeSensorType(int use_imu, int use_stereo);

  // internal
  void clearState();
  bool initialStructure();
  bool visualInitialAlign();
  bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
  void slideWindow();
  void slideWindowNew();
  void slideWindowOld();
  void optimization();
  void vector2double();
  void double2vector();
  bool failureDetection();
  bool getIMUInterval(double t0, double t1,
                      vector<pair<double, Eigen::Vector3d>> &accVector,
                      vector<pair<double, Eigen::Vector3d>> &gyrVector);
  void getPoseInWorldFrame(Eigen::Matrix4d &T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
  void predictPtsInNextFrame();
  void outliersRejection(set<int> &removeIndex);
  double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici,
                           Vector3d &tici, Matrix3d &Rj, Vector3d &Pj,
                           Matrix3d &ricj, Vector3d &ticj, double depth,
                           Vector3d &uvi, Vector3d &uvj);
  void updateLatestStates();
  void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration,
                      Eigen::Vector3d angular_velocity);
  bool IMUAvailable(double t);
  void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

  enum SolverFlag { INITIAL, NON_LINEAR };

  enum MarginalizationFlag { MARGIN_OLD = 0, MARGIN_SECOND_NEW = 1 };

  Parameters &params;

  std::mutex mProcess;
  std::mutex mBuf;
  std::mutex mPropagate;
  queue<pair<double, Eigen::Vector3d>> accBuf;
  queue<pair<double, Eigen::Vector3d>> gyrBuf;
  queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>>
      featureBuf;
  double prevTime = 0.0;
  double curTime = 0.0;
  bool openExEstimation = false;

  std::thread trackThread;
  std::thread processThread;

  FeatureTrackerMono featureTracker;

  SolverFlag solver_flag;
  MarginalizationFlag marginalization_flag;
  Vector3d g;

  Matrix3d ric[2];
  Vector3d tic[2];

  Vector3d Ps[(WINDOW_SIZE + 1)];
  Vector3d Vs[(WINDOW_SIZE + 1)];
  Matrix3d Rs[(WINDOW_SIZE + 1)];
  Vector3d Bas[(WINDOW_SIZE + 1)];
  Vector3d Bgs[(WINDOW_SIZE + 1)];
  double td = 0.0;

  Matrix3d back_R0, last_R, last_R0;
  Vector3d back_P0, last_P, last_P0;
  double Headers[(WINDOW_SIZE + 1)];

  IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)] = {};
  Vector3d acc_0, gyr_0;

  vector<double> dt_buf[(WINDOW_SIZE + 1)];
  vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
  vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

  int frame_count = 0;
  int sum_of_outlier = 0, sum_of_back = 0, sum_of_front = 0, sum_of_invalid = 0;
  int inputImageCnt = 0;

  FeatureManager f_manager;
  MotionEstimator m_estimator;
  InitialEXRotation initial_ex_rotation;

  bool first_imu = false;
  bool is_valid = false, is_key = false;
  bool failure_occur = false;

  vector<Vector3d> point_cloud;
  vector<Vector3d> margin_cloud;
  vector<Vector3d> key_poses;
  double initial_timestamp = 0.0;

  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  double para_Feature[NUM_OF_F][SIZE_FEATURE];
  double para_Ex_Pose[2][SIZE_POSE];
  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1];
  double para_Tr[1][1];

  MarginalizationInfo *last_marginalization_info = nullptr;
  vector<double *> last_marginalization_parameter_blocks;

  map<double, ImageFrame> all_image_frame;
  IntegrationBase *tmp_pre_integration = nullptr;

  Eigen::Vector3d initP;
  Eigen::Matrix3d initR;

  double latest_time = 0.0;
  Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0,
      latest_gyr_0;
  Eigen::Quaterniond latest_Q;

  bool initFirstPoseFlag = false;
  bool initThreadFlag = false;
};

}  // namespace vins::estimator