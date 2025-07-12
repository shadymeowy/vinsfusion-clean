#include <loop_fusion/parameters.h>

namespace vins::loop_fusion {

ros::Publisher pub_match_img;
camodocal::CameraPtr m_camera;
std::string BRIEF_PATTERN_FILE;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;

void Parameters::read_from_file(std::string config_file) {
  FILE *fh = fopen(config_file.c_str(), "r");
  if (fh == NULL) {
    ROS_WARN("config_file doesn't exist; wrong config_file path");
    ROS_BREAK();
    return;
  }
  fclose(fh);

  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  fsSettings["image0_topic"] >> image0_topic;
  fsSettings["image1_topic"] >> image1_topic;
  max_cnt = fsSettings["max_cnt"];
  min_dist = fsSettings["min_dist"];
  f_threshold = fsSettings["F_threshold"];
  show_track = fsSettings["show_track"];
  flow_back = fsSettings["flow_back"];

  multiple_thread = fsSettings["multiple_thread"];

  use_imu = fsSettings["imu"];
  printf("USE_IMU: %d\n", use_imu);
  if (use_imu) {
    fsSettings["imu_topic"] >> imu_topic;
    printf("IMU_TOPIC: %s\n", imu_topic.c_str());
    acc_n = fsSettings["acc_n"];
    acc_w = fsSettings["acc_w"];
    gyr_n = fsSettings["gyr_n"];
    gyr_w = fsSettings["gyr_w"];
    g.z() = fsSettings["g_norm"];
  }

  solver_time = fsSettings["max_solver_time"];
  num_iterations = fsSettings["max_num_iterations"];
  min_parallax = fsSettings["keyframe_parallax"];
  min_parallax /= focal_length;

  fsSettings["output_path"] >> output_folder;
  vins_result_path = output_folder + "/vio_loop.csv";
  std::cout << "result path " << vins_result_path << std::endl;
  std::ofstream fout(vins_result_path, std::ios::out);
  fout.close();

  estimate_extrinsic = fsSettings["estimate_extrinsic"];
  if (estimate_extrinsic == 2) {
    ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
    ric.push_back(Eigen::Matrix3d::Identity());
    tic.push_back(Eigen::Vector3d::Zero());
    ex_calib_result_path = output_folder + "/extrinsic_parameter.csv";
  } else {
    if (estimate_extrinsic == 1) {
      ROS_WARN(" Optimize extrinsic param around initial guess!");
      ex_calib_result_path = output_folder + "/extrinsic_parameter.csv";
    }
    if (estimate_extrinsic == 0) ROS_WARN(" fix extrinsic param ");

    cv::Mat cv_T;
    fsSettings["body_T_cam0"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    ric.push_back(T.block<3, 3>(0, 0));
    tic.push_back(T.block<3, 1>(0, 3));
  }

  num_of_cam = fsSettings["num_of_cam"];
  printf("camera number %d\n", num_of_cam);

  if (num_of_cam != 1 && num_of_cam != 2) {
    printf("num_of_cam should be 1 or 2\n");
    assert(0);
  }

  int pn = config_file.find_last_of('/');
  std::string configPath = config_file.substr(0, pn);

  std::string cam0Calib;
  fsSettings["cam0_calib"] >> cam0Calib;
  std::string cam0Path = configPath + "/" + cam0Calib;
  cam_names.push_back(cam0Path);

  if (num_of_cam == 2) {
    stereo = 1;
    std::string cam1Calib;
    fsSettings["cam1_calib"] >> cam1Calib;
    std::string cam1Path = configPath + "/" + cam1Calib;
    // printf("%s cam1 path\n", cam1Path.c_str() );
    cam_names.push_back(cam1Path);

    cv::Mat cv_T;
    fsSettings["body_T_cam1"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    ric.push_back(T.block<3, 3>(0, 0));
    tic.push_back(T.block<3, 1>(0, 3));
  }

  init_depth = 5.0;
  bias_acc_threshold = 0.1;
  bias_gyr_threshold = 0.1;

  td = fsSettings["td"];
  estimate_td = fsSettings["estimate_td"];
  if (estimate_td)
    ROS_INFO_STREAM(
        "Unsynchronized sensors, online estimate time offset, initial td: "
        << td);
  else
    ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << td);

  row = fsSettings["image_height"];
  col = fsSettings["image_width"];
  ROS_INFO("ROW: %d COL: %d ", row, col);

  if (!use_imu) {
    estimate_extrinsic = 0;
    estimate_td = 0;
    printf("no imu, fix extrinsic param; no time offset calibration\n");
  }

  fsSettings["pose_graph_save_path"] >> pose_graph_save_path;
  save_image = fsSettings["save_image"];
  load_previous_pose_graph = fsSettings["load_previous_pose_graph"];

  fsSettings.release();

  char *env_terminate_t_str = getenv("VINS_TERMINATE_TIME");
  if (env_terminate_t_str != nullptr) {
    terminate_t = atof(env_terminate_t_str);
  } else {
    // default value, no termination time set
    terminate_t = -1.0;
  }
}

}  // namespace vins::loop_fusion