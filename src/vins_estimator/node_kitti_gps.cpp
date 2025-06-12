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

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <stdio.h>
#include <vins_estimator/estimator/estimator.h>
#include <vins_estimator/utility/visualization.h>

#include <cmath>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>

using namespace std;
using namespace Eigen;

Estimator estimator;
ros::Publisher pubGPS;

int main(int argc, char **argv) {
  ros::init(argc, argv, "vins_estimator");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);

  pubGPS = n.advertise<sensor_msgs::NavSatFix>("/gps", 1000);

  string config_file;
  if (n.getParam("config_path", config_file)) {
    ROS_INFO_STREAM("Successfully loaded config_file: " << config_file);
  } else {
    ROS_ERROR_STREAM("Failed to load config_file parameter.");
    return -1;
  }
  std::cout << "config_file: " << config_file << std::endl;

  string data_path;
  if (n.getParam("data_path", data_path)) {
    ROS_INFO_STREAM("Successfully loaded data_path: " << data_path);
  } else {
    ROS_ERROR_STREAM("Failed to load data_path parameter.");
    return -1;
  }
  if (data_path.back() != '/') {
    data_path += '/';
  }
  std::cout << "data_path: " << data_path << std::endl;

  // load image list
  FILE *file;
  file = std::fopen((data_path + "image_00/timestamps.txt").c_str(), "r");
  if (file == NULL) {
    printf("cannot find file: %simage_00/timestamps.txt \n", data_path.c_str());
    ROS_BREAK();
    return 0;
  }
  vector<double> imageTimeList;
  int year, month, day;
  int hour, minute;
  double second;
  while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute,
                &second) != EOF) {
    // printf("%lf\n", second);
    imageTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
  }
  std::fclose(file);

  // load gps list
  vector<double> GPSTimeList;
  {
    FILE *file;
    file = std::fopen((data_path + "oxts/timestamps.txt").c_str(), "r");
    if (file == NULL) {
      printf("cannot find file: %soxts/timestamps.txt \n", data_path.c_str());
      ROS_BREAK();
      return 0;
    }
    int year, month, day;
    int hour, minute;
    double second;
    while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour,
                  &minute, &second) != EOF) {
      // printf("%lf\n", second);
      GPSTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
    }
    std::fclose(file);
  }

  readParameters(config_file);
  estimator.setParameter();
  registerPub(n);

  FILE *outFile;
  outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(), "w");
  if (outFile == NULL)
    printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());
  string leftImagePath, rightImagePath;
  cv::Mat imLeft, imRight;
  double baseTime;

  for (size_t i = 0; i < imageTimeList.size(); i++) {
    if (ros::ok()) {
      if (imageTimeList[0] < GPSTimeList[0])
        baseTime = imageTimeList[0];
      else
        baseTime = GPSTimeList[0];

      // printf("base time is %f\n", baseTime);
      printf("process image %d\n", (int)i);
      stringstream ss;
      ss << setfill('0') << setw(10) << i;
      leftImagePath = data_path + "image_00/data/" + ss.str() + ".png";
      rightImagePath = data_path + "image_01/data/" + ss.str() + ".png";
      printf("%s\n", leftImagePath.c_str());
      printf("%s\n", rightImagePath.c_str());

      imLeft = cv::imread(leftImagePath, cv::IMREAD_GRAYSCALE);
      imRight = cv::imread(rightImagePath, cv::IMREAD_GRAYSCALE);

      double imgTime = imageTimeList[i] - baseTime;

      // load gps
      FILE *GPSFile;
      string GPSFilePath = data_path + "oxts/data/" + ss.str() + ".txt";
      GPSFile = std::fopen(GPSFilePath.c_str(), "r");
      if (GPSFile == NULL) {
        printf("cannot find file: %s\n", GPSFilePath.c_str());
        ROS_BREAK();
        return 0;
      }
      double lat, lon, alt, roll, pitch, yaw;
      double vn, ve, vf, vl, vu;
      double ax, ay, az, af, al, au;
      double wx, wy, wz, wf, wl, wu;
      double pos_accuracy, vel_accuracy;
      double navstat, numsats;
      double velmode, orimode;

      // TODO: proper checking of fscanf return value
      int ret;
      ret = fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &lat, &lon, &alt, &roll,
                   &pitch, &yaw);
      assert(ret == 6);
      // printf("lat:%lf lon:%lf alt:%lf roll:%lf pitch:%lf yaw:%lf \n",
      // lat, lon, alt, roll, pitch, yaw);
      ret = fscanf(GPSFile, "%lf %lf %lf %lf %lf ", &vn, &ve, &vf, &vl, &vu);
      assert(ret == 5);
      // printf("vn:%lf ve:%lf vf:%lf vl:%lf vu:%lf \n",  vn, ve, vf, vl, vu);
      ret = fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &ax, &ay, &az, &af, &al,
                   &au);
      assert(ret == 6);
      // printf("ax:%lf ay:%lf az:%lf af:%lf al:%lf au:%lf\n",  ax, ay, az, af,
      // al, au);
      ret = fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &wx, &wy, &wz, &wf, &wl,
                   &wu);
      assert(ret == 6);
      // printf("wx:%lf wy:%lf wz:%lf wf:%lf wl:%lf wu:%lf\n",  wx, wy, wz, wf,
      // wl, wu);
      ret = fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &pos_accuracy,
                   &vel_accuracy, &navstat, &numsats, &velmode, &orimode);
      assert(ret == 6);
      // printf("pos_accuracy:%lf vel_accuracy:%lf navstat:%lf numsats:%lf
      // velmode:%lf orimode:%lf\n", 	    pos_accuracy, vel_accuracy, navstat,
      // numsats, velmode, orimode);

      std::fclose(GPSFile);

      sensor_msgs::NavSatFix gps_position;
      gps_position.header.frame_id = "NED";
      gps_position.header.stamp = ros::Time(imgTime);
      gps_position.status.status = navstat;
      gps_position.status.service = numsats;
      gps_position.latitude = lat;
      gps_position.longitude = lon;
      gps_position.altitude = alt;
      gps_position.position_covariance[0] = pos_accuracy;
      // printf("pos_accuracy %f \n", pos_accuracy);
      pubGPS.publish(gps_position);

      estimator.inputImage(imgTime, imLeft, imRight);

      Eigen::Matrix<double, 4, 4> pose;
      estimator.getPoseInWorldFrame(pose);
      if (outFile != NULL)
        fprintf(outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n", pose(0, 0),
                pose(0, 1), pose(0, 2), pose(0, 3), pose(1, 0), pose(1, 1),
                pose(1, 2), pose(1, 3), pose(2, 0), pose(2, 1), pose(2, 2),
                pose(2, 3));

      // cv::imshow("leftImage", imLeft);
      // cv::imshow("rightImage", imRight);
      // cv::waitKey(2);
    } else
      break;
  }
  if (outFile != NULL) fclose(outFile);
  return 0;
}
