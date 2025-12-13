#pragma once

#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <execinfo.h>
#include <vins_estimator/estimator/parameters.h>
#include <vins_estimator/featureTracker/feature_tracker_klt.h>
#include <vins_estimator/featureTracker/feature_tracker_tapnext.h>
#include <vins_estimator/utility/tic_toc.h>

#include <csignal>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace camodocal;
using namespace Eigen;

namespace vins::estimator {

// header only tracker glue
class FeatureTracker {
 public:
  explicit FeatureTracker(Parameters &params)
      : klt_tracker_(params), tapnext_tracker_(params), params(params) {}

  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(
      double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat()) {
    // check is tapnext enabled
    if (!params.tapnext_enable) {
      return klt_tracker_.trackImage(_cur_time, _img, _img1);
    }

    auto klt = klt_tracker_.trackImage(_cur_time, _img, _img1);
    auto tapnext = tapnext_tracker_.trackImage(_cur_time, _img, _img1);

    // merge results
    // note that ids are unique across both trackers
    for (auto &it : tapnext) {
      klt[it.first] = it.second;
    }
    return klt;
  }

  void readIntrinsicParameter(const vector<string> &calib_file) {
    klt_tracker_.readIntrinsicParameter(calib_file);
    tapnext_tracker_.readIntrinsicParameter(calib_file);
  }

  void setPrediction(map<int, Eigen::Vector3d> &predictPts) {
    klt_tracker_.setPrediction(predictPts);
    // tapnext_tracker_.setPrediction(predictPts);
  }
  void removeOutliers(set<int> &removePtsIds) {
    klt_tracker_.removeOutliers(removePtsIds);
    // tapnext_tracker_.removeOutliers(removePtsIds);
  }
  cv::Mat getTrackImage() {
    auto klt_image = klt_tracker_.getTrackImage();

    cv::Mat image;
    if (params.tapnext_enable) {
      auto tapnext_image = tapnext_tracker_.getTrackImage();
      cv::hconcat(klt_image, tapnext_image, image);
    } else {
      image = klt_image;
    }
    return image;
  }

 private:
  FeatureTrackerKLT klt_tracker_;
  FeatureTrackerTAPNext tapnext_tracker_;
  Parameters &params;
};

}  // namespace vins::estimator