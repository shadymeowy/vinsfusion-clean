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

#include <vins_estimator/featureTracker/feature_tracker.h>

namespace vins::estimator {

bool FeatureTracker::inBorder(const cv::Point2f &pt) const {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(const cv::Point2f &pt1, const cv::Point2f &pt2) {
  // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < static_cast<int>(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < static_cast<int>(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

FeatureTracker::FeatureTracker(Parameters &params)
    : params(params), stereo_cam_(false), n_id_(0), has_prediction_(false) {}

void FeatureTracker::setMask() {
  mask_ = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

  // prefer to keep features that are tracked for long time
  vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned i = 0; i < cur_pts_.size(); i++)
    cnt_pts_id.emplace_back(track_cnt_[i], make_pair(cur_pts_[i], ids_[i]));

  sort(cnt_pts_id.begin(), cnt_pts_id.end(),
       [](const pair<int, pair<cv::Point2f, int>> &a,
          const pair<int, pair<cv::Point2f, int>> &b) {
         return a.first > b.first;
       });

  cur_pts_.clear();
  ids_.clear();
  track_cnt_.clear();

  for (auto &it : cnt_pts_id) {
    if (mask_.at<uchar>(it.second.first) == 255) {
      cur_pts_.push_back(it.second.first);
      ids_.push_back(it.second.second);
      track_cnt_.push_back(it.first);
      cv::circle(mask_, it.second.first, params.min_dist, 0, -1);
    }
  }
}

double FeatureTracker::distance(const cv::Point2f &pt1,
                                const cv::Point2f &pt2) {
  // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>
FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img,
                           const cv::Mat &_img1) {
  TicToc t_r;
  cur_time_ = _cur_time;
  cur_img_ = _img;
  row = cur_img_.rows;
  col = cur_img_.cols;
  const cv::Mat &rightImg = _img1;
  /*
  {
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
      clahe->apply(cur_img, cur_img);
      if(!rightImg.empty())
          clahe->apply(rightImg, rightImg);
  }
  */
  cur_pts_.clear();

  if (prev_pts_.size() > 0) {
    TicToc t_o;
    vector<uchar> status;
    vector<float> err;
    if (has_prediction_) {
      cur_pts_ = predict_pts_;
      cv::calcOpticalFlowPyrLK(
          prev_img_, cur_img_, prev_pts_, cur_pts_, status, err,
          cv::Size(21, 21), 1,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                           0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);

      int succ_num = 0;
      for (unsigned char statu : status) {
        if (statu) succ_num++;
      }
      if (succ_num < 10)
        cv::calcOpticalFlowPyrLK(prev_img_, cur_img_, prev_pts_, cur_pts_,
                                 status, err, cv::Size(21, 21), 3);
    } else
      cv::calcOpticalFlowPyrLK(prev_img_, cur_img_, prev_pts_, cur_pts_, status,
                               err, cv::Size(21, 21), 3);
    // reverse check
    if (params.flow_back) {
      vector<uchar> reverse_status;
      vector<cv::Point2f> reverse_pts = prev_pts_;
      cv::calcOpticalFlowPyrLK(
          cur_img_, prev_img_, cur_pts_, reverse_pts, reverse_status, err,
          cv::Size(21, 21), 1,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                           0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);
      // cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts,
      // reverse_status, err, cv::Size(21, 21), 3);
      for (size_t i = 0; i < status.size(); i++) {
        if (status[i] && reverse_status[i] &&
            distance(prev_pts_[i], reverse_pts[i]) <= 0.5) {
          status[i] = 1;
        } else
          status[i] = 0;
      }
    }

    for (int i = 0; i < static_cast<int>(cur_pts_.size()); i++)
      if (status[i] && !inBorder(cur_pts_[i])) status[i] = 0;
    reduceVector(prev_pts_, status);
    reduceVector(cur_pts_, status);
    reduceVector(ids_, status);
    reduceVector(track_cnt_, status);
    ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    // printf("track cnt %d\n", (int)ids.size());
  }

  for (auto &n : track_cnt_) n++;

  // if (true)
  {
    // rejectWithF();
    ROS_DEBUG("set mask begins");
    TicToc t_m;
    setMask();
    ROS_DEBUG("set mask costs %fms", t_m.toc());

    ROS_DEBUG("detect feature begins");
    TicToc t_t;
    int n_max_cnt = params.max_cnt - static_cast<int>(cur_pts_.size());
    if (n_max_cnt > 0) {
      if (mask_.empty()) cout << "mask is empty " << endl;
      if (mask_.type() != CV_8UC1) cout << "mask type wrong " << endl;

      vector<cv::Point2f> n_pts;
      n_pts.reserve(n_max_cnt);
      cv::goodFeaturesToTrack(cur_img_, n_pts, n_max_cnt, 0.01,
                              params.min_dist, mask_);
      for (auto &p : n_pts) {
        cur_pts_.push_back(p);
        ids_.push_back(n_id_++);
        track_cnt_.push_back(1);
      }
    }

    ROS_DEBUG("detect feature costs: %f ms", t_t.toc());
    // printf("feature cnt after add %d\n", (int)ids.size());
  }

  cur_un_pts_ = undistortedPts(cur_pts_, m_camera_[0]);
  pts_velocity_ =
      ptsVelocity(ids_, cur_un_pts_, cur_un_pts_map_, prev_un_pts_map_);

  if (!_img1.empty() && stereo_cam_) {
    ids_right_.clear();
    cur_right_pts_.clear();
    cur_un_right_pts_.clear();
    right_pts_velocity_.clear();
    cur_un_right_pts_map_.clear();
    if (!cur_pts_.empty()) {
      // printf("stereo image; track feature on right image\n");
      vector<cv::Point2f> reverseLeftPts;
      vector<uchar> status;
      vector<uchar> statusRightLeft;
      vector<float> err;
      // cur left ---- cur right
      cv::calcOpticalFlowPyrLK(cur_img_, rightImg, cur_pts_, cur_right_pts_,
                               status, err, cv::Size(21, 21), 3);
      // reverse check cur right ---- cur left
      if (params.flow_back) {
        cv::calcOpticalFlowPyrLK(rightImg, cur_img_, cur_right_pts_,
                                 reverseLeftPts, statusRightLeft, err,
                                 cv::Size(21, 21), 3);
        for (size_t i = 0; i < status.size(); i++) {
          if (status[i] && statusRightLeft[i] && inBorder(cur_right_pts_[i]) &&
              distance(cur_pts_[i], reverseLeftPts[i]) <= 0.5)
            status[i] = 1;
          else
            status[i] = 0;
        }
      }

      ids_right_ = ids_;
      reduceVector(cur_right_pts_, status);
      reduceVector(ids_right_, status);
      // only keep left-right pts
      /*
      reduceVector(cur_pts, status);
      reduceVector(ids, status);
      reduceVector(track_cnt, status);
      reduceVector(cur_un_pts, status);
      reduceVector(pts_velocity, status);
      */
      cur_un_right_pts_ = undistortedPts(cur_right_pts_, m_camera_[1]);
      right_pts_velocity_ =
          ptsVelocity(ids_right_, cur_un_right_pts_, cur_un_right_pts_map_,
                      prev_un_right_pts_map_);
    }
    prev_un_right_pts_map_ = cur_un_right_pts_map_;
  }
  if (params.show_track)
    drawTrack(cur_img_, rightImg, ids_, cur_pts_, cur_right_pts_,
              prev_left_pts_map_);

  prev_img_ = cur_img_;
  prev_pts_ = cur_pts_;
  prev_un_pts_ = cur_un_pts_;
  prev_un_pts_map_ = cur_un_pts_map_;
  prev_time_ = cur_time_;
  has_prediction_ = false;

  prev_left_pts_map_.clear();
  for (size_t i = 0; i < cur_pts_.size(); i++)
    prev_left_pts_map_[ids_[i]] = cur_pts_[i];

  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
  for (size_t i = 0; i < ids_.size(); i++) {
    int feature_id = ids_[i];
    double x = cur_un_pts_[i].x;
    double y = cur_un_pts_[i].y;
    double z = 1;
    double p_u = cur_pts_[i].x;
    double p_v = cur_pts_[i].y;
    int camera_id = 0;
    double velocity_x = pts_velocity_[i].x;
    double velocity_y = pts_velocity_[i].y;

    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
  }

  if (!_img1.empty() && stereo_cam_) {
    for (size_t i = 0; i < ids_right_.size(); i++) {
      int feature_id = ids_right_[i];
      double x = cur_un_right_pts_[i].x;
      double y = cur_un_right_pts_[i].y;
      double z = 1;
      double p_u = cur_right_pts_[i].x;
      double p_v = cur_right_pts_[i].y;
      int camera_id = 1;
      double velocity_x = right_pts_velocity_[i].x;
      double velocity_y = right_pts_velocity_[i].y;

      Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
      xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
      featureFrame[feature_id].emplace_back(camera_id, xyz_uv_velocity);
    }
  }

  // printf("feature track whole time %f\n", t_r.toc());
  return featureFrame;
}

void FeatureTracker::rejectWithF() {
  if (cur_pts_.size() >= 8) {
    ROS_DEBUG("FM ransac begins");
    TicToc t_f;
    vector<cv::Point2f> un_cur_pts(cur_pts_.size());
    vector<cv::Point2f> un_prev_pts(prev_pts_.size());

    for (unsigned i = 0; i < cur_pts_.size(); i++) {
      Eigen::Vector3d tmp_p;
      m_camera_[0]->liftProjective(
          Eigen::Vector2d(cur_pts_[i].x, cur_pts_[i].y), tmp_p);
      tmp_p.x() = params.focal_length * tmp_p.x() / tmp_p.z() + col / 2.0;
      tmp_p.y() = params.focal_length * tmp_p.y() / tmp_p.z() + row / 2.0;
      un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      m_camera_[0]->liftProjective(
          Eigen::Vector2d(prev_pts_[i].x, prev_pts_[i].y), tmp_p);
      tmp_p.x() = params.focal_length * tmp_p.x() / tmp_p.z() + col / 2.0;
      tmp_p.y() = params.focal_length * tmp_p.y() / tmp_p.z() + row / 2.0;
      un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC,
                           params.f_threshold, 0.99, status);
    int size_a = cur_pts_.size();
    reduceVector(prev_pts_, status);
    reduceVector(cur_pts_, status);
    reduceVector(cur_un_pts_, status);
    reduceVector(ids_, status);
    reduceVector(track_cnt_, status);
    ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts_.size(),
              1.0 * cur_pts_.size() / size_a);
    ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
  }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file) {
  for (size_t i = 0; i < calib_file.size(); i++) {
    ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
    camodocal::CameraPtr camera =
        CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    m_camera_.push_back(camera);
  }
  if (calib_file.size() == 2) stereo_cam_ = true;
}

void FeatureTracker::showUndistortion(const string & /*name*/) {
  cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
  vector<Eigen::Vector2d> distortedp;
  vector<Eigen::Vector2d> undistortedp;

  for (int i = 0; i < col; i++)
    for (int j = 0; j < row; j++) {
      Eigen::Vector2d a(i, j);
      Eigen::Vector3d b;
      m_camera_[0]->liftProjective(a, b);
      distortedp.push_back(a);
      undistortedp.emplace_back(b.x() / b.z(), b.y() / b.z());
      // printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
    }

  for (int i = 0; i < static_cast<int>(undistortedp.size()); i++) {
    cv::Mat pp(3, 1, CV_32FC1);
    pp.at<float>(0, 0) = undistortedp[i].x() * params.focal_length + col / 2;
    pp.at<float>(1, 0) = undistortedp[i].y() * params.focal_length + row / 2;
    pp.at<float>(2, 0) = 1.0;
    // cout << trackerData[0].K << endl;
    // printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
    // printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
    if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 &&
        pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600) {
      undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300,
                               pp.at<float>(0, 0) + 300) =
          cur_img_.at<uchar>(distortedp[i].y(), distortedp[i].x());
    } else {
      // ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x,
      // pp.at<float>(1, 0), pp.at<float>(0, 0));
    }
  }
  // turn the following code on if you need
  // cv::imshow(name, undistortedImg);
  // cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(
    vector<cv::Point2f> &pts, const camodocal::CameraPtr &cam) {
  vector<cv::Point2f> un_pts;
  for (auto &pt : pts) {
    Eigen::Vector2d a(pt.x, pt.y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.emplace_back(b.x() / b.z(), b.y() / b.z());
  }
  return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(
    vector<int> &ids, vector<cv::Point2f> &pts,
    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts) {
  vector<cv::Point2f> pts_velocity;
  cur_id_pts.clear();
  for (unsigned i = 0; i < ids.size(); i++) {
    cur_id_pts.insert(make_pair(ids[i], pts[i]));
  }

  // caculate points velocity
  if (!prev_id_pts.empty()) {
    double dt = cur_time_ - prev_time_;

    for (unsigned i = 0; i < pts.size(); i++) {
      std::map<int, cv::Point2f>::iterator it;
      it = prev_id_pts.find(ids[i]);
      if (it != prev_id_pts.end()) {
        double v_x = (pts[i].x - it->second.x) / dt;
        double v_y = (pts[i].y - it->second.y) / dt;
        pts_velocity.emplace_back(v_x, v_y);
      } else
        pts_velocity.emplace_back(0, 0);
    }
  } else {
    for (unsigned i = 0; i < cur_pts_.size(); i++) {
      pts_velocity.emplace_back(0, 0);
    }
  }
  return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts,
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap) {
  // int rows = imLeft.rows;
  int cols = imLeft.cols;
  if (!imRight.empty() && stereo_cam_)
    cv::hconcat(imLeft, imRight, im_track_);
  else
    im_track_ = imLeft.clone();
  cv::cvtColor(im_track_, im_track_, cv::COLOR_GRAY2RGB);

  for (size_t j = 0; j < curLeftPts.size(); j++) {
    double len = std::min(1.0, 1.0 * track_cnt_[j] / 20);
    cv::circle(im_track_, curLeftPts[j], 2,
               cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
  }
  if (!imRight.empty() && stereo_cam_) {
    for (size_t i = 0; i < curRightPts.size(); i++) {
      cv::Point2f rightPt = curRightPts[i];
      rightPt.x += cols;
      cv::circle(im_track_, rightPt, 2, cv::Scalar(0, 255, 0), 2);
      // cv::Point2f leftPt = curLeftPtsTrackRight[i];
      // cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
    }
  }

  map<int, cv::Point2f>::iterator mapIt;
  for (size_t i = 0; i < curLeftIds.size(); i++) {
    int id = curLeftIds[i];
    mapIt = prevLeftPtsMap.find(id);
    if (mapIt != prevLeftPtsMap.end()) {
      cv::arrowedLine(im_track_, curLeftPts[i], mapIt->second,
                      cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
    }
  }

  // draw prediction
  /*
  for(size_t i = 0; i < predict_pts_debug.size(); i++)
  {
      cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255),
  2);
  }
  */
  // printf("predict pts size %d \n", (int)predict_pts_debug.size());

  // cv::Mat imCur2Compress;
  // cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));
}

void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts) {
  has_prediction_ = true;
  predict_pts_.clear();
  predict_pts_debug_.clear();
  map<int, Eigen::Vector3d>::iterator itPredict;
  for (size_t i = 0; i < ids_.size(); i++) {
    // printf("prevLeftId size %d prevLeftPts size
    // %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
    int id = ids_[i];
    itPredict = predictPts.find(id);
    if (itPredict != predictPts.end()) {
      Eigen::Vector2d tmp_uv;
      m_camera_[0]->spaceToPlane(itPredict->second, tmp_uv);
      predict_pts_.emplace_back(tmp_uv.x(), tmp_uv.y());
      predict_pts_debug_.emplace_back(tmp_uv.x(), tmp_uv.y());
    } else
      predict_pts_.push_back(prev_pts_[i]);
  }
}

void FeatureTracker::removeOutliers(set<int> &removePtsIds) {
  std::set<int>::iterator itSet;
  vector<uchar> status;
  for (size_t i = 0; i < ids_.size(); i++) {
    itSet = removePtsIds.find(ids_[i]);
    if (itSet != removePtsIds.end())
      status.push_back(0);
    else
      status.push_back(1);
  }

  reduceVector(prev_pts_, status);
  reduceVector(ids_, status);
  reduceVector(track_cnt_, status);
}

cv::Mat FeatureTracker::getTrackImage() { return im_track_; }

}  // namespace vins::estimator