#include <vins_estimator/featureTracker/klt_tracker.h>

#include <cstdio>
#include <vector>

namespace vins::klt {

bool in_border(const cv::Point2f &pt, int width, int height) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < width - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < height - BORDER_SIZE;
}

double distance(const cv::Point2f &pt1, const cv::Point2f &pt2) {
  // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < static_cast<int>(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void reduceVector(std::vector<int> &v, std::vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < static_cast<int>(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void set_mask(cv::Mat &mask, std::vector<cv::Point2f> &pts,
              std::vector<int> &cnt, std::vector<int> &ids, int min_dist) {
  // prefer to keep features that are tracked for long time
  std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned i = 0; i < pts.size(); i++)
    cnt_pts_id.emplace_back(cnt[i], std::make_pair(pts[i], ids[i]));

  sort(cnt_pts_id.begin(), cnt_pts_id.end(),
       [](const std::pair<int, std::pair<cv::Point2f, int>> &a,
          const std::pair<int, std::pair<cv::Point2f, int>> &b) {
         return a.first > b.first;
       });

  pts.clear();
  ids.clear();
  cnt.clear();

  for (auto &it : cnt_pts_id) {
    if (mask.at<uchar>(it.second.first) == 255) {
      pts.push_back(it.second.first);
      ids.push_back(it.second.second);
      cnt.push_back(it.first);
      cv::circle(mask, it.second.first, min_dist, 0, -1);
    }
  }
}

int track_klt(const unsigned char *img, int width, int height, float *x_out,
              float *y_out, int *ids_out, int *cnt_out, int min_dist,
              int max_cnt, bool flow_back) {
  // static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  // clahe->apply(_img, _img);

  static cv::Mat prev_img;
  static std::vector<cv::Point2f> prev_pts;
  static std::vector<int> ids;
  static std::vector<int> track_cnt;
  static int n_id = 0;

  cv::Mat cur_img =
      cv::Mat(height, width, CV_8UC1, const_cast<unsigned char *>(img));
  std::vector<cv::Point2f> cur_pts;

  if (prev_pts.size() > 0) {
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err,
                             cv::Size(21, 21), 3);

    // reverse check
    if (flow_back) {
      std::vector<uchar> reverse_status;
      std::vector<cv::Point2f> reverse_pts = prev_pts;
      cv::calcOpticalFlowPyrLK(
          cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err,
          cv::Size(21, 21), 1,
          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                           0.01),
          cv::OPTFLOW_USE_INITIAL_FLOW);

      for (size_t i = 0; i < status.size(); i++) {
        if (status[i] && reverse_status[i] &&
            distance(prev_pts[i], reverse_pts[i]) <= 0.5) {
          status[i] = 1;
        } else
          status[i] = 0;
      }
    }

    for (int i = 0; i < static_cast<int>(cur_pts.size()); i++)
      if (status[i] && !in_border(cur_pts[i], width, height))
        status[i] = 0;
    reduceVector(cur_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
  }

  for (auto &n : track_cnt)
    n++;

  {
    std::printf("set mask begins");
    cv::Mat mask = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
    set_mask(mask, cur_pts, track_cnt, ids, min_dist);
    std::printf("set mask costs");

    std::printf("detect feature begins");
    int n_max_cnt = max_cnt - static_cast<int>(cur_pts.size());
    if (n_max_cnt > 0) {
      if (mask.empty())
        std::cout << "mask is empty " << std::endl;
      if (mask.type() != CV_8UC1)
        std::cout << "mask type wrong " << std::endl;

      std::vector<cv::Point2f> n_pts;
      n_pts.reserve(n_max_cnt);
      cv::goodFeaturesToTrack(cur_img, n_pts, n_max_cnt, 0.01, min_dist, mask);
      for (auto &p : n_pts) {
        cur_pts.push_back(p);
        ids.push_back(n_id++);
        track_cnt.push_back(1);
      }
    }

    std::printf("detect feature costs");
    // printf("feature cnt after add %d\n", (int)ids.size());
  }

  prev_img = cur_img.clone();
  prev_pts = cur_pts;

  for (size_t i = 0; i < cur_pts.size(); i++) {
    x_out[i] = cur_pts[i].x;
    y_out[i] = cur_pts[i].y;
    ids_out[i] = ids[i];
    cnt_out[i] = track_cnt[i];
  }

  return static_cast<int>(cur_pts.size());
}

} // namespace vins::klt