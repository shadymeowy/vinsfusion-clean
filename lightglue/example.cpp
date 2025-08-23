#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "matcher.h"

// Detect Shi–Tomasi keypoints on the ORIGINAL images
static std::vector<cv::Point2f> good_keypoints(const cv::Mat &image,
                                               int num = 256,
                                               float min_dist = 40.0) {
  cv::Mat gray;
  if (image.channels() == 1)
    gray = image;
  else
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(gray, corners, num, 0.001, min_dist);
  return corners;
}

// Visualization identical to the original Python behavior
static void visualize(const cv::Mat &image1, const cv::Mat &image2,
                      const std::vector<cv::Point2f> &pts1,
                      const std::vector<cv::Point2f> &pts2,
                      const std::vector<int> &matches) {
  cv::Mat gray1, gray2;
  if (image1.channels() == 1)
    gray1 = image1;
  else
    cv::cvtColor(image1, gray1, cv::COLOR_BGR2GRAY);
  if (image2.channels() == 1)
    gray2 = image2;
  else
    cv::cvtColor(image2, gray2, cv::COLOR_BGR2GRAY);

  cv::Mat concat;
  cv::hconcat(gray1, gray2, concat);
  cv::cvtColor(concat, concat, cv::COLOR_GRAY2BGR);

  const size_t N = std::min({pts1.size(), pts2.size(), matches.size()});

  // 1) draw all keypoints in gray
  for (size_t i = 0; i < N; ++i) {
    int x1 = static_cast<int>(std::lround(pts1[i].x));
    int y1 = static_cast<int>(std::lround(pts1[i].y));
    int x2 = static_cast<int>(std::lround(pts2[i].x));
    int y2 = static_cast<int>(std::lround(pts2[i].y));
    cv::circle(concat, {x1, y1}, 3, cv::Scalar(128, 128, 128), -1);
    cv::circle(concat, {x2 + image1.cols, y2}, 3, cv::Scalar(128, 128, 128),
               -1);
  }

  // 2) matched pairs in HSV-derived colors (no lines)
  for (size_t i = 0; i < N; ++i) {
    int j = matches[i];
    if (j == -1 || static_cast<size_t>(j) >= pts2.size()) continue;

    int x1 = static_cast<int>(std::lround(pts1[i].x));
    int y1 = static_cast<int>(std::lround(pts1[i].y));
    int x2 = static_cast<int>(std::lround(pts2[j].x));
    int y2 = static_cast<int>(std::lround(pts2[j].y));

    int hue = (x1 + y1) % 180;
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    cv::Vec3b c = bgr.at<cv::Vec3b>(0, 0);
    cv::Scalar color_bgr(c[0], c[1], c[2]);

    cv::circle(concat, {x1, y1}, 3, color_bgr, -1);
    cv::circle(concat, {x2 + image1.cols, y2}, 3, color_bgr, -1);
  }

  cv::imshow("Matches", concat);
  cv::imwrite("assets/matches.png", concat);
  cv::waitKey(0);
  cv::destroyAllWindows();
}

int main() {
  try {
    const std::string path_image1 = "assets/1.png";
    const std::string path_image2 = "assets/2.png";
    const std::string model_path = "/datasets/superpoint_lightglue_end2end.onnx";

    // Network input geometry
    const int W = 240, H = 512, Nmax1 = 256, Nmax2 = 2048;

    // Load ORIGINAL images (any size); matcher will preprocess internally
    cv::Mat img1 = cv::imread(path_image1, cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread(path_image2, cv::IMREAD_COLOR);
    if (img1.empty() || img2.empty())
      throw std::runtime_error("Failed to load input images.");

    // Detect keypoints on ORIGINAL images
    std::vector<cv::Point2f> kpts1 = good_keypoints(img1, Nmax1, 40.0);
    std::vector<cv::Point2f> kpts2 = good_keypoints(img2, Nmax2, 10.0);

    // Run matcher (internally preprocesses + scales keypoints)
    Matcher matcher(model_path, W, H, Nmax1, Nmax2);
    std::vector<int> id1_to_2;
    matcher.match(img1, img2, kpts1, kpts2, id1_to_2);

    std::cout << "Matches size: " << id1_to_2.size() << std::endl;

    // Visualize in ORIGINAL coordinates (no rescale needed since we draw by
    // index)
    visualize(img1, img2, kpts1, kpts2, id1_to_2);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }
}
