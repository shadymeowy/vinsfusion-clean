#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace vins::klt {

int track_klt(const unsigned char *img, int width, int height, float *x_out, float *y_out,
               int *ids_out, int *cnt_out,
               int min_dist, int max_cnt, bool flow_back);

}
