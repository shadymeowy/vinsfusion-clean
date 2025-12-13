#include "anms.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

void AdaptiveNMS::run(const std::vector<float> &xs,
                      const std::vector<float> &ys,
                      const std::vector<float> &response,
                      std::vector<bool> &result, int height, int width,
                      int num_ret_points, int max_tolerance,
                      int max_iterations) {
  double eps_var = 0.25;

  int low = 1;
  int high = height;
  int radius = low + ((high - low) / 2);
  int prevradius = -1;
  int result_count = 0;

  // if keypoints are less than the required number
  if (xs.size() <= static_cast<std::size_t>(num_ret_points)) {
    result.clear();
    result.resize(xs.size(), true);
    return;
  }

  assert(xs.size() == ys.size() && xs.size() == response.size());

  // Sort keypoints by response
  indices_.resize(xs.size());
  x_sorted_.clear();
  y_sorted_.clear();
  x_sorted_.reserve(xs.size());
  y_sorted_.reserve(ys.size());

  std::iota(indices_.begin(), indices_.end(), 0);
  std::sort(indices_.begin(), indices_.end(),
            [&response](std::size_t a, std::size_t b) {
              return response[a] > response[b];
            });

  for (std::size_t i = 0; i < indices_.size(); ++i) {
    x_sorted_.push_back(xs[indices_[i]]);
    y_sorted_.push_back(ys[indices_[i]]);
  }

  unsigned int K = num_ret_points;
  unsigned int Kmax = K + max_tolerance;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // binary search step
    radius = low + ((high - low) / 2);
    // needed to reassure the same radius is not repeated again
    if (radius == prevradius || low > high) {
      // return the keypoints from the previous iteration
      break;
    }

    // reset result
    result.clear();
    result.resize(xs.size(), false);
    result_count = 0;

    // initializing grid
    double c = eps_var * radius / sqrt(2);
    double thresh = static_cast<double>(radius) / c;
    double thresh2 = thresh * thresh;
    int tmp = static_cast<int>(radius / c);

    int num_cell_rows = static_cast<int>(height / c);
    int num_cell_cols = static_cast<int>(width / c);

    covered_.clear();
    covered_.resize((num_cell_rows + 1) * (num_cell_cols + 1), false);

    for (unsigned int i = 0; i < xs.size(); ++i) {
      int row = static_cast<int>(y_sorted_[i] / c);
      int col = static_cast<int>(x_sorted_[i] / c);
      row = std::clamp(row, 0, num_cell_rows);
      col = std::clamp(col, 0, num_cell_cols);

      if (covered_[(row * (num_cell_cols + 1)) + col]) {
        continue;
      }

      // if the cell is not covered
      result[indices_[i]] = true;
      result_count++;

      // get range which current radius is covering
      int row_min = std::max(0, row - tmp);
      int row_max = std::min(num_cell_rows, row + tmp);
      int col_min = std::max(0, col - tmp);
      int col_max = std::min(num_cell_cols, col + tmp);

      for (int row_to_cov = row_min; row_to_cov <= row_max; ++row_to_cov) {
        for (int col_to_cov = col_min; col_to_cov <= col_max; ++col_to_cov) {
          double dist = (((row_to_cov - row) * (row_to_cov - row)) +
                         ((col_to_cov - col) * (col_to_cov - col)));

          if (dist <= thresh2) {
            // check the distance to every cell
            covered_[(row_to_cov * (num_cell_cols + 1)) + col_to_cov] = true;
          }
        }
      }
    }

    // solution found
    if (result_count >= K && result_count <= Kmax) {
      break;
    }
    if (result_count < K) {
      // update binary search range
      high = radius - 1;
    } else {
      low = radius + 1;
    }

    prevradius = radius;
  }

  // If target number is reached, return
  if (result_count == K) {
    return;
  }

  // If not enough keypoints found
  // loop over the remaining keypoints and add them
  if (result_count < K) {
    for (std::size_t i = 0; i < indices_.size(); ++i) {
      if (!result[indices_[i]]) {
        result[indices_[i]] = true;
        result_count++;
      }
      if (result_count == K) {
        break;
      }
    }
  }

  // If too many keypoints found
  // remove the weakest keypoints
  if (result_count > K) {
    for (int i = static_cast<int>(indices_.size()) - 1; i >= 0; --i) {
      if (result[indices_[i]]) {
        result[indices_[i]] = false;
        result_count--;
        if (result_count == K) {
          break;
        }
      }
    }
  }

  // Now the result contains the required number of keypoints
}