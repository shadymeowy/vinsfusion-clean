#pragma once

#include <vector>

class AdaptiveNMS {
public:
  AdaptiveNMS() = default;
  ~AdaptiveNMS() = default;

  void run(const std::vector<float> &xs, const std::vector<float> &ys,
           const std::vector<float> &response, std::vector<bool> &result,
           int height, int width, int num_ret_points, int max_tolerance = 10,
           int max_iterations = 32);

private:
  // reuse buffers
  std::vector<std::size_t> indices_;
  std::vector<float> x_sorted_;
  std::vector<float> y_sorted_;
  std::vector<bool> covered_;
};
