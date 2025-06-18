#pragma once

#include <ceres/ceres.h>
#include <vins_estimator/utility/utility.h>

#include <Eigen/Dense>

namespace vins::estimator {

// Reprojection factor for DPVO patches.
// Parameter blocks: pose_i(7), pose_j(7), extrinsic(7), inv_depth(1)
// No td parameter, no velocity/rolling shutter correction.
// Per-instance sqrt_info (not static).
class DpvoProjectionFactor
    : public ceres::SizedCostFunction<2, 7, 7, 7, 1> {
 public:
  DpvoProjectionFactor(const Eigen::Vector3d& _pts_i,
                       const Eigen::Vector3d& _pts_j,
                       const Eigen::Matrix2d& _sqrt_info);

  bool Evaluate(const double* const* parameters, double* residuals,
                double** jacobians) const override;

  Eigen::Vector3d pts_i, pts_j;
  Eigen::Matrix2d sqrt_info_;
};

}  // namespace vins::estimator
