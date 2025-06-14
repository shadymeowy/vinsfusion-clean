/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <vins_estimator/utility/utility.h>

#include <eigen3/Eigen/Dense>

namespace vins::estimator {

class PoseManifold : public ceres::Manifold {
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const;
  virtual bool PlusJacobian(const double *x, double *jacobian) const;
  virtual bool Minus(const double *x, const double *delta,
                     double *x_plus_delta) const {
    // raise an error, not implemented
    throw std::runtime_error(
        "Minus operation is not implemented for PoseManifold.");
  }
  virtual bool MinusJacobian(const double *x, double *jacobian) const {
    throw std::runtime_error(
        "PlusMinus operation is not implemented for PoseManifold.");
  }
  virtual int AmbientSize() const { return 7; };
  virtual int TangentSize() const { return 6; };
};

}  // namespace vins::estimator
