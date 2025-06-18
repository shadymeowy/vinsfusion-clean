#include <vins_estimator/factor/dpvo_projection_factor.h>

namespace vins::estimator {

DpvoProjectionFactor::DpvoProjectionFactor(const Eigen::Vector3d& _pts_i,
                                           const Eigen::Vector3d& _pts_j,
                                           const Eigen::Matrix2d& _sqrt_info)
    : pts_i(_pts_i), pts_j(_pts_j), sqrt_info_(_sqrt_info) {}

bool DpvoProjectionFactor::Evaluate(const double* const* parameters,
                                    double* residuals,
                                    double** jacobians) const {
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                        parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
                        parameters[1][5]);

  Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
                         parameters[2][5]);

  double inv_dep_i = parameters[3][0];

  // Guard against degenerate inverse depth (zero or negative causes inf/nan)
  if (std::abs(inv_dep_i) < 1e-3) {
    Eigen::Map<Eigen::Vector2d>(residuals).setZero();
    if (jacobians) {
      if (jacobians[0]) Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]).setZero();
      if (jacobians[1]) Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]).setZero();
      if (jacobians[2]) Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[2]).setZero();
      if (jacobians[3]) Eigen::Map<Eigen::Vector2d>(jacobians[3]).setZero();
    }
    return true;
  }

  Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
  Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
  Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
  Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
  Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

  Eigen::Map<Eigen::Vector2d> residual(residuals);
  double dep_j = pts_camera_j.z();
  if (dep_j < 0.1) {
    residual.setZero();
    if (jacobians) {
      if (jacobians[0]) Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]).setZero();
      if (jacobians[1]) Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]).setZero();
      if (jacobians[2]) Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[2]).setZero();
      if (jacobians[3]) Eigen::Map<Eigen::Vector2d>(jacobians[3]).setZero();
    }
    return true;
  }
  residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
  residual = sqrt_info_ * residual;

  // Catch-all: if residual is not finite or too large, zero out
  constexpr double MAX_RESIDUAL_SQ = 1e4;  // cap residual magnitude
  if (!residual.allFinite() || residual.squaredNorm() > MAX_RESIDUAL_SQ) {
    residual.setZero();
    if (jacobians) {
      if (jacobians[0]) Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]).setZero();
      if (jacobians[1]) Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]).setZero();
      if (jacobians[2]) Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[2]).setZero();
      if (jacobians[3]) Eigen::Map<Eigen::Vector2d>(jacobians[3]).setZero();
    }
    return true;
  }

  if (jacobians) {
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d ric = qic.toRotationMatrix();
    Eigen::Matrix<double, 2, 3> reduce;
    reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
              0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
    reduce = sqrt_info_ * reduce;

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(
          jacobians[0]);
      Eigen::Matrix<double, 3, 6> jaco_i;
      jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
      jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri *
                              -Utility::skewSymmetric(pts_imu_i);
      jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(
          jacobians[1]);
      Eigen::Matrix<double, 3, 6> jaco_j;
      jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
      jaco_j.rightCols<3>() =
          ric.transpose() * Utility::skewSymmetric(pts_imu_j);
      jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
      jacobian_pose_j.rightCols<1>().setZero();
    }

    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
          jacobian_ex_pose(jacobians[2]);
      Eigen::Matrix<double, 3, 6> jaco_ex;
      jaco_ex.leftCols<3>() =
          ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
      Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
      jaco_ex.rightCols<3>() =
          -tmp_r * Utility::skewSymmetric(pts_camera_i) +
          Utility::skewSymmetric(tmp_r * pts_camera_i) +
          Utility::skewSymmetric(ric.transpose() *
                                 (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
      jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
      jacobian_ex_pose.rightCols<1>().setZero();
    }

    if (jacobians[3]) {
      Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
      jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric *
                         pts_i * -1.0 / (inv_dep_i * inv_dep_i);
    }

    // Safety: if any Jacobian is not finite, zero everything
    auto safeJ7 = [](double* j) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> m(j);
      if (!m.allFinite()) m.setZero();
    };
    auto safeJ1 = [](double* j) {
      Eigen::Map<Eigen::Vector2d> m(j);
      if (!m.allFinite()) m.setZero();
    };
    if (jacobians[0]) safeJ7(jacobians[0]);
    if (jacobians[1]) safeJ7(jacobians[1]);
    if (jacobians[2]) safeJ7(jacobians[2]);
    if (jacobians[3]) safeJ1(jacobians[3]);
  }

  return true;
}

}  // namespace vins::estimator
