#include "point_measurement.h"

PointMeasurement::PointMeasurement(const Eigen::Vector2d& normalized_plane_point, const Sophus::SE3d& pose_w_c)
    : pose_c_w_{pose_w_c.inverse()}
    , normalized_plane_point_{normalized_plane_point}
{

}

LinearizedPointMeasurement PointMeasurement::linearize(const Eigen::Vector3d& current_state) const
{
  // Transform world point to camera coordinate frame based on current state estimate.
  Eigen::Vector3d x_c_pred = pose_c_w_ * current_state;

  // Predict normalized image coordinate based on current state estimate.
  Eigen::Vector2d x_n_pred = x_c_pred.head<2>() / x_c_pred.z();

  // Construct linearization object.
  LinearizedPointMeasurement linearization;

  // Compute measurement error.
  linearization.b = normalized_plane_point_ - x_n_pred;

  // Compute Jacobian.
  const double d = 1.0 / x_c_pred.z();

  Eigen::Matrix<double,2,3> d_pi;
  d_pi <<   d, 0.0, -d * x_n_pred.x(),
          0.0,   d, -d * x_n_pred.y();

  linearization.A = d_pi * pose_c_w_.so3().matrix();

  return linearization;
}
