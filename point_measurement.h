#pragma once

#include "sophus/se3.hpp"

// Forward declaration.
struct LinearizedPointMeasurement;

/// \brief Represents a camera projection of a world point, given in normalized image coordinates.
class PointMeasurement
{
public:
  /// \brief Constructs the measurement.
  /// \param normalized_plane_point Measured image point as normalized image coordinate.
  /// \param pose_w_c Pose of the camera in the world coordinate system.
  PointMeasurement(const Eigen::Vector2d& normalized_plane_point, const Sophus::SE3d& pose_w_c);

  /// \brief Linearized the measurement prediction function at the current state.
  /// \param current_state Current estimate of the world point.
  LinearizedPointMeasurement linearize(const Eigen::Vector3d& current_state) const;

  enum {
    measure_dim = 2, /// Dimension of image measurements (normalized image coordinate)
    state_dim = 3    /// Dimension of state (3D point)
  };

private:
  Sophus::SE3d pose_c_w_;
  Eigen::Vector2d normalized_plane_point_;
};

/// \brief Struct for linearization of measurement prediction function.
struct LinearizedPointMeasurement
{
  /// Measurement Jacobian
  Eigen::Matrix<double, PointMeasurement::measure_dim, PointMeasurement::state_dim> A;

  /// Measurement error
  Eigen::Matrix<double, PointMeasurement::measure_dim, 1> b;
};
