#pragma once

#include "sophus/se3.hpp"

// Forward declaration.
struct LinearizedPoseMeasurement;

/// \brief Represents a camera projection of a world point, given in normalized image coordinates.
class PoseMeasurement
{
public:
  /// \brief Constructs the measurement.
  /// \param normalized_plane_point Measured image point as normalized image coordinate.
  /// \param world_point Corresponding world point in world coordinate system.
  PoseMeasurement(const Eigen::Vector2d& normalized_plane_point, const Eigen::Vector3d& world_point);

  /// \brief Linearized the measurement prediction function at the current state.
  /// \param current_state Current estimate of the camera pose T_w_c.
  LinearizedPoseMeasurement linearize(const Sophus::SE3d& current_state) const;

  constexpr static int measure_dim = 2; /// Dimension of image measurements (normalized image coordinate)
  constexpr static int state_dim = 6;   /// Dimension of state (pose)

private:
  Eigen::Vector3d world_point_;
  Eigen::Vector2d normalized_plane_point_;
};

/// \brief Struct for linearization of measurement prediction function.
struct LinearizedPoseMeasurement
{
  /// Measurement Jacobian
  Eigen::Matrix<double, PoseMeasurement::measure_dim, PoseMeasurement::state_dim> A;

  /// Measurement error
  Eigen::Matrix<double, PoseMeasurement::measure_dim, 1> b;
};
