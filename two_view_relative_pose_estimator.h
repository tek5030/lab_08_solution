#pragma once

#include "relative_pose_estimate.h"

/// \brief Estimates the relative pose between to camera frames through epipolar geometry.
class TwoViewRelativePoseEstimator
{
public:
  /// \brief Constructor
  /// \param K The intrinsic camera calibration matrix.
  explicit TwoViewRelativePoseEstimator(const cv::Matx33d& K, double max_epipolar_distance = 2.0);

  /// \brief Estimate the relative pose from 2d-2d correspondences.
  /// \param corr The 2d-2d correspondences between two frames.
  RelativePoseEstimate estimate(const FrameToFrameCorrespondences& corr);

private:
  cv::Matx33d K_;
  double max_epipolar_distance_;
};
