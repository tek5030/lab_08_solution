#pragma once

#include "Eigen/Core"
#include "opencv2/core.hpp"

// Convenient shorthand for distortion vectors.
namespace cv
{
using Vec5d = Vec<double, 5>;
}

class IntrinsicCalibration
{
public:
  IntrinsicCalibration(cv::Matx33d K, cv::Vec5d dist_coeffs);

  const cv::Matx33d K_cv() const;
  const Eigen::Matrix3d K() const;
  const cv::Vec5d distCoeffs() const;

private:
  cv::Matx33d K_cv_;
  cv::Vec5d dist_coeffs_;
  Eigen::Matrix3d K_;
};
