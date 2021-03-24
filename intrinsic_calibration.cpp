#include "intrinsic_calibration.h"

#include "opencv2/core/eigen.hpp"

IntrinsicCalibration::IntrinsicCalibration(cv::Matx33d K, cv::Vec5d dist_coeffs)
    : K_cv_{K}
    , dist_coeffs_{dist_coeffs}
{
  cv::cv2eigen(K_cv_, K_);
}


const cv::Matx33d IntrinsicCalibration::K_cv() const
{
  return K_cv_;
}


const Eigen::Matrix3d IntrinsicCalibration::K() const
{
  return K_;
}

const cv::Vec5d IntrinsicCalibration::distCoeffs() const
{
  return dist_coeffs_;
}
