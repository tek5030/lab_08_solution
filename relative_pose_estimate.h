#pragma once

#include <sophus/se3.hpp>
#include "correspondences.h"

class RelativePoseEstimate
{
public:
  RelativePoseEstimate();

  RelativePoseEstimate(const cv::Matx33d& R,
                       const cv::Vec3d& t,
                       const FrameToFrameCorrespondences& inliers,
                       int num_passed);

  bool isFound() const;

  const Sophus::SE3d& pose() const;
  const cv::Matx33d& R() const;
  const cv::Vec3d& t() const;
  const FrameToFrameCorrespondences& inliers() const;
  int numPassed() const;

private:
  cv::Matx33d R_;
  cv::Vec3d t_;
  FrameToFrameCorrespondences inliers_;
  int num_passed_;
  Sophus::SE3d pose_;
};
