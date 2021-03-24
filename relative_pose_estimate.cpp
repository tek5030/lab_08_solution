#include "relative_pose_estimate.h"

#include "opencv2/core/eigen.hpp"

RelativePoseEstimate::RelativePoseEstimate()
    : R_{}
    , t_{}
    , inliers_{}
    , num_passed_{0}
{

}

RelativePoseEstimate::RelativePoseEstimate(const cv::Matx33d& R, const cv::Vec3d& t, const FrameToFrameCorrespondences& inliers, int num_passed)
    : R_{R}
    , t_{t}
    , inliers_{inliers}
    , num_passed_{num_passed}
{
  Eigen::Matrix3d R_eig;
  cv::cv2eigen(R_, R_eig);

  Eigen::Vector3d t_eig;
  cv::cv2eigen(t_, t_eig);

  pose_ = Sophus::SE3d{R_eig, t_eig};
}

bool RelativePoseEstimate::isFound() const
{
  return num_passed_ > 0;
}

const Sophus::SE3d& RelativePoseEstimate::pose() const
{
  return pose_;
}

const cv::Matx33d& RelativePoseEstimate::R() const
{
  return R_;
}

const cv::Vec3d& RelativePoseEstimate::t() const
{
  return t_;
}

const FrameToFrameCorrespondences& RelativePoseEstimate::inliers() const
{
  return inliers_;
}

int RelativePoseEstimate::numPassed() const
{
  return num_passed_;
}
