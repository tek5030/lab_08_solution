#include "dlt_points_estimator.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d.hpp"

PointsEstimate DltPointsEstimator::estimate(
    std::shared_ptr<Frame> frame_1,
    std::shared_ptr<Frame> frame_2,
    const FrameToFrameCorrespondences& corr)
{
  using Matrix34d = Eigen::Matrix<double, 3, 4>;

  Matrix34d proj_mat_1 = frame_1->K()*frame_1->pose_W_C().inverse().matrix3x4();
  Matrix34d proj_mat_2 = frame_2->K()*frame_2->pose_W_C().inverse().matrix3x4();

  cv::Mat proj_mat_1_cv;
  cv::Mat proj_mat_2_cv;
  cv::eigen2cv(proj_mat_1, proj_mat_1_cv);
  cv::eigen2cv(proj_mat_2, proj_mat_2_cv);

  cv::Mat X_hom;
  cv::triangulatePoints(proj_mat_1_cv, proj_mat_2_cv, corr.points_1(), corr.points_2(), X_hom);

  PointsEstimate estimate;
  estimate.world_points.reserve(static_cast<size_t>(X_hom.cols));

  for (int i = 0; i < X_hom.cols; ++i)
  {
    float w = X_hom.at<float>(3, i);
    estimate.world_points.emplace_back(X_hom.at<float>(0, i) / w, X_hom.at<float>(1, i) / w, X_hom.at<float>(2, i) / w);
  }

  return estimate;
}
