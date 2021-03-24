#pragma once

#include "points_estimator.h"
#include "point_measurement.h"

/// \brief Iterative triangulation based on Structure-Only BA.
class SobaPointsEstimator : public PointsEstimator
{
public:
  /// \brief Constructs the points estimator.
  /// \param initial_points_estimator Pointer to a points estimator for initialization.
  SobaPointsEstimator(PointsEstimator::Ptr initial_points_estimator);

  /// \brief Triangulates based on minimizing the reprojection error with the Structure-Only BA.
  /// \param frame_1 The first frame.
  /// \param frame_2 The second frame.
  /// \param corr Correspondences between these frames.
  /// \return The results.
  PointsEstimate estimate(
      std::shared_ptr<Frame> frame_1,
      std::shared_ptr<Frame> frame_2,
      const FrameToFrameCorrespondences& corr) override;

private:
  PointsEstimator::Ptr initial_points_estimator_;

  // Gauss-Newton optimization, minimizing reprojection error.
  std::vector<cv::Point3f> optimize(const std::vector<std::vector<PointMeasurement>>& measurements,
                                    const std::vector<cv::Point3f>& initial_points);
};
