#pragma once

#include "points_estimator.h"

/// \brief Points estimator based on the DLT triangulation algorithm.
class DltPointsEstimator : public PointsEstimator
{
public:
  DltPointsEstimator() = default;

  /// \brief Triangulates based on the DLT algorithm.
  /// \param frame_1 The first frame.
  /// \param frame_2 The second frame.
  /// \param corr Correspondences between these frames.
  /// \return The results.
  PointsEstimate estimate(
      std::shared_ptr<Frame> frame_1,
      std::shared_ptr<Frame> frame_2,
      const FrameToFrameCorrespondences& corr) override;

};
