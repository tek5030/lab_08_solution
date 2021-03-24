#pragma once

#include "frame.h"
#include "correspondences.h"
#include "opencv2/core.hpp"
#include <memory>

/// \brief Struct for points estimation results from triangulation
struct PointsEstimate
{
  std::vector<cv::Point3f> world_points; /// 3D world points.
};

/// \brief Interface for point estimators based on triangulation.
class PointsEstimator
{
public:
  virtual ~PointsEstimator() = default;

  /// \brief Estimates world points with triangulation.
  /// \param frame_1 The first frame.
  /// \param frame_2 The second frame.
  /// \param corr Correspondences between these frames.
  /// \return The results.
  virtual PointsEstimate estimate(std::shared_ptr<Frame> frame_1,
                                  std::shared_ptr<Frame> frame_2,
                                  const FrameToFrameCorrespondences& corr) = 0;

  /// \brief Shared pointer to PointsEstimator, for convenience.
  using Ptr = std::shared_ptr<PointsEstimator>;
};
