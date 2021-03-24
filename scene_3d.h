#pragma once

#include "map.h"
#include "opencv2/viz.hpp"
#include "pose_estimator.h"

/// \brief 3D visualizer.
class Scene3D
{
public:
  explicit Scene3D(const cv::Matx33d& K);

  void setInitialMap(const Map& map);

  void addMap(const Map& map);

  void updateTrackingFrame(const PoseEstimate& estimate);

  void reset();

  void update();

private:
  cv::Matx33d K_;
  cv::viz::Viz3d vis_3d_;
  int last_keyframe_id_ = 0;
};
