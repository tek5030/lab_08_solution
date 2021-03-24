#pragma once

#include "frame.h"
#include "correspondences.h"
#include "points_estimator.h"
#include "opencv2/core.hpp"

struct Map
{
  Map(std::shared_ptr<Frame> frame_1,
      std::shared_ptr<Frame> frame_2,
      const std::vector<cv::Point3f>& world_points,
      cv::Mat world_descriptors);

  std::shared_ptr<Frame> frame_1;
  std::shared_ptr<Frame> frame_2;
  std::vector<cv::Point3f> world_points;
  cv::Mat descriptors;
};

std::shared_ptr<Map> createMap(std::shared_ptr<Frame> frame_1,
    std::shared_ptr<Frame> frame_2,
    const FrameToFrameCorrespondences& corr,
    std::vector<cv::Point3f> world_points);
