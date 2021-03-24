#pragma once

#include "correspondences.h"
#include "frame.h"

#include "map.h"
#include "opencv2/features2d.hpp"

class Matcher
{
public:
  explicit Matcher(int norm_type, float max_ratio = 0.8f);

  FrameToFrameCorrespondences matchFrameToFrame(const Frame& frame_1, const Frame& frame_2) const;
  MapToFrameCorrespondences matchMapToFrame(const Map& map, const Frame& frame) const;

private:
  cv::BFMatcher matcher_;
  float max_ratio_;
};
