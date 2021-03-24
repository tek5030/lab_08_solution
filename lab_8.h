#pragma once

#include "calibrated_camera.h"
#include "frame.h"

#include "matcher.h"
#include "opencv2/videoio.hpp"
#include <memory>

class Lab8
{
public:
  explicit Lab8(std::shared_ptr<CalibratedCamera> cam);

  void run();

private:
  std::shared_ptr<Frame> captureFrame();

  std::shared_ptr<CalibratedCamera> cam_;
  cv::Ptr<cv::Feature2D> detector_;
  cv::Ptr<cv::Feature2D> desc_extractor_;
  Matcher matcher_;
};
