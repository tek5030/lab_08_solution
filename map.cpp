#include "map.h"
#include "Eigen/Dense"

Map::Map(std::shared_ptr<Frame> frame_1_in,
         std::shared_ptr<Frame> frame_2_in,
         const std::vector<cv::Point3f>& world_points_in,
         cv::Mat world_descriptors)
    : frame_1{frame_1_in}
    , frame_2{frame_2_in}
    , world_points{world_points_in}
    , descriptors{world_descriptors}
{

}

std::shared_ptr<Map> createMap(
    std::shared_ptr<Frame> frame_1,
    std::shared_ptr<Frame> frame_2,
    const FrameToFrameCorrespondences& corr,
    std::vector<cv::Point3f> world_points)
{
  cv::Mat frame_2_descriptors = frame_2->descriptors();
  cv::Mat world_descriptors;

  for (const auto& index : corr.point_index_2())
  {
    world_descriptors.push_back(frame_2_descriptors.row(static_cast<int>(index)));
  }

  return std::make_shared<Map>(frame_1, frame_2, world_points, world_descriptors);
}