#include <iostream>
#include "correspondences.h"


FrameToFrameCorrespondences::FrameToFrameCorrespondences(const std::vector<cv::Point2f>& points_1,
                                           const std::vector<cv::Point2f>& points_2,
                                           const std::vector<size_t>& point_index_1,
                                           const std::vector<size_t>& point_index_2)
    : points_1_{points_1}
    , points_2_{points_2}
    , point_index_1_{point_index_1}
    , point_index_2_{point_index_2}
{
  if (points_1_.size() != points_2_.size()
      && points_1_.size() != point_index_1_.size()
      && point_index_1_.size() != point_index_2_.size())
  {
    throw std::runtime_error{"Corresponding points must have same size"};
  }
}

FrameToFrameCorrespondences::FrameToFrameCorrespondences(std::vector<cv::Point2f>&& frame_1,
                                           std::vector<cv::Point2f>&& frame_2,
                                           std::vector<size_t>&& point_index_1,
                                           std::vector<size_t>&& point_index_2)
    : points_1_{std::move(frame_1)}
    , points_2_{std::move(frame_2)}
    , point_index_1_{std::move(point_index_1)}
    , point_index_2_{std::move(point_index_2)}
{
  if (points_1_.size() != points_2_.size()
      && points_1_.size() != point_index_1_.size()
      && point_index_1_.size() != point_index_2_.size())
  {
    throw std::runtime_error{"Corresponding points must have same size"};
  }
}

size_t FrameToFrameCorrespondences::size() const
{
  return points_1_.size();
}

const std::vector<cv::Point2f>& FrameToFrameCorrespondences::points_1() const
{
  return points_1_;
}

const std::vector<cv::Point2f>& FrameToFrameCorrespondences::points_2() const
{
  return points_2_;
}

const std::vector<size_t>& FrameToFrameCorrespondences::point_index_1() const
{
  return point_index_1_;
}

const std::vector<size_t>& FrameToFrameCorrespondences::point_index_2() const
{
  return point_index_2_;
}


MapToFrameCorrespondences::MapToFrameCorrespondences(const std::vector<cv::Point3f>& map_points,
                                                     const std::vector<cv::Point2f>& frame_points,
                                                     const std::vector<size_t>& map_point_indices,
                                                     const std::vector<size_t>& frame_point_indices)
    : map_points_{map_points}
    , frame_points_{frame_points}
    , map_point_indices_{map_point_indices}
    , frame_point_indices_{frame_point_indices}
{
  if (map_points_.size() != frame_points_.size()
      && map_points_.size() != map_point_indices_.size()
      && map_point_indices_.size() != frame_point_indices_.size())
  {
    throw std::runtime_error{"Corresponding points must have same size"};
  }
}

MapToFrameCorrespondences::MapToFrameCorrespondences(std::vector<cv::Point3f>&& map_points,
                                                     std::vector<cv::Point2f>&& frame_points,
                                                     std::vector<size_t>&& map_point_indices,
                                                     std::vector<size_t>&& frame_point_indices)
    : map_points_{std::move(map_points)}
    , frame_points_{std::move(frame_points)}
    , map_point_indices_{std::move(map_point_indices)}
    , frame_point_indices_{std::move(frame_point_indices)}
{
  if (map_points_.size() != frame_points_.size()
      && map_points_.size() != map_point_indices_.size()
      && map_point_indices_.size() != frame_point_indices_.size())
  {
    throw std::runtime_error{"Corresponding points must have same size"};
  }
}

const std::vector<cv::Point3f>& MapToFrameCorrespondences::map_points() const
{
  return map_points_;
}

const std::vector<cv::Point2f>& MapToFrameCorrespondences::frame_points() const
{
  return frame_points_;
}

const std::vector<size_t>& MapToFrameCorrespondences::map_point_indices() const
{
  return map_point_indices_;
}

const std::vector<size_t>& MapToFrameCorrespondences::frame_point_indices() const
{
  return frame_point_indices_;
}

size_t MapToFrameCorrespondences::size() const
{
  return map_points_.size();
}
