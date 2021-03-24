#pragma once

#include "opencv2/core.hpp"

class FrameToFrameCorrespondences
{
public:
  FrameToFrameCorrespondences() = default;

  FrameToFrameCorrespondences(const std::vector<cv::Point2f>& points_1,
                       const std::vector<cv::Point2f>& points_2,
                       const std::vector<size_t>& point_index_1,
                       const std::vector<size_t>& point_index_2);

  FrameToFrameCorrespondences(std::vector<cv::Point2f>&& points_1,
                       std::vector<cv::Point2f>&& points_2,
                       std::vector<size_t>&& point_index_1,
                       std::vector<size_t>&& point_index_2);

  const std::vector<cv::Point2f>& points_1() const;
  const std::vector<cv::Point2f>& points_2() const;
  const std::vector<size_t>& point_index_1() const;
  const std::vector<size_t>& point_index_2() const;

  size_t size() const;

private:
  std::vector<cv::Point2f> points_1_;
  std::vector<cv::Point2f> points_2_;
  std::vector<size_t> point_index_1_;
  std::vector<size_t> point_index_2_;
};

class MapToFrameCorrespondences
{
public:
  MapToFrameCorrespondences() = default;

  MapToFrameCorrespondences(const std::vector<cv::Point3f>& map_points,
                            const std::vector<cv::Point2f>& frame_points,
                            const std::vector<size_t>& map_point_indices,
                            const std::vector<size_t>& frame_point_indices);

  MapToFrameCorrespondences(std::vector<cv::Point3f>&& map_points,
                            std::vector<cv::Point2f>&& frame_points,
                            std::vector<size_t>&& map_point_indices,
                            std::vector<size_t>&& frame_point_indices);

  const std::vector<cv::Point3f>& map_points() const;
  const std::vector<cv::Point2f>& frame_points() const;
  const std::vector<size_t>& map_point_indices() const;
  const std::vector<size_t>& frame_point_indices() const;

  size_t size() const;

private:
  std::vector<cv::Point3f> map_points_;
  std::vector<cv::Point2f> frame_points_;
  std::vector<size_t> map_point_indices_;
  std::vector<size_t> frame_point_indices_;
};
