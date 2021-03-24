#include "matcher.h"
#include "feature_utils.h"

Matcher::Matcher(int norm_type, float max_ratio)
    : matcher_{norm_type}
    , max_ratio_{max_ratio}
{ }


FrameToFrameCorrespondences Matcher::matchFrameToFrame(const Frame& frame_1, const Frame& frame_2) const
{
  std::vector<std::vector<cv::DMatch>> matches;
  matcher_.knnMatch(frame_2.descriptors(), frame_1.descriptors(), matches, 2);
  std::vector<cv::DMatch> good_matches = extractGoodRatioMatches(matches, max_ratio_);

  std::vector<cv::Point2f> points_1;
  std::vector<cv::Point2f> points_2;
  std::vector<size_t> point_index_1;
  std::vector<size_t> point_index_2;
  points_1.reserve(matches.size());
  points_2.reserve(matches.size());
  point_index_1.reserve(matches.size());
  point_index_2.reserve(matches.size());

  const auto& keypts1 = frame_1.keypoints();
  const auto& keypts2 = frame_2.keypoints();

  for (auto& match : good_matches)
  {
    const auto& ind_1 = match.trainIdx;
    points_1.push_back(keypts1[ind_1].pt);
    point_index_1.push_back(static_cast<size_t>(ind_1));

    const auto& ind_2 = match.queryIdx;
    points_2.push_back(keypts2[ind_2].pt);
    point_index_2.push_back(static_cast<size_t>(ind_2));
  }

  return {std::move(points_1), std::move(points_2), std::move(point_index_1), std::move(point_index_2)};
}

MapToFrameCorrespondences Matcher::matchMapToFrame(const Map& map, const Frame& frame) const
{
  std::vector<std::vector<cv::DMatch>> matches;
  matcher_.knnMatch(frame.descriptors(), map.descriptors, matches, 2);
  std::vector<cv::DMatch> good_matches = extractGoodRatioMatches(matches, max_ratio_);

  std::vector<cv::Point3f> map_points;
  std::vector<cv::Point2f> frame_points;
  std::vector<size_t> map_point_indices;
  std::vector<size_t> frame_point_indices;
  map_points.reserve(matches.size());
  frame_points.reserve(matches.size());
  map_point_indices.reserve(matches.size());
  frame_point_indices.reserve(matches.size());

  const auto& keypts = frame.keypoints();

  for (auto& match : good_matches)
  {
    const auto& map_ind = match.trainIdx;
    map_points.push_back(map.world_points[map_ind]);
    map_point_indices.push_back(static_cast<size_t>(map_ind));

    const auto& frame_ind = match.queryIdx;
    frame_points.push_back(keypts[frame_ind].pt);
    frame_point_indices.push_back(static_cast<size_t>(frame_ind));
  }

  return {std::move(map_points), std::move(frame_points), std::move(map_point_indices), std::move(frame_point_indices)};
}


