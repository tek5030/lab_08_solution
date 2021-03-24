#include "scene_3d.h"

#include "opencv2/core/eigen.hpp"

Scene3D::Scene3D(const cv::Matx33d& K)
    : K_{K}
    , vis_3d_{"3D visualization"}
{
  reset();
}

void Scene3D::setInitialMap(const Map& map)
{
  const auto& pose_1 = map.frame_1->pose_W_C();
  cv::Mat pose_1_cv;
  cv::eigen2cv(pose_1.matrix(), pose_1_cv);
  vis_3d_.showWidget("Camera_1", cv::viz::WCameraPosition(K_, 0.1, cv::viz::Color::white()));
  vis_3d_.setWidgetPose("Camera_1", cv::Affine3d{pose_1_cv});
  vis_3d_.showWidget("Camera_1_axis", cv::viz::WCoordinateSystem(0.1), cv::Affine3d{pose_1_cv});

  const auto& pose_2 = map.frame_2->pose_W_C();
  cv::Mat pose_2_cv;
  cv::eigen2cv(pose_2.matrix(), pose_2_cv);
  vis_3d_.showWidget("Camera_2", cv::viz::WCameraPosition(K_, 0.1, cv::viz::Color::magenta()));
  vis_3d_.setWidgetPose("Camera_2", cv::Affine3d{pose_2_cv});
  vis_3d_.showWidget("Camera_2_axis", cv::viz::WCoordinateSystem(0.1), cv::Affine3d{pose_2_cv});

  const auto& t_1 = pose_1.translation();
  const auto& t_2 = pose_2.translation();
  vis_3d_.showWidget("Camera_2_line", cv::viz::WLine({t_1.x(), t_1.y(), t_1.z()},
                                                {t_2.x(), t_2.y(), t_2.z()},
                                                cv::viz::Color::blue()));

  vis_3d_.showWidget("Camera_2_cloud", cv::viz::WCloud(map.world_points, cv::viz::Color::magenta()));

  last_keyframe_id_ = 2;
}

void Scene3D::updateTrackingFrame(const PoseEstimate& estimate)
{
  if (estimate.isFound())
  {
    // Extract a reference to the camera pose.
    const auto& pose = estimate.pose_W_C;

    // Convert pose to cv::Mat.
    cv::Mat pose_W_C;
    cv::eigen2cv(pose.matrix(), pose_W_C);


    // Visualize the camera in 3D.
    vis_3d_.showWidget("Camera", cv::viz::WCameraPosition(K_, 0.1, cv::viz::Color::green()));
    vis_3d_.setWidgetPose("Camera", cv::Affine3d{pose_W_C});
  }
}

void Scene3D::addMap(const Map& map)
{
  // Add newest keyframe.
  std::string keyframe_id = "Camera_" + std::to_string(++last_keyframe_id_);

  const auto& pose_2 = map.frame_2->pose_W_C();
  cv::Mat pose_2_cv;
  cv::eigen2cv(pose_2.matrix(), pose_2_cv);
  vis_3d_.showWidget(keyframe_id, cv::viz::WCameraPosition(K_, 0.1, cv::viz::Color::yellow()));
  vis_3d_.setWidgetPose(keyframe_id, cv::Affine3d{pose_2_cv});
  vis_3d_.showWidget(keyframe_id + "_axis", cv::viz::WCoordinateSystem(0.1), cv::Affine3d{pose_2_cv});

  const auto& pose_1 = map.frame_1->pose_W_C();
  const auto& t_1 = pose_1.translation();
  const auto& t_2 = pose_2.translation();
  vis_3d_.showWidget(keyframe_id + "_line", cv::viz::WLine({t_1.x(), t_1.y(), t_1.z()},
                                                {t_2.x(), t_2.y(), t_2.z()},
                                                cv::viz::Color::blue()));

  vis_3d_.showWidget(keyframe_id + "_cloud", cv::viz::WCloud(map.world_points));
}

void Scene3D::reset()
{
  vis_3d_.removeAllWidgets();
  last_keyframe_id_ = 0;

  cv::Vec3d t_cam{0.0, 0.0, -5.0};
  vis_3d_.setViewerPose({cv::Matx33d::eye(), t_cam});

  vis_3d_.showWidget("axis", cv::viz::WCoordinateSystem(0.5), cv::Affine3d{});
}

void Scene3D::update()
{
  vis_3d_.spinOnce();
}
