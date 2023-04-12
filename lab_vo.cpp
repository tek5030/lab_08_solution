#include "lab_vo.h"

#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "two_view_relative_pose_estimator.h"
#include "pnp_pose_estimator.h"
#include "scene_3d.h"
#include "moba_pose_estimator.h"
#include "dlt_points_estimator.h"
#include "soba_points_estimator.h"
#include <chrono>
#include <iomanip>
#include <utility>

// Make shorthand aliases for timing tools.
using Clock = std::chrono::high_resolution_clock;
using DurationInMs = std::chrono::duration<double, std::milli>;

LabVO::LabVO(std::shared_ptr<CalibratedCamera> cam)
    : cam_{std::move(cam)}
    , detector_{cv::xfeatures2d::SURF::create(100, 4, 3, true)}
    , desc_extractor_{detector_}
    , matcher_{desc_extractor_->defaultNorm()}
{ }

void LabVO::run()
{
  // Create the 2d-2d relative pose estimator.
  // We will use this for map creation.
  TwoViewRelativePoseEstimator frame_to_frame_pose_estimator{cam_->calibration().K_cv(), 0.5};

  // Create the 3d-2d pose estimator.
  // We will use this to navigate in maps between keyframes.
  MobaPoseEstimator pose_estimator(std::make_shared<PnPPoseEstimator>(cam_->calibration().K()), cam_->calibration().K().block<2,1>(0,2), {cam_->calibration().K()(0,0), cam_->calibration().K()(1,1)});

  // Create points estimator.
  // We will use this estimator to triangulate points.
  PointsEstimator::Ptr init_points_estimator = std::make_shared<DltPointsEstimator>();
  PointsEstimator::Ptr points_estimator = std::make_shared<SobaPointsEstimator>(init_points_estimator);

  // Create 3d scene visualization.
  Scene3D scene_3d(cam_->calibration().K_cv());

  // Construct empty pointers to hold frames and maps.
  std::shared_ptr<Frame> active_keyframe;
  std::shared_ptr<Frame> tracking_frame;
  std::shared_ptr<Map> active_map;
  std::shared_ptr<Map> init_map;

  for (;;)
  {
    auto start = Clock::now();

    // Captures and makes the frame ready for matching.
    tracking_frame = captureFrame();

    // Construct image for visualization.
    cv::Mat vis_img = tracking_frame->image();
    if (vis_img.channels() < 3)
    {
      cv::cvtColor(vis_img, vis_img, cv::COLOR_GRAY2BGR);
    }

    // If we have an active map, track the frame using 3d-2d correspondences.
    if (active_map)
    {
      // Compute 3d-2d correspondences.
      const auto corr = matcher_.matchMapToFrame(*active_map, *tracking_frame);

      // Estimate pose from 3d-2d correspondences.
      PoseEstimate estimate = pose_estimator.estimate(corr.frame_points(), corr.map_points());

      if (estimate.isFound())
      {
        // Update frame pose with 3d-2d estimate.
        tracking_frame->setPose(estimate.pose_W_C);

        // Visualize tracking.
        scene_3d.updateTrackingFrame(estimate);
        for (const auto& point : estimate.image_inlier_points)
        {
          cv::drawMarker(vis_img, point, {0, 255, 0}, cv::MARKER_CROSS, 5);
        }
      }
    }

    // If we only have one active keyframe and no map,
    // visualize the map initialization from 2d-2d correspondences.
    else if (active_keyframe)
    {
      // Compute 2d-2d correspondences.
      auto corr = matcher_.matchFrameToFrame(*active_keyframe, *tracking_frame);

      // Estimate pose from 2d-2d correspondences.
      const auto estimate = frame_to_frame_pose_estimator.estimate(corr);

      if (estimate.isFound())
      {
        // Update frame poses with 2d-2d estimate (first camera is origin).
        active_keyframe->setPose(Sophus::SE3d{});
        tracking_frame->setPose(estimate.pose());

        // Compute an initial 3d map from the correspondences by using the epipolar geometry.
        PointsEstimate init_estimate = points_estimator->estimate(active_keyframe, tracking_frame, estimate.inliers());
        init_map = createMap(active_keyframe, tracking_frame, estimate.inliers(), init_estimate.world_points);

        if (init_map)
        {
          // Visualize initial 3d map with relative poses.
          scene_3d.setInitialMap(*init_map);

          const auto& pt1 = estimate.inliers().points_1();
          const auto& pt2 = estimate.inliers().points_2();
          for (size_t i = 0; i < estimate.inliers().size(); ++i)
          {
            cv::line(vis_img, pt1[i], pt2[i], {255, 0, 0});
            cv::drawMarker(vis_img, pt1[i], {255, 255, 255}, cv::MARKER_CROSS, 5);
            cv::drawMarker(vis_img, pt2[i], {255, 0, 255}, cv::MARKER_CROSS, 5);
          }
        }
      }
    }

    // Stop the clock and print the processing time in the frame.
    auto end = Clock::now();
    DurationInMs duration = end - start;
    std::stringstream corr_duration_txt;
    corr_duration_txt << std::fixed << std::setprecision(0);
    corr_duration_txt << "Processing: " << duration.count() << "ms";
    cv::putText(vis_img, corr_duration_txt.str(), {10, 20}, cv::FONT_HERSHEY_PLAIN, 1.0, {0, 0, 255});
    cv::imshow("Test", vis_img);

    // Get input from keyboard.
    const auto key = cv::waitKey(10);
    if (key == ' ')
    {
      if (!active_keyframe)
      {
        // Set active keyframe.
        active_keyframe = tracking_frame;
      }
      else if (!active_map)
      {
        // Set active map.
        active_map = init_map;
        active_keyframe = active_map->frame_2;
      }
      else
      {
        // Add a new consecutive map as an odometry step.
        if (tracking_frame->hasPose())
        {
          // Use 2d-2d pose estimator to extract inliers for map point triangulation.
          const auto estimate = frame_to_frame_pose_estimator.estimate(
              matcher_.matchFrameToFrame(*active_keyframe, *tracking_frame));

          // Try to create a new map based on the 2d-2d inliers.
          PointsEstimate points_estimate = points_estimator->estimate(active_keyframe, tracking_frame, estimate.inliers());
          auto new_map = createMap(active_keyframe, tracking_frame, estimate.inliers(),
                                   points_estimate.world_points);

          if (new_map)
          {
            // Update map.
            active_map = new_map;
            active_keyframe = tracking_frame;
            scene_3d.addMap(*new_map);
          }
        }
      }
    }
    else if (key == 'r')
    {
      // Reset.
      // Make all reference data empty.
      active_keyframe = nullptr;
      active_map = nullptr;
      scene_3d.reset();
    }
    else if (key > 0)
    {
      // Quit.
      break;
    }

    // Update the 3d visualization.
    scene_3d.update();
  }
}

std::shared_ptr<Frame> LabVO::captureFrame()
{
  cv::Mat distorted_image = cam_->captureImage();

  return std::make_shared<Frame>(cam_->calibration(), distorted_image, detector_, desc_extractor_);
}
