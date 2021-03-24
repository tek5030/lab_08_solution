cmake_minimum_required( VERSION 3.5.1 )

project(lab_8_vo)

# Compiler flags
if (MSVC)
  set(compiler_options /W4)
  add_definitions(/D_USE_MATH_DEFINES)
else ()
  set(compiler_options -Wall -Wextra -Wpedantic)
endif (MSVC)

# Add libraries.
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.3.4 REQUIRED)
find_package(Sophus REQUIRED)

add_executable(${PROJECT_NAME}
  lab_8.h
  lab_8.cpp
  pose_measurement.h
  pose_measurement.cpp
  calibrated_camera.h
  calibrated_opencv_camera.h
  calibrated_opencv_camera.cpp
  correspondences.h
  correspondences.cpp
  feature_utils.h
  feature_utils.cpp
  frame.h
  frame.cpp
  intrinsic_calibration.h
  intrinsic_calibration.cpp
  main.cpp
  matcher.h
  matcher.cpp
  map.h
  map.cpp
  moba_pose_estimator.h
  moba_pose_estimator.cpp
  pose_estimator.h
  point_measurement.h
  point_measurement.cpp
  pnp_pose_estimator.h
  pnp_pose_estimator.cpp
  two_view_relative_pose_estimator.h
  two_view_relative_pose_estimator.cpp
  relative_pose_estimate.h
  relative_pose_estimate.cpp
  scene_3d.h
  scene_3d.cpp
  points_estimator.h
  dlt_points_estimator.h
  dlt_points_estimator.cpp
  soba_points_estimator.h
  soba_points_estimator.cpp)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Sophus::Sophus
  )

target_compile_options(${PROJECT_NAME} PRIVATE ${compiler_options})
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 14
  )
