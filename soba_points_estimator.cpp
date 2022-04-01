#include "soba_points_estimator.h"
#include "Eigen/Sparse"
#include "Eigen/SparseQR"
#include <iostream>

SobaPointsEstimator::SobaPointsEstimator(PointsEstimator::Ptr initial_points_estimator)
    : initial_points_estimator_{initial_points_estimator}
{

}

PointsEstimate SobaPointsEstimator::estimate(
    std::shared_ptr<Frame> frame_1,
    std::shared_ptr<Frame> frame_2,
    const FrameToFrameCorrespondences& corr)
{
  // Get initial points estimate.
  PointsEstimate init_estimate = initial_points_estimator_->estimate(frame_1, frame_2, corr);

  // Create measurement set for each camera.
  std::vector<std::vector<PointMeasurement>> measurements(2);

  const size_t num_measurements = corr.size();

  // Add first frame.
  measurements[0].reserve(num_measurements);
  for (size_t j=0; j < num_measurements; ++j)
  {
    // Add measurement for frame 1.
    measurements[0].emplace_back(frame_1->toNormalized({corr.points_1()[j].x, corr.points_1()[j].y}),
                                 frame_1->pose_W_C());
  }

  // Add second frame.
  measurements[1].reserve(num_measurements);
  for (size_t j=0; j < num_measurements; ++j)
  {
    // Add corresponding measurement for frame 2.
    measurements[1].emplace_back(frame_2->toNormalized({corr.points_2()[j].x, corr.points_2()[j].y}),
                                 frame_2->pose_W_C());
  }

  init_estimate.world_points = optimize(measurements, init_estimate.world_points);

  return init_estimate;
}

std::vector<cv::Point3f> SobaPointsEstimator::optimize(const std::vector<std::vector<PointMeasurement>>& measurements,
                                                       const std::vector<cv::Point3f>& initial_points)
{
  const size_t max_iterations = 3;
  constexpr int measure_dim = PointMeasurement::measure_dim;
  constexpr int state_dim = PointMeasurement::state_dim;

  const size_t num_cameras = measurements.size();
  const size_t num_points = measurements[0].size();

  Eigen::VectorXd b(measure_dim * num_cameras * num_points);

  Eigen::VectorXd current_state(state_dim * num_points);
  for (size_t i=0; i < num_points; ++i)
  {
    current_state.segment<state_dim>(state_dim*i) = Eigen::Vector3d{initial_points[i].x, initial_points[i].y, initial_points[i].z};
  }

  // Sparse solver.
  Eigen::LeastSquaresConjugateGradient <Eigen::SparseMatrix<double>> solver;
  std::vector<Eigen::Triplet<double>> A_triplets;
  A_triplets.reserve(measure_dim * state_dim * num_cameras * num_points);

  // Comment when done!
  std::cout << "SOBA---------" << std::endl;

  size_t iteration = 0;
  double curr_cost = 0.0f;
  while (iteration < max_iterations)
  {
    // Linearize.
    // Build A and b from each measurement.
    Eigen::SparseMatrix<double> A(measure_dim * num_cameras * num_points, state_dim * num_points);
    A_triplets.clear();
    for (size_t i=0; i < num_cameras; ++i)
    {
      for (size_t j=0; j < num_points; ++j)
      {
        const LinearizedPointMeasurement linearization = measurements[i][j].linearize(current_state.segment<state_dim>(state_dim*j));

        const size_t block_row = num_points*measure_dim*i + measure_dim*j;
        const size_t block_col = state_dim*j;

        // Fill triplets.
        // Corresponds to A.block(num_points*measure_dim*i + measure_dim*j, state_dim*j, measure_dim, state_dim) = linearization.A;
        for (size_t r=0; r < measure_dim; ++r)
        {
          for (size_t c=0; c < state_dim; ++c)
          {
            A_triplets.emplace_back(block_row + r, block_col + c, linearization.A(r,c));
          }
        }

        b.segment(num_points*measure_dim*i + measure_dim*j, measure_dim) = linearization.b;
      }
    }

    // Compute current cost.
    curr_cost = b.squaredNorm();

    // Remove when done!
    std::cout << "Cost before update: " << curr_cost << std::endl;

    // Solve linearized system.
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());
    solver.compute(A);
    Eigen::VectorXd update = solver.solve(b);

    // Update state.
    current_state += update;

    ++iteration;
  }


  std::vector<cv::Point3f> estimated_points;
  for (size_t i=0; i < num_points; ++i)
  {
    estimated_points.emplace_back(
        current_state(i*state_dim),
        current_state(i*state_dim+1),
        current_state(i*state_dim+2));
  }


  return estimated_points;
}
