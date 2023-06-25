#include "barrier_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace se {
  BarrierTracker::BarrierTracker(const Calibrations & calibrations)
    : calibrations_{calibrations} {
    // Set transition Matrix
    transition_matrix_ = Eigen::Matrix<float, state_size, state_size>::Identity();
    transition_matrix_(state_size - 1u, state_size - 1u) = transition_matrix_(state_size, state_size) = 1.0f - calibrations_.lambda;
    transition_matrix_(state_size - 1u, state_size) = transition_matrix_(state_size, state_size - 1u) = calibrations_.lambda;
  }

  void BarrierTracker::Run(const RadarScan & radar_scan) {
    // Convert detections
    ConvertDetectionsToCartesianPoints(radar_scan);
    // Prediction Step
    MakePrediction();
    // Association
    MakeGating();
    MakeAssociation();
    // Correction Step
    MakeCorrectionStep();
    // Convert to output structure
    ConvertBarriersToOutput();
  }

  const Barriers & BarrierTracker::GetBarriers(void) const {
    return output_barriers_;
  }

  void BarrierTracker::ConvertDetectionsToCartesianPoints(const RadarScan & radar_scan) {
    // Clear old points
    points_cartesian_.clear();

    // Convert detections
    std::transform(radar_scan.detections.begin(), radar_scan.detections.end(),
      std::back_inserter(points_cartesian_),
      [](const RadarDetection & radar_detection) -> CartesianPoint {
        CartesianPoint cartesian_point;

        const auto c = std::cos(radar_detection.azimuth);
        const auto s = std::sin(radar_detection.azimuth);

        // Pose
        cartesian_point.point(0u) = c * radar_detection.range;
        cartesian_point.point(1u) = s * radar_detection.range;

        // Covariance
        cartesian_point.covariance(0u, 0u) = std::pow(c * radar_detection.range_std, 2) + std::pow(s * radar_detection.range * radar_detection.azimuth_std, 2);
        cartesian_point.covariance(0u, 1u) = cartesian_point.covariance(1u, 0u) = c * s * (std::pow(radar_detection.range_std, 2) + std::pow(radar_detection.range * radar_detection.azimuth_std, 2));
        cartesian_point.covariance(1u, 1u) = std::pow(s * radar_detection.range_std, 2) + std::pow(c * radar_detection.range * radar_detection.azimuth_std, 2);

        return cartesian_point;
      }
    );
  }

  void BarrierTracker::SetObservationMatrix(const float x) {
    observation_matrix_(0u, 0u) = 1.0f;
    for (auto index = 1u; index < state_size -2u; index++)
      observation_matrix_(0u, index) = std::pow(x, index);
  }

  void BarrierTracker::MakePrediction(void) {
    std::transform(barriers_.begin(), barriers_.end(), 
      barriers_.begin(), 
      [this](const InternalBarrier barrier) {
        InternalBarrier predicted;

        predicted.state = transition_matrix_ * barrier.state;
        predicted.covariance = transition_matrix_ * barrier.covariance * transition_matrix_.transpose(); // TODO: Add Q

        return predicted;
      }
    );
  }

  void BarrierTracker::MakeGating(void) {
    // Clear gates
    gates_.clear();
    // Find detections in gates
    std::vector<uint8_t> barrier_gate;
    for (const auto barrier : barriers_) {
      for (auto det_index = 0; det_index < points_cartesian_.size(); det_index++) {
        const auto x = points_cartesian_.at(det_index).point(0u);
        const auto y = points_cartesian_.at(det_index).point(1u);
        const auto cov_y = points_cartesian_.at(det_index).covariance(1u, 1u);

        if (((x - barrier.state(5u)) > -calibrations_.delta_start) && ((barrier.state(6u) - x) > -calibrations_.delta_end)) {
          // In range gate - calculate distance
          const auto innovation = y - barrier.state(1u);
          SetObservationMatrix(x);
          const auto s = observation_matrix_ * barrier.covariance.block<4u, 4u>(0u, 0u) * observation_matrix_.transpose() + cov_y;
          // Calculate distance
          const auto distance = std::pow(innovation, 2u) / s;
          // Check if in gate
          if (distance <= calibrations_.gate)
            barrier_gate.push_back(det_index);
        }
      }
      // Set detections in gate for barrier
      gates_.push_back(barrier_gate);
    }
  }

  void BarrierTracker::MakeAssociation(void) {
    //
  }

  void BarrierTracker::MakeCorrectionStep(void) {
    for (auto barrier_index = 0u; barrier_index < barriers_.size(); barrier_index++ ) {
      for (const auto detection_index : associations_.at(barrier_index)) {
        // Find observation matrix
        SetObservationMatrix(points_cartesian_.at(detection_index).point(0u));
        // Make step
        const auto innovation = points_cartesian_.at(detection_index).point(1u) - observation_matrix_ * barriers_.at(barrier_index).state.head<4u>();
        const auto innovation_covariance = observation_matrix_ * barriers_.at(barrier_index).covariance.block<4u, 4u>(0u, 0u) * observation_matrix_.transpose() + points_cartesian_.at(detection_index).covariance(1u, 1u);
        const auto kalman_gain = barriers_.at(barrier_index).covariance.block<4u, 4u>(0u, 0u) * observation_matrix_.transpose() / innovation_covariance;
        // Set estimation
        barriers_.at(barrier_index).state.head<4u>() += kalman_gain * innovation;
        barriers_.at(barrier_index).covariance.block<4u, 4u>(0u, 0u) = (Eigen::Matrix4f::Identity() - kalman_gain * observation_matrix_) * barriers_.at(barrier_index).covariance.block<4u, 4u>(0u, 0u);
      }
    }
  }

  void BarrierTracker::ConvertBarriersToOutput(void) {
    // TODO: Set timestamo
    // Convert barriers
    output_barriers_.barriers.clear();
    std::transform(barriers_.begin(), barriers_.end(),
      std::back_inserter(output_barriers_.barriers),
      [this](const BarrierTracker::InternalBarrier barrier) {
        return ConvertBarrierToOutput(barrier);
      }
    );
  }

  Barrier BarrierTracker::ConvertBarrierToOutput(const BarrierTracker::InternalBarrier barrier) {
    Barrier output_barrier;

    output_barrier.a0 = barrier.state(0u);
    output_barrier.a0_std = std::sqrt(barrier.covariance(0u, 0u));
    output_barrier.a1 = barrier.state(1u);
    output_barrier.a1_std = std::sqrt(barrier.covariance(1u, 1u));
    output_barrier.a2 = barrier.state(2u);
    output_barrier.a2_std = std::sqrt(barrier.covariance(2u, 2u));
    output_barrier.a3 = barrier.state(3u);
    output_barrier.a3_std = std::sqrt(barrier.covariance(3u, 3u));

    return output_barrier;
  }
} //  namespace se
