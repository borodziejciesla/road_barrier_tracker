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
     // Associate
    // Correction Step
    // TODO
    // COnvert to output
    // TODO
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
    predicted_barriers_.clear();
    std::transform(barriers_.begin(), barriers_.end(), 
      std::back_inserter(predicted_barriers_), 
      [this](const InternalBarrier barrier) {
        InternalBarrier predicted;

        predicted.state = transition_matrix_ * barrier.state;
        predicted.covariance = transition_matrix_ * barrier.covariance * transition_matrix_.transpose(); // TODO: Add Q

        return predicted;
      }
    );
  }

  void BarrierTracker::MakeGating(void) {
    for (const auto barrier : barriers_) {
      for (auto det_index = 0; det_index < points_cartesian_.size(); det_index++) {
        //
      }
    }
  }
} //  namespace se
