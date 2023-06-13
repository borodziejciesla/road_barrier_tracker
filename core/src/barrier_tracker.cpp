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
      }
    );
  }
} //  namespace se
