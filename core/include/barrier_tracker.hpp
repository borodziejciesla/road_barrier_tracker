#ifndef CORE_INCLUDE_BARRIER_TRACKER_HPP_
#define CORE_INCLUDE_BARRIER_TRACKER_HPP_

#include <vector>

#include <eigen3/Eigen/Dense>

#include "barriers.hpp"
#include "calibrations.hpp"
#include "radar_scan.hpp"

namespace se {
  class BarrierTracker {
    public:
      explicit BarrierTracker(const Calibrations & calibrations);
      ~BarrierTracker(void) = default;

      void Run(const RadarScan & radar_scan);
      const Barriers & GetBarriers(void) const;

    private:
      static constexpr uint8_t state_size = 6u;

      struct CartesianPoint {
        Eigen::Vector2f point;
        Eigen::Matrix2f covariance;
      };

      struct InternalBarrier {
        Eigen::Vector<float, state_size> state;
        Eigen::Matrix<float, state_size, state_size> covariance;
      };

      void ConvertDetectionsToCartesianPoints(const RadarScan & radar_scan);
      void SetObservationMatrix(const float x);
      void MakePrediction(void);
      void MakeGating(void);
      void ConvertBarriersToOutput(void);
      Barrier ConvertBarrierToOutput(const InternalBarrier barrier);

      Barriers output_barriers_;
      Calibrations calibrations_;
      std::vector<CartesianPoint> points_cartesian_;
      Eigen::Matrix<float, state_size, state_size> transition_matrix_;
      Eigen::Matrix<float, 1u, state_size - 2u> observation_matrix_;
      std::vector<std::vector<uint8_t>> gates_;
      std::vector<InternalBarrier> barriers_;
      std::vector<InternalBarrier> predicted_barriers_;
  };
} //  namespace se

#endif  //  CORE_INCLUDE_BARRIER_TRACKER_HPP_
