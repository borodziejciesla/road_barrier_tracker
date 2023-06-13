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
      struct CartesianPoint {
        Eigen::Vector2f point;
        Eigen::Matrix2f covariance;
      };

      void ConvertDetectionsToCartesianPoints(const RadarScan & radar_scan);      

      Barriers output_barriers_;
      Calibrations calibrations_;
      std::vector<CartesianPoint> points_cartesian_;
  };
} //  namespace se

#endif  //  CORE_INCLUDE_BARRIER_TRACKER_HPP_
