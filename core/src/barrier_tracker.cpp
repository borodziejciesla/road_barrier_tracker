#include "barrier_tracker.hpp"

namespace se {
  BarrierTracker::BarrierTracker(const Calibrations & calibrations)
    : calibrations_{calibrations} {}

  void BarrierTracker::Run(const RadarScan & radar_scan) {
    //
  }

  const Barriers & BarrierTracker::GetBarriers(void) const {
    return output_barriers_;
  }
} //  namespace se
