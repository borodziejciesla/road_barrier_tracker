#ifndef CORE_INCLUDE_CALIBRATIONS_HPP_
#define CORE_INCLUDE_CALIBRATIONS_HPP_

namespace se {
  struct Calibrations {
    float lambda = 0.9f;      //  [-] Barrier shrinking factor
    float delta_start = 2.0;  //  [m] Maximum delta of start point for gating
    float delta_end = 2.0;    //  [m] Maximum delta of end point for gating
    float gate = 2.7055;      //  [-] Association gate
  };
} //  namespace se

#endif  //  CORE_INCLUDE_CALIBRATIONS_HPP_
