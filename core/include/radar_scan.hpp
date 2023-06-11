#ifndef CORE_INCLUDE_RADAR_SCAN_HPP_
#define CORE_INCLUDE_RADAR_SCAN_HPP_

#include <vector>

#include "radar_detection.hpp"

namespace se {
  struct RadarScan {
    double timestamp = 0.0;
    std::vector<RadarDetection> detections;
  };
} //  namespace se

#endif  //  CORE_INCLUDE_RADAR_SCAN_HPP_
