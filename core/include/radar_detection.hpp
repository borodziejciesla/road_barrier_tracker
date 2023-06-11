#ifndef CORE_INCLUDE_RADAR_DETECTION_HPP_
#define CORE_INCLUDE_RADAR_DETECTION_HPP_

namespace se {
  struct RadarDetection {
    float range = 0.0f;
    float range_std = 0.0f;
    float azimuth = 0.0f;
    float azimuth_std = 0.0f;
    float range_rate = 0.0f;
    float range_rate_std = 0.0f;
  };
} //  namespace se

#endif  //  CORE_INCLUDE_RADAR_DETECTION_HPP_
