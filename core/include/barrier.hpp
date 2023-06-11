#ifndef CORE_INCLUDE_BARRIER_HPP_
#define CORE_INCLUDE_BARRIER_HPP_

namespace se {
  struct Barrier {
    float x_start = 0.0;
    float x_end = 0.0;

    float a0 = 0.0f;
    float a0_std = 0.0f;
    float a1 = 0.0f;
    float a1_std = 0.0f;
    float a2 = 0.0f;
    float a2_std = 0.0f;
    float a3 = 0.0f;
    float a3_std = 0.0f;
  };
} //  namespace se

#endif  //  CORE_INCLUDE_BARRIER_HPP_
