#ifndef CORE_INCLUDE_BARRIERS_HPP_
#define CORE_INCLUDE_BARRIERS_HPP_

#include <vector>

#include "barrier.hpp"

namespace se {
  struct Barriers {
    double timestamp = 0.0;
    std::vector<Barrier> barriers;
  };
} //  namespace se

#endif  //  CORE_INCLUDE_BARRIERS_HPP_
