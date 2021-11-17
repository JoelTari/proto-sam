#ifndef MEASURE_META_ABSOLUTE_POSITION_H_
#define MEASURE_META_ABSOLUTE_POSITION_H_

#include "meta.h"


// meta absolute position measure
namespace __MetaMeasureAbsolutePosition
{
  // namespace is necessary so that the vars names (x) doesnt pollute global
  // scope
  static constexpr const char absolute_position[] = "absolute_position";
  static constexpr const char x[]                 = "x";
  static constexpr const char y[]                 = "y";
  using MetaMeasureAbsolutePosition_t = MeasureMeta<absolute_position, 2, x, y>;
}   // namespace __MetaMeasureAbsolutePosition
using MetaMeasureAbsolutePosition_t = __MetaMeasureAbsolutePosition::
    MetaMeasureAbsolutePosition_t;   // unwind namespace

#endif
