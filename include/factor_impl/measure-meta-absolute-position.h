#ifndef MEASURE_META_ABSOLUTE_POSITION_H_
#define MEASURE_META_ABSOLUTE_POSITION_H_

#include "core/meta.h"


// meta absolute position measure
namespace MetaMeasureAbsolutePosition
{
  // namespace is necessary so that the vars names (x) doesnt pollute global
  // scope
  inline static constexpr const char absolute_position[] = "absolute_position";
  inline static constexpr const char x[]                 = "x";
  inline static constexpr const char y[]                 = "y";
  using MetaMeasureAbsolutePosition_t = MeasureMeta<absolute_position, 2, x, y>;
}   // namespace
using MetaMeasureAbsolutePosition_t = MetaMeasureAbsolutePosition::MetaMeasureAbsolutePosition_t;

#endif
