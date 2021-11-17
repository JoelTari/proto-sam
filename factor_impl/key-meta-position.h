#ifndef KEY_META_POSITION_H_
#define KEY_META_POSITION_H_

#include "meta.h"

namespace __MetaKeyPosition
{
  static constexpr const char position[] = "position";
  static constexpr const char x[]        = "x";
  static constexpr const char y[]        = "y";
  using MetaKeyPosition_t                = KeyMeta<position, 2, x, y>;
}   // namespace __MetaKeyPosition
using MetaKeyPosition_t = __MetaKeyPosition::MetaKeyPosition_t;

#endif

