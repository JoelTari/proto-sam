#ifndef KEY_META_POSITION_H_
#define KEY_META_POSITION_H_

#include "core/meta.h"

namespace __MetaKeyPosition
{
  inline static constexpr const char position[] = "position";
  inline static constexpr const char x[]        = "x";
  inline static constexpr const char y[]        = "y";
  using MetaKeyPosition_t                = KeyMeta<position, 2, x, y>;
}   // namespace __MetaKeyPosition
using MetaKeyPosition_t                = __MetaKeyPosition::MetaKeyPosition_t;
#endif

