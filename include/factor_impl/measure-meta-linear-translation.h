#ifndef MEASURE_META_LINEAR_TRANSLATION_H_
#define MEASURE_META_LINEAR_TRANSLATION_H_

#include "core/meta.h"

// meta linear translation measure
namespace __MetaLinearTranslation
{
  inline static constexpr const std::string_view test {"sadfa"};
  inline static constexpr const char linearTranslation[] = "linear-translation";
  inline static constexpr const char dx[]                = "dx";
  inline static constexpr const char dy[]                = "dy";
  using MetaMeasureLinearTranslation_t
      = MeasureMeta<linearTranslation, 2, dx, dy>;
}   // namespace __MetaLinearTranslation
using MetaMeasureLinearTranslation_t
    = __MetaLinearTranslation::MetaMeasureLinearTranslation_t;


#endif
