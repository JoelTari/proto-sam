#pragma once

#include "meta/meta.h"
#include <Eigen/Dense>
#include <manif/manif.h>

namespace details_sam::Meta::Measure::MotionSE2Impl::exports
{
  struct MotionSE2;
}

namespace sam::Meta::Measure{
  using namespace details_sam::Meta::Measure::MotionSE2Impl::exports;
}
