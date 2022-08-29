
#include "core/sam-system.h"
#include "factor_impl/anchor.hpp"
#include "factor_impl/linear-translation.hpp"
#include "factor_impl/anchorSE2.hpp"
#include "factor_impl/pose-matcher-SE2.hpp"
#include "factor_impl/cartesian-landmark-obs-SE2.hpp"
#include "factor_impl/motion-model-SE2.hpp"

#include <gtest/gtest.h>
#include "test_utils.h"

TEST(FailSystem, EmptySystem){

  // lets declare a system with lots of factor types
  using namespace ::sam::Factor;
  auto sys = ::sam::System::SamSystem<Anchor2d,LinearTranslation2d,AnchorSE2,PoseMatcherSE2,MotionModelSE2,LandmarkCartesianObsSE2>("my_robot");

  // system is declared, but is empty. it should run, not produce anything
  sys.sam_optimise();

  // TODO: test if one type has factor(s), but another factor type has no factor

  // TODO: jsonify
  
}
