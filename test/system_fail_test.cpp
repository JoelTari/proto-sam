
#include "anchor2d/anchor2d.h"
#include "anchorSE2/anchorSE2.h"
#include "cartesian-landmark-obs-SE2/cartesian-landmark-obs-SE2.h"
#include "motion-model-SE2/motion-model-SE2.h"
#include "relative-matcher-2d/relative-matcher-2d.h"
#include "relative-matcher-SE2/relative-matcher-SE2.h"
#include "system/sam-system.h"
#include "test_utils.h"

#include <gtest/gtest.h>

TEST(FailSystem, EmptySystem)
{
  // lets declare a system with lots of factor types
  using namespace ::sam::Factor;
  auto sys = ::sam::Inference::System<::sam::Inference::SolverSparseQR,
                                      Anchor2d,
                                      RelativeMatcher2d,
                                      AnchorSE2,
                                      RelativeMatcherSE2,
                                      MotionModelSE2,
                                      LandmarkCartesianObsSE2>("my_robot");

  // system is declared, but is empty. it should run, not produce anything
  sys.sam_optimise();

  // TODO: test if one type has factor(s), but another factor type has no factor

  // TODO: jsonify
}
