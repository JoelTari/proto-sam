/* 
 * Copyright 2023 AKKA Technologies and LAAS-CNRS (joel.tari@akka.eu) 
 * 
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by 
 * the European Commission - subsequent versions of the EUPL (the "Licence"); 
 * You may not use this work except in compliance with the Licence. 
 * You may obtain a copy of the Licence at: 
 * 
 * https://joinup.ec.europa.eu/software/page/eupl 
 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the Licence is distributed on an "AS IS" basis, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the Licence for the specific language governing permissions and 
 * limitations under the Licence. 
 */
 

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
  auto sys = ::sam::Inference::SparseSystem<::sam::Inference::SolverSparseQR,
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
