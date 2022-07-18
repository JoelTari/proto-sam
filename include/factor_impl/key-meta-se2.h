#ifndef KEY_META_se2_H_
#define KEY_META_se2_H_

#include "core/meta.h"

// #include <manif/manif.h>

// WARN: this is the meta for the lie algebra, a.k.a. \mathfrak{se}(2) \approx \mathbb R^3  , not
// the manifold \mathcal{SE}(2). The manifold point can be deduced by applying the Exp operator

// NOTE: the Lie algebra of SE(2) is the set of 3*3 skew-symmetric matrices which is represented
// here via almost R^3 (translation is made via the hat and vee operators)

namespace __MetaKeyPose_se2
{
  inline static constexpr const char pose[] = "pose_se2_lie_algebra";
  inline static constexpr const char vx[]   = "vx";
  inline static constexpr const char vy[]   = "vy";
  inline static constexpr const char vt[]   = "vtheta";
  using MetaKeyPose_se2_t                   = KeyMeta<pose, 3, vx, vy, vt>;
}   // namespace __MetaKeyPose_se2
using MetaKeyPose_se2_t = __MetaKeyPose_se2::MetaKeyPose_se2_t;


// URGENT: temporary
namespace __MetaKeyPoseManif_SE2
{
  inline static constexpr const char pose[] = "pose_manif_SE2";
  inline static constexpr const char x[]   = "x";
  inline static constexpr const char y[]   = "y";
  inline static constexpr const char t[]   = "theta";
  using MetaKeyPoseManif_SE2_t = KeyMeta<pose,3,x,y,t>;
}
using MetaKeyPoseManif_SE2_t = __MetaKeyPoseManif_SE2::MetaKeyPoseManif_SE2_t;
#endif
