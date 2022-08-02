#ifndef MEASURE_META_RIGID_BODY_MOTION_SE2_H
#define MEASURE_META_RIGID_BODY_MOTION_SE2_H

#include "core/meta.h"
#include <eigen3/Eigen/Dense>
#include <manif/manif.h>

// Rigid Body Motion holds a measure expressed in SEn (here SE2)

namespace details_sam::Meta::Measure{
  namespace RigidBodyMotionSE2Impl{
    inline static constexpr const char rigid_body_motion_label[] = "rigid body motion SE2";
    inline static constexpr const char x[] = "x";
    inline static constexpr const char y[] = "y";
    inline static constexpr const char t[] = "theta";
    
    // the measure type
    using RigidBodyMotionSE2_t = manif::SE2d;
    using namespace ::sam::Meta::Measure;

    namespace exports{
      struct RigidBodyMotionSE2 : Base<RigidBodyMotionSE2, RigidBodyMotionSE2_t, rigid_body_motion_label, vx,vy,vt>
      {
        static constexpr std::size_t compute_kM_impl(){ return RigidBodyMotionSE2_t::RowsAtCompileTime; }

        template <const char* COMPONENT>
        static auto get_component_impl(const RigidBodyMotionSE2_t& element)
        {
          if constexpr (std::string_view(COMPONENT) == x)
            return element.x();
          else
          {
            if constexpr (std::string_view(COMPONENT) == y)
              return element.y();
            else
            {
                static_assert(std::string_view(COMPONENT) == t);
                return element.angle();
            }
          }
        }

        static double get_component_impl(const char*          component,
                                         const RigidBodyMotionSE2_t& element)
        {
          if (std::string_view(component) == vx)
            return element.x();
          else if (std::string_view(component) == vy)
            return element.y();
          else if (std::string_view(component) == vt)
            return element.angle();
          else
            throw std::runtime_error("component requested doesnt exist in key position meta");
        }
      };
    }
  }
}

namespace sam::Meta::Measure{
  using namespace details_sam::Meta::Measure::RigidBodyMotionSE2Impl::exports;
}
namespace sam::Measure{
  using RigidBodyMotionSE2 = typename sam::Meta::Measure::RigidBodyMotionSE2::type;
}


#endif

