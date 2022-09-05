#pragma once

#include "meta/meta.h"
#include <Eigen/Dense>
#include <manif/manif.h>

namespace details_sam::Meta::Measure{
  namespace MotionSE2Impl{
    inline static constexpr const char rigid_body_motion_label[] = "rigid body motion SE2";
    inline static constexpr const char x[] = "x";
    inline static constexpr const char y[] = "y";
    inline static constexpr const char t[] = "theta";
    
    // the measure type
    using MotionSE2_t = manif::SE2d;
    using namespace ::sam::Meta::Measure;

    namespace exports{
      struct MotionSE2 : Base<MotionSE2, MotionSE2_t, rigid_body_motion_label, x,y,t>
      {
        static constexpr std::size_t compute_kM_impl(){ return MotionSE2_t::DoF; }

        template <const char* COMPONENT>
        static auto get_component_impl(const MotionSE2_t& element)
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
                                         const MotionSE2_t& element)
        {
          if (std::string_view(component) == x)
            return element.x();
          else if (std::string_view(component) == y)
            return element.y();
          else if (std::string_view(component) == t)
            return element.angle();
          else
            throw std::runtime_error("component requested doesnt exist in key position meta");
        }
      };
      using MotionSE2_t = typename MotionSE2::type;
    }
  }
}


namespace sam::Meta::Measure{
  using namespace details_sam::Meta::Measure::MotionSE2Impl::exports;
}
namespace sam::Measure
{
  using MotionSE2_t = typename sam::Meta::Measure::MotionSE2_t;
}
