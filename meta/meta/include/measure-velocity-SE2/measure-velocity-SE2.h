#pragma once

#include <Eigen/Dense>
#include <manif/manif.h>

#include "meta/meta_interface.h"

namespace details_sam::Meta::Measure{
  namespace VelocitySE2Impl{
    inline static constexpr const char velocity_label[] = "velocity SE2";
    inline static constexpr const char vx[] = "vx";
    inline static constexpr const char vy[] = "vy";
    inline static constexpr const char vt[] = "vtheta";
    
    // the measure type
    using VelocitySE2_t = manif::SE2Tangentd;
    using namespace ::sam::Meta::Measure;

    namespace exports{
      struct VelocitySE2 : Base<VelocitySE2, VelocitySE2_t, velocity_label, vx,vy,vt>
      {
        static constexpr std::size_t compute_kM_impl(){ return VelocitySE2_t::DoF; }

        template <const char* COMPONENT>
        static auto get_component_impl(const VelocitySE2_t& velocity)
        {
          if constexpr (std::string_view(COMPONENT) == vx)
            return velocity.x();
          else
          {
            if constexpr (std::string_view(COMPONENT) == vy)
              return velocity.y();
            else
            {
                static_assert(std::string_view(COMPONENT) == vt);
                return velocity.angle();
            }
          }
        }

        static double get_component_impl(const char*          component,
                                         const VelocitySE2_t& velocity)
        {
          if (std::string_view(component) == vx)
            return velocity.x();
          else if (std::string_view(component) == vy)
            return velocity.y();
          else if (std::string_view(component) == vt)
            return velocity.angle();
          else
            throw std::runtime_error("component requested doesnt exist in key position meta");
        }
      };
      using VelocitySE2_t = typename VelocitySE2::type;
    }
  }
}

namespace sam::Meta::Measure{
  using namespace details_sam::Meta::Measure::VelocitySE2Impl::exports;
}
namespace sam::Measure{
  using VelocitySE2_t = typename sam::Meta::Measure::VelocitySE2_t;
}


