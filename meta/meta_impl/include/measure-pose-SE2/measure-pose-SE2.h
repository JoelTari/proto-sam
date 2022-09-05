#pragma once

#include "meta/meta.h"
#include <Eigen/Dense>
#include <manif/manif.h>


namespace details_sam::Meta::Measure{
  namespace PoseSE2Impl{
    inline static constexpr const char _pose_label[] = " pose SE2";
    inline static constexpr const char x[] = "x";
    inline static constexpr const char y[] = "y";
    inline static constexpr const char t[] = "theta";
    
    // the measure type
    using PoseSE2_t = typename manif::SE2d;
    using namespace ::sam::Meta::Measure;

    namespace exports{
      struct PoseSE2 : Base<PoseSE2, PoseSE2_t, _pose_label, x,y,t>
      {
        static constexpr std::size_t compute_kM_impl(){ return manif::SE2d::DoF; }

        template <const char* COMPONENT>
        static auto get_component_impl(const PoseSE2_t& SE2_element)
        {
          if constexpr (std::string_view(COMPONENT) == x)
            return SE2_element.x();
          else
          {
            if constexpr (std::string_view(COMPONENT) == y)
              return SE2_element.y();
            else
            {
                static_assert(std::string_view(COMPONENT) == t);
                return SE2_element.angle();
            }
          }
        }

        // remove
        static double get_component_impl(const char*          component,
                                         const PoseSE2_t& SE2_element)
        {
          if (std::string_view(component) == x)
            return SE2_element.x();
          else if (std::string_view(component) == y)
            return SE2_element.y();
          else if (std::string_view(component) == t)
            return SE2_element.angle();
          else
            throw std::runtime_error("component requested doesnt exist in key position meta");
        }
      };
      using PoseSE2_t = typename PoseSE2::type;
    }
  }
}


namespace sam::Meta::Measure{
  using namespace details_sam::Meta::Measure::PoseSE2Impl::exports;
}
namespace sam::Measure{
  using PoseSE2_t = typename sam::Meta::Measure::PoseSE2_t;
}
