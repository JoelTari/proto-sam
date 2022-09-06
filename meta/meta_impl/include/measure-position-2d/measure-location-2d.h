#pragma once

#include <Eigen/Dense>
#include "meta/meta.h"

namespace details_sam::Meta::Measure
{
  namespace Spatial2dImpl{
    inline static constexpr const char _position_str[] = "_position";
    inline static constexpr const char x[]                 = "x";
    inline static constexpr const char y[]                 = "y";
    // the measure type
    using Spatial2d_t = Eigen::Vector<double, 2>;
    using namespace ::sam::Meta::Measure;

    namespace exports{

      struct Spatial2d : Base<Spatial2d, Spatial2d_t, _position_str, x, y>
      {
        template <const char* COMPONENT>
        static double get_component_impl(const Spatial2d_t & measure)
        {
          if constexpr (std::string_view(COMPONENT) == x)
            return measure(0,0);
          else
          {
            if constexpr (std::string_view(COMPONENT) == y)
            {
              static_assert(std::string_view(COMPONENT) == y);
              return measure(1,0);
            }
          }
        }

        static constexpr std::size_t compute_kM_impl(){ return Spatial2d_t::RowsAtCompileTime; }

        // method where the component name is given dynamic
        static double get_component_impl(const char*                       component,
                                         const Spatial2d_t & measure);
        // {
        //   if (std::string_view(component) == x)
        //     return measure(0, 0);
        //   else if (std::string_view(component) == y)
        //     return measure(1, 0);
        //   else
        //     throw std::runtime_error("component requested doesnt exist in measure  position meta");
        // }
      };
      using Spatial2d_t = typename Spatial2d::type;

    } // end exports
  }
}

namespace sam::Meta::Measure{
  using namespace details_sam::Meta::Measure::Spatial2dImpl::exports;
}
namespace sam::Measure{
  using Spatial2d_t = typename sam::Meta::Measure::Spatial2d_t;
}
