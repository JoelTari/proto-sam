#ifndef MEASURE_META_LINEAR_TRANSLATION_H_
#define MEASURE_META_LINEAR_TRANSLATION_H_

#include "core/meta.h"

#include <eigen3/Eigen/Dense>

namespace details_sam::Meta::Measure
{
  namespace LinearTranslation2dImpl{
    inline static constexpr const char linearTranslation_str[] = "linear-translation 2d";
    inline static constexpr const char dx[]                = "dx";
    inline static constexpr const char dy[]                = "dy";

    using LinearTranslation2d_t = Eigen::Vector<double, 2>;
    using namespace ::sam::Meta;

    inline namespace exports{

      struct LinearTranslation2d
          : ::sam::Meta::Measure::Base<
                          LinearTranslation2d,
                          LinearTranslation2d_t,
                          linearTranslation_str,
                          dx,
                          dy>
      {
        static constexpr std::size_t compute_kM_impl(){ return LinearTranslation2d_t::RowsAtCompileTime; }
        // method where the component name is given in static
        template <const char* COMPONENT>
        static double get_component_impl(const LinearTranslation2d_t& measure)
        {
          if constexpr (std::string_view(COMPONENT) == dx)
            return measure(0, 0);
          else
          {
            if constexpr (std::string_view(COMPONENT) == dy)
            {
              static_assert(std::string_view(COMPONENT) == dy);
              return measure(1, 0);
            }
          }
        }

        // method where the component name is given dynamic
        static double get_component_impl(const char*                       component,
                                         const LinearTranslation2d_t& measure)
        {
          if (std::string_view(component) == dx)
            return measure(0, 0);
          else if (std::string_view(component) == dy)
            return measure(1, 0);
          else
            throw std::runtime_error("component requested doesnt exist in linear translation meta");
        }
      };
    } // exports
  } // Impl
} // details_sam

namespace sam::Meta::Measure
{
  using namespace details_sam::Meta::Measure::LinearTranslation2dImpl::exports;
}
namespace sam::Measure
{
  using LinearTranslation2d_t = typename sam::Meta::Measure::LinearTranslation2d::type;
}

#endif
