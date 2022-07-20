#ifndef MEASURE_META_LINEAR_TRANSLATION_H_
#define MEASURE_META_LINEAR_TRANSLATION_H_

#include "core/meta.h"

#include <eigen3/Eigen/Dense>

// meta linear translation measure
namespace __MetaLinearTranslation
{
  inline static constexpr const char linearTranslation[] = "linear-translation";
  inline static constexpr const char dx[]                = "dx";
  inline static constexpr const char dy[]                = "dy";
  // the measure type
  using MeasureLinearTranslation_t = Eigen::Vector<double, 2>;
  // using MetaMeasureLinearTranslation_t
  //     = MeasureMeta<Eigen::Vector<double,2>,linearTranslation, 2, dx, dy>;
  struct MetaMeasureLinearTranslation_t
      : MeasureMeta<MetaMeasureLinearTranslation_t,
                    MeasureLinearTranslation_t,
                    linearTranslation,
                    2,
                    dx,
                    dy>
  {
    // method where the component name is given in static
    template <const char* COMPONENT>
    static double get_component_impl(const MeasureLinearTranslation_t& measure)
    {
      if constexpr (std::string_view(COMPONENT) == dx)
        return measure(0, 0);
      else
      {
        if constexpr (std::string_view(COMPONENT) == dy)
          return measure(1, 0);
        else
          return -1;   // FIX: create failure here , static throw ?
      }
    }

    // method where the component name is given dynamic
    static double get_component_impl(const char*                       component,
                                     const MeasureLinearTranslation_t& measure)
    {
      if (std::string_view(component) == dx)
        return measure(0, 0);
      else if (std::string_view(component) == dy)
        return measure(1, 0);
      else
        throw std::runtime_error("component requested doesnt exist in linear translation meta");
    }
  };
}   // namespace __MetaLinearTranslation
using MetaMeasureLinearTranslation_t = __MetaLinearTranslation::MetaMeasureLinearTranslation_t;


#endif
