#ifndef MEASURE_META_ABSOLUTE_POSITION_H_
#define MEASURE_META_ABSOLUTE_POSITION_H_

#include "core/meta.h"
#include <eigen3/Eigen/Dense>


namespace details_sam::Meta::Measure
{
  namespace AbsolutePosition2dImpl{
    inline static constexpr const char absolute_position_str[] = "absolute_position";
    inline static constexpr const char x[]                 = "x";
    inline static constexpr const char y[]                 = "y";
    // the measure type
    using AbsolutePosition2d_t = Eigen::Vector<double, 2>;
    using namespace ::sam::Meta::Measure;

    inline namespace exports{

      struct AbsolutePosition2d : Base<AbsolutePosition2d, AbsolutePosition2d_t, absolute_position_str, x, y>
      {
        template <const char* COMPONENT>
        static double get_component_impl(const AbsolutePosition2d_t & measure)
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

        static constexpr std::size_t compute_kM_impl(){ return AbsolutePosition2d_t::RowsAtCompileTime; }

        // method where the component name is given dynamic
        static double get_component_impl(const char*                       component,
                                         const AbsolutePosition2d_t & measure)
        {
          if (std::string_view(component) == x)
            return measure(0, 0);
          else if (std::string_view(component) == y)
            return measure(1, 0);
          else
            throw std::runtime_error("component requested doesnt exist in measure absolute position meta");
        }
      };

    } // end exports
  }
}

namespace sam::Meta::Measure{
  using namespace details_sam::Meta::Measure::AbsolutePosition2dImpl::exports;
}
namespace sam::Measure{
  using AbsolutePosition2d_t = typename sam::Meta::Measure::AbsolutePosition2d::type;
}

// // meta absolute position measure
// namespace __MetaMeasureAbsolutePosition
// {
//   // namespace is necessary so that the vars names (x) doesnt pollute global
//   // scope
//   inline static constexpr const char absolute_position[] = "absolute_position";
//   inline static constexpr const char x[]                 = "x";
//   inline static constexpr const char y[]                 = "y";
//   // the measure type
//   using MeasureAbsolutePosition_t = Eigen::Vector<double, 2>;
//
//   struct MetaMeasureAbsolutePosition_t : MeasureMeta<MetaMeasureAbsolutePosition_t, MeasureAbsolutePosition_t, absolute_position, x, y>
//   {
//     template <const char* COMPONENT>
//     static double get_component_impl(const MeasureAbsolutePosition_t & measure)
//     {
//       if constexpr (std::string_view(COMPONENT) == x)
//         return measure(0,0);
//       else
//       {
//         if constexpr (std::string_view(COMPONENT) == y)
//         {
//           static_assert(std::string_view(COMPONENT) == y);
//           return measure(1,0);
//         }
//       }
//     }
//
//     static constexpr std::size_t compute_kM_impl(){ return MeasureAbsolutePosition_t::RowsAtCompileTime; }
//
//     // method where the component name is given dynamic
//     static double get_component_impl(const char*                       component,
//                                      const MeasureAbsolutePosition_t & measure)
//     {
//       if (std::string_view(component) == x)
//         return measure(0, 0);
//       else if (std::string_view(component) == y)
//         return measure(1, 0);
//       else
//         throw std::runtime_error("component requested doesnt exist in measure absolute position meta");
//     }
//   };
//   // using MetaMeasureAbsolutePosition_t
//   //     = MeasureMeta<MeasureAbsolutePosition_t, absolute_position, 2, x, y>;
// }   // namespace MetaMeasureAbsolutePosition
// using MetaMeasureAbsolutePosition_t = __MetaMeasureAbsolutePosition::MetaMeasureAbsolutePosition_t;

#endif
