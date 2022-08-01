#ifndef MEASURE_META_ABSOLUTE_POSE_SE2_H_
#define MEASURE_META_ABSOLUTE_POSE_SE2_H_

#include "core/meta.h"
#include <eigen3/Eigen/Dense>
#include <manif/manif.h>


namespace details_sam::Meta::Measure{
  namespace AbsolutePoseSE2Impl{
    inline static constexpr const char absolute_pose_label[] = "absolute pose SE2";
    inline static constexpr const char x[] = "x";
    inline static constexpr const char y[] = "y";
    inline static constexpr const char t[] = "theta";
    
    // the measure type
    using AbsolutePoseSE2_t = typename manif::SE2d;
    using namespace ::sam::Meta::Measure;

    namespace exports{
      struct AbsolutePoseSE2 : Base<AbsolutePoseSE2, AbsolutePoseSE2_t, absolute_pose_label, x,y,t>
      {
        static constexpr std::size_t compute_kM_impl(){ return manif::SE2d::DoF; }

        template <const char* COMPONENT>
        static auto get_component_impl(const AbsolutePoseSE2_t& SE2_element)
        {
          if constexpr (std::string_view(COMPONENT) == x)
            return SE2_element.x();
          else
          {
            if constexpr (std::string_view(COMPONENT) == y)
              return SE2_element.y();
            else
            {
                static_assert(std::string_view(COMPONENT) == y);
                return SE2_element.angle();
            }
          }
        }

        // remove
        static double get_component_impl(const char*          component,
                                         const AbsolutePoseSE2_t& SE2_element)
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
    }
  }
}


namespace sam::Meta::Measure{
  using namespace details_sam::Meta::Measure::AbsolutePoseSE2Impl::exports;
}
namespace sam::Measure{
  using AbsolutePoseSE2 = typename sam::Meta::Measure::AbsolutePoseSE2::type;
}


// namespace __MetaMeasureAbsolutePoseSE2
// {
//   inline static constexpr const char absolute_pose[] = "absolute_pose_SE2";
//   inline static constexpr const char x[] = "x";
//   inline static constexpr const char y[] = "y";
//   inline static constexpr const char t[] = "theta";
//   
//   // the measure type
//   using MeasureAbsolutePoseSE2_t = typename manif::SE2d;
//
//   struct MetaMeasureAbsolutePoseSE2_t : MeasureMeta<MetaMeasureAbsolutePoseSE2_t, MeasureAbsolutePoseSE2_t, absolute_pose, x,y,t>
//   {
//     static constexpr std::size_t compute_kM_impl(){ return manif::SE2d::DoF; }
//
//     template <const char* COMPONENT>
//     static auto get_component_impl(const MeasureAbsolutePoseSE2_t& SE2_element)
//     {
//       if constexpr (std::string_view(COMPONENT) == x)
//         return SE2_element.x();
//       else
//       {
//         if constexpr (std::string_view(COMPONENT) == y)
//           return SE2_element.y();
//         else
//         {
//             static_assert(std::string_view(COMPONENT) == y);
//             return SE2_element.angle();
//         }
//       }
//     }
//
//     // remove
//     static double get_component_impl(const char*          component,
//                                      const MeasureAbsolutePoseSE2_t& SE2_element)
//     {
//       if (std::string_view(component) == x)
//         return SE2_element.x();
//       else if (std::string_view(component) == y)
//         return SE2_element.y();
//       else if (std::string_view(component) == t)
//         return SE2_element.angle();
//       else
//         throw std::runtime_error("component requested doesnt exist in key position meta");
//     }
//   };
// }
// using MetaMeasureAbsolutePoseSE2_t= __MetaMeasureAbsolutePoseSE2::MetaMeasureAbsolutePoseSE2_t;

#endif
