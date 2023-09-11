/* 
 * Copyright 2023 AKKA Technologies and LAAS-CNRS (joel.tari@akka.eu) 
 * 
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by 
 * the European Commission - subsequent versions of the EUPL (the "Licence"); 
 * You may not use this work except in compliance with the Licence. 
 * You may obtain a copy of the Licence at: 
 * 
 * https://joinup.ec.europa.eu/software/page/eupl 
 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the Licence is distributed on an "AS IS" basis, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the Licence for the specific language governing permissions and 
 * limitations under the Licence. 
 */
 
#pragma once

#include <Eigen/Dense>
#include "meta/meta_interface.h"

namespace details_sam::Meta::Measure
{
  namespace Motion2dImpl{
    inline static constexpr const char linearTranslation_str[] = "linear-translation 2d";
    inline static constexpr const char dx[]                = "dx";
    inline static constexpr const char dy[]                = "dy";

    using Motion2d_t = Eigen::Vector<double, 2>;
    using namespace ::sam::Meta;

    namespace exports{

      struct Motion2d
          : ::sam::Meta::Measure::Base<
                          Motion2d,
                          Motion2d_t,
                          linearTranslation_str,
                          dx,
                          dy>
      {
        static constexpr std::size_t compute_kM_impl(){ return Motion2d_t::RowsAtCompileTime; }
        // method where the component name is given in static
        template <const char* COMPONENT>
        static double get_component_impl(const Motion2d_t& measure)
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
                                         const Motion2d_t& measure);
        // {
        //   if (std::string_view(component) == dx)
        //     return measure(0, 0);
        //   else if (std::string_view(component) == dy)
        //     return measure(1, 0);
        //   else
        //     throw std::runtime_error("component requested doesnt exist in linear translation meta");
        // }
      };
      using Motion2d_t = typename Motion2d::type;
    } // exports
  } // Impl
} // details_sam

namespace sam::Meta::Measure
{
  using namespace details_sam::Meta::Measure::Motion2dImpl::exports;
}
namespace sam::Measure
{
  using Motion2d_t = typename sam::Meta::Measure::Motion2d_t;
}
