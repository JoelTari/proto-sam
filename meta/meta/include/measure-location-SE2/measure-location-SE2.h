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

#include "meta/meta_interface.h"
#include <Eigen/Dense>
#include <manif/manif.h>


namespace details_sam::Meta::Measure{
  namespace LocationSE2Impl{
    inline static constexpr const char _pose_label[] = " pose SE2";
    inline static constexpr const char x[] = "x";
    inline static constexpr const char y[] = "y";
    inline static constexpr const char t[] = "theta";
    
    // the measure type
    using LocationSE2_t = typename manif::SE2d;
    using namespace ::sam::Meta::Measure;

    namespace exports{
      struct LocationSE2 : Base<LocationSE2, LocationSE2_t, _pose_label, x,y,t>
      {
        static constexpr std::size_t compute_kM_impl(){ return manif::SE2d::DoF; }

        template <const char* COMPONENT>
        static auto get_component_impl(const LocationSE2_t& SE2_element)
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
                                         const LocationSE2_t& SE2_element)
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
      using LocationSE2_t = typename LocationSE2::type;
    }
  }
}


namespace sam::Meta::Measure{
  using namespace details_sam::Meta::Measure::LocationSE2Impl::exports;
}
namespace sam::Measure{
  using LocationSE2_t = typename sam::Meta::Measure::LocationSE2_t;
}
