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
#include <manif/manif.h>


namespace details_sam::Meta::Key
{
  namespace SpatialSE2Impl {
    inline static constexpr const char pose_SE2_label[] = "pose SE2";
    inline static constexpr const char x[]   = "x";
    inline static constexpr const char y[]   = "y";
    inline static constexpr const char t[]   = "theta";
    using SpatialSE2_t = typename manif::SE2d;
    using Tangent_Space_SE2_t = typename manif::SE2Tangentd;

    using namespace ::sam::Meta::Key;

    namespace exports{
      struct SpatialSE2 : ::sam::Meta::Key::Base<SpatialSE2, pose_SE2_label, SpatialSE2_t, Tangent_Space_SE2_t ,x,y,t>
      {
        constexpr static std::size_t compute_kN_impl()
        {
            return manif::SE2Tangentd::DoF;
        }
        
        template<const char* COMPONENT>
        static auto get_component_impl(const SpatialSE2_t & key_SE2)
        {
          if constexpr(std::string_view(COMPONENT) ==x)
            return key_SE2.x();
          else
          {
            if constexpr(std::string_view(COMPONENT)==y)
            {
              return key_SE2.y();
            }
            else
            {
              static_assert(std::string_view(COMPONENT)==t);
              return key_SE2.angle();
              // return key_SE2.rotation();
            }
          }
        }

        static double get_component_impl(const char* component, const SpatialSE2_t & key_SE2)
        {
          if (std::string_view(component) == x)
            return key_SE2.x();
          else if (std::string_view(component) == y)
            return key_SE2.y();
          else if (std::string_view(component) == t)
            return key_SE2.angle();
          else
            throw std::runtime_error("component requested doesnt exist in key position meta");
        }

      };
      using SpatialSE2_t = typename SpatialSE2::type;
    }

  }
}

namespace sam::Meta::Key{
  using namespace details_sam::Meta::Key::SpatialSE2Impl::exports;
}
namespace sam::Key{
  using SpatialSE2_t = typename sam::Meta::Key::SpatialSE2_t;
}
