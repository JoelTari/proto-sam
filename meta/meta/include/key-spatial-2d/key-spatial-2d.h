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
#include <stdexcept>
#include <string_view>


namespace details_sam::Meta::Key
{
  namespace Spatial2dImpl{

    inline static constexpr const char position_str[] = "position 2d";
    inline static constexpr const char x[]        = "x";
    inline static constexpr const char y[]        = "y";
    using Spatial2d_t                           = Eigen::Vector<double, 2>;
    using namespace ::sam::Meta::Key;

    namespace exports{
      struct Spatial2d : 
        ::sam::Meta::Key::EuclidBase <Spatial2d, position_str, Spatial2d_t,x,y>
      {
        constexpr static std::size_t compute_kN_impl()
        {
          return Spatial2d_t::RowsAtCompileTime;
        }

        template <const char* COMPONENT>
          static auto get_component_impl(const Spatial2d_t& key_spatial_element)
          {
            if constexpr (std::string_view(COMPONENT) == x)
              return key_spatial_element(0, 0);
            else
            {
              static_assert(std::string_view(COMPONENT) == y);
              return key_spatial_element(1, 0);
            }
          }

        // remove
        static double get_component_impl(const char*          component,
            const Spatial2d_t& key_spatial_element)
        {
          if (std::string_view(component) == x)
            return key_spatial_element(0, 0);
          else if (std::string_view(component) == y)
            return key_spatial_element(1, 0);
          else
            throw std::runtime_error("component requested doesnt exist in key position meta");
        }

      };

      using Spatial2d_t = typename Spatial2d::type;
    }

  }
}

// expose the key meta position structure
namespace sam::Meta::Key{
  // for users
  using namespace details_sam::Meta::Key::Spatial2dImpl::exports;
  // exported : Spatial2d, Spatial2d_t (the key_t)
}
namespace sam::Key{
  using Spatial2d_t = typename sam::Meta::Key::Spatial2d_t;
}
