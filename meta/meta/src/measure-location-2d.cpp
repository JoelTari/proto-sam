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
 
#include "measure-location-2d/measure-location-2d.h"

namespace details_sam::Meta::Measure::Location2dImpl::exports
{
    double Location2d::get_component_impl(const char*                       component,
                                     const Location2d_t & measure)
    {
      if (std::string_view(component) == x)
        return measure(0, 0);
      else if (std::string_view(component) == y)
        return measure(1, 0);
      else
        throw std::runtime_error("component requested doesnt exist in measure  position meta");
    }
}
