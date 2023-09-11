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
 
#include "key-spatial-2d/key-spatial-2d.h"


// using namespace ::sam::Meta::Key;
//
// constexpr std::size_t Spatial2d::compute_kN_impl()
// {
//   return Spatial2d_t::RowsAtCompileTime;
// }
//
// // remove
// double Spatial2d::get_component_impl(const char*          component,
//     const Spatial2d_t& key_spatial_element)
// {
//   if (std::string_view(component) == x)
//     return key_spatial_element(0, 0);
//   else if (std::string_view(component) == y)
//     return key_spatial_element(1, 0);
//   else
//     throw std::runtime_error("component requested doesnt exist in key position meta");
// }
