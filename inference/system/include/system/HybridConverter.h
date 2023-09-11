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

#include "system/SystemConverter.hpp"
#include "utils/config.h"
#include "utils/utils.h"
// #include <boost/multi_index_container.hpp>
#include <amd.h>
#include <vector>


namespace sam::Inference::HybridConverter
{
  std::vector<std::pair<std::string, std::string>>
      infer_fillinedges(const std::vector<int>&                              permutation_vector,
                        const typename SystemConverter::DispatchContainer_t& dispatch_container);

  std::vector<int> amd_order_permutation(int              N,
                                         const int* const hessian_outer_indexes,
                                         const int* const hessian_inner_indexes);

}   // namespace sam::Inference::HybridConverter
