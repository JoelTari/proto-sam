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
