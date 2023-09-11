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

#include "system/GraphConverter.hpp"
#include "utils/config.h"
#include "utils/utils.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <vector>


namespace sam::Inference::GraphConverter
{

  UndirectedGraph_t infer_fillinedges(const std::vector<int> & permutation_order, const UndirectedGraph_t & g);

  CliqueTree_t MaxCardinalitySearch(const UndirectedGraph_t & cover_graph);

}
