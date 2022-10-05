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

}
