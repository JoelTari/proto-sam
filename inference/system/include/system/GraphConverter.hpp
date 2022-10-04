#pragma once

#include "system/SystemConverter.hpp"
#include "utils/config.h"
#include "utils/utils.h"
// #include <boost/multi_index_container.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <vector>


namespace sam::Inference::GraphConverter
{
  // no namespace : BGL

  struct MRFVertexBundle
  {
    std::string key_id;
    int         semantic_ordering_idx;
  };

  struct MRFEdgeBundle
  {
  };

  struct MRFGraphBundle
  {
  };

  // https://www.boost.org/doc/libs/1_80_0/libs/graph/doc/adjacency_list.html
  using UndirectedGraph_t = boost::adjacency_list<boost::vecS,
                                                  boost::vecS,
                                                  boost::undirectedS,
                                                  MRFVertexBundle,
                                                  MRFEdgeBundle,
                                                  MRFGraphBundle,
                                                  boost::vecS>;

  template <typename VWF, typename VWM>
  UndirectedGraph_t build_undirected_graph(
      const VWF& tup_vwf, const VWM & tup_vwm
      , const typename SystemConverter::DispatchContainer_t& keys_affectation)
  {
      PROFILE_SCOPE("build MRF from data");
      // this->MRF = GraphConverter::build_undirected_graph(
      //     this->all_factors_tuple_,
      //     this->all_vectors_marginals_
      //     );
      int N = keys_affectation.size();
      int M = SystemConverter::Semantic::M(tup_vwf);

      std::vector<std::pair<int,int>> edges;
      std::vector<GraphConverter::MRFEdgeBundle> edges_properties;
        // property map
        // put( my_property_map_type , {e or v or g}  , value )
        // get(my_property_map_object, {e or v or g}) -> return a copy
        // at() -> return the a reference
        // value type of a property ? use property_traits<MyPropertyMap>::value_type (e.g., int / std::string)

      std::apply(
          [&](const auto & ...vwf)
          {
             auto lambda = [&](const auto & avwf)
             {
               using Factor_t = typename std::remove_cvref_t<decltype(avwf)>::value_type::Factor_t  ;
               if constexpr ( Factor_t::kNbKeys == 2 )
               {
                 for(const auto & wf : avwf)
                 {
                    std::array<int, 2> v_indexes;
                   // edges = all possible combinations of keys
                   int v1_idx = keys_affectation.find(std::get<0>(wf.factor.keys_set).key_id)->natural_semantic_idx;
                   int v2_idx = keys_affectation.find(std::get<1>(wf.factor.keys_set).key_id)->natural_semantic_idx;
                   edges.emplace_back(v1_idx,v2_idx);
                   edges_properties.emplace_back(); // TODO:
                 }
               }
               else 
               {
                 if (Factor_t::kNbKeys > 1)
                 {
                   // FIX: implement for factors with more than 2
                   std::runtime_error("combinations edges of more than 2 keys not implemented yet");
                 }
               }
             };
             ((lambda(vwf)), ...);
          }
          ,tup_vwf);


      // boost::graph_traits< UndirectedGraph_t >::vertex_descriptor v1 = this->MRF.add_vertex();
      // boost::add_vertex(this->MRF);

      std::cout << edges.size() << " pairs\n";
      auto MRF = GraphConverter::UndirectedGraph_t(edges.begin(), edges.end(), edges_properties.begin() ,N,M, GraphConverter::MRFGraphBundle());
      // auto vertex_index_map = boost::get( boost::vertex_index, this->MRF );
      return MRF;
  }

}   // namespace sam::Inference::GraphConverter
