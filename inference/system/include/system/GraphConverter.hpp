#pragma once

#include "system/SystemConverter.hpp"
#include "utils/config.h"
#include "utils/utils.h"
// #include <boost/multi_index_container.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <vector>


namespace sam::Inference::GraphConverter
{
  // no namespace : BGL

  struct MRFVertexBundle
  {
    std::string key_id ="";
    int         semantic_ordering_idx=0;
  };

  struct MRFEdgeBundle
  {
    // perhaps pair of string of the key_id ?
  };

  struct MRFGraphBundle
  {
  };

  // https://www.boost.org/doc/libs/1_80_0/libs/graph/doc/adjacency_list.html
  using UndirectedGraph_t = boost::adjacency_list<boost::vecS, // OutEdgeList  (def vecS)
                                                  boost::vecS,      // VertexList (def vecS)
                                                  boost::undirectedS,    // def directedS
                                                  MRFVertexBundle,  // prop
                                                  MRFEdgeBundle,    // prop
                                                  MRFGraphBundle,   // prop
                                                  boost::listS>; // edge list (def listS)
  // choosing edgelist and vertexlist (last & first)
  // https://www.boost.org/doc/libs/1_80_0/libs/graph/doc/using_adjacency_list.html#sec:choosing-graph-type

  template <typename TUP_VWF>
  std::size_t UpperBoundNumberOfUndirectedEdges(const TUP_VWF & tup_vwf)
  {
    // the number of unique pairs in a set for a given factor type is n(n-1)/2, where n is the number of keys
    // the number of undirected edges is n(n-1)/2 * vector<Factor_t>.size()
    // some double count may occur with parallel factors, that is why it is an upper bound
    // note: a prior factor doesnot entail an edge (and indeed n-1 will be 0)

    return 
      std::apply(
          [](const auto & ... vwf)
          {
            auto lambda = [](const auto & avwf) -> std::size_t
            {
              std::size_t n = std::remove_cvref_t<decltype(avwf)>::value_type::Factor_t::kNbKeys;
              return (n*(n-1)/2 )*avwf.size();
            };
            return (lambda(vwf) + ...) ;
          },tup_vwf);

  }

  template <typename VWF, typename VWM>
  UndirectedGraph_t
      build_undirected_graph(const VWF&                                           tup_vwf,
                             const VWM&                                           tup_vwm,
                             const typename SystemConverter::DispatchContainer_t& keys_affectation)
  {
    PROFILE_SCOPE("build MRF");

    int N = keys_affectation.size();
    int M = SystemConverter::Semantic::M(tup_vwf);

    // putting the 
    std::vector<std::pair<int, int>>           edges;
    edges.reserve(UpperBoundNumberOfUndirectedEdges(tup_vwf));
    std::vector<GraphConverter::MRFEdgeBundle> edges_properties;
    // property map
    // put( my_property_map_type , {e or v or g}  , value )
    // get(my_property_map_object, {e or v or g}) -> return a copy
    // at() -> return the a reference
    // value type of a property ? use property_traits<MyPropertyMap>::value_type (e.g., int /
    // std::string)

    // fast 晴 (0.5 ms on 3500)
    std::apply(
        [&](const auto&... vwf)
        {
          PROFILE_SCOPE("prepare edges");
          auto lambda = [&](const auto& avwf)
          {
            using Factor_t = typename std::remove_cvref_t<decltype(avwf)>::value_type::Factor_t;
            // note: no edge is registered for prior factors (or any factor that act on a unique
            // key)
            if constexpr (Factor_t::kNbKeys > 1)
            {
              for (const auto& wf : avwf)
              {
                std::array<std::size_t, Factor_t::kNbKeys> v_indexes;
                // 1. collect indexes
                v_indexes = std::apply(
                    [&](const auto&... kcm) -> std::array<std::size_t, 2>
                    { return {keys_affectation.find(kcm.key_id)->natural_semantic_idx...}; },
                    wf.factor.keys_set);
                // 2. combine: e.g. [2,4,7] makes [{2,4},{2,7},{4,7}]
                for (int i = 0; i < Factor_t::kNbKeys - 1; ++i)
                {
                  // combine i with all j > i
                  for (int j = i + 1; j < Factor_t::kNbKeys; ++j)
                  {
                    edges.emplace_back(v_indexes[i], v_indexes[j]);
                    edges_properties.emplace_back();   // TODO: perhaps the pair of factor ids
                  }
                }
                // edges.emplace_back(v_indexes[0],v_indexes[1]);
              }
            }
          };
          ((lambda(vwf)), ...);
        },
        tup_vwf);


    // std::cout << UpperBoundNumberOfUndirectedEdges(tup_vwf) << " (upper bound on the number of edges)\n";
    // std::cout << edges.size() << " pairs\n";
    // for ( const auto & [e1,e2] : edges)
    // {
    //   std::cout << e1 << ", " << e2 << '\n';
    // }


    // PROFILE_SCOPE("Build graph object");
    auto MRF = GraphConverter::UndirectedGraph_t(edges.begin(),
                                                 edges.end(),
                                                 edges_properties.begin(),
                                                 N,
                                                 M,
                                                 GraphConverter::MRFGraphBundle());
    // NEXT: fill properties of vertices: string key_id
    using VertexMRF = boost::graph_traits<UndirectedGraph_t>::vertex_descriptor;
    using VertexMRF_iter = boost::graph_traits<UndirectedGraph_t>::vertex_iterator;
    boost::property_map<UndirectedGraph_t, boost::vertex_bundle_t>::type vertex_bundle_map = boost::get(boost::vertex_bundle, MRF);
    boost::property_map<UndirectedGraph_t, boost::vertex_index_t>::type vertex_index_map = boost::get(boost::vertex_index, MRF);


    auto pair_vertex_iter = boost::vertices(MRF); // a pair of VertexMRF_iter
    for (; pair_vertex_iter.first != pair_vertex_iter.second; ++pair_vertex_iter.first)
    {
      VertexMRF v = *pair_vertex_iter.first; // v is just an integer
      //boost::put(boost::vertex_bundle,MRF, pair_vertex_iter.first ,vb );
      std::size_t natural_semantic_idx = vertex_index_map[v]; // this is the way I built the edge
      vertex_bundle_map[v].key_id = keys_affectation.get<1>().find(natural_semantic_idx)->key_id;
      // assert(keys_affectation.find(vertex_bundle_map[v].key_id).natural_semantic_idx == natural_semantic_idx );
      // std::cout << "-------\nPrint vertex: \n";
      // std::cout << "\tkey id : " << vertex_bundle_map[v].key_id << "\n";
      // std::cout << "\tsemantic idx: " << vertex_bundle_map[v].semantic_ordering_idx << "\n";
      // std::cout << "\tgraph index: "  << vertex_index_map[v] << "\n";
      // std::cout << "\tthe descriptor: "  << v << "\n"; 
      // // interesting, v is the same as vertex_index_map[v] (because Im using vecS ? so in that case it is still safer to use the map wrapper if ever I change the structure)
      boost::inv_adjacent_vertices(v,MRF);
    }

    // // tmp: add edge and see if edge size change
    // // parallel edges allowed because the EdgeList is vecS (setS or hash_setS wouldn't allow that)
    // std::cout << "num edges: " << boost::num_edges(MRF) << '\n';
    // auto paredge = edges[0];
    // auto [new_edg_desc, IsNew ] = add_edge(paredge.first, paredge.second,MRF); // can add properties before g
    // std::cout << "num edges after new parallel edge: " << boost::num_edges(MRF) << ". insertion occured : " << IsNew <<  '\n';
    // constexpr auto ispar = boost::is_multigraph<UndirectedGraph_t>::value; // no multigraph if OutEdgeList is (hash_)setS
    // std::cout << " multigraph : " << ispar << '\n';
    


    return MRF;
  }

}   // namespace sam::Inference::GraphConverter
