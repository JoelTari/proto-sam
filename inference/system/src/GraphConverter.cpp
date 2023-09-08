#include "system/GraphConverter.h"
#include "system/GraphConverter.hpp"
#include <algorithm>
#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <string>

using namespace sam::Inference;

// #define ENABLE_DEBUG_TRACE_TMP 1

// input : MRF, permutation vector
GraphConverter::UndirectedGraph_t
    GraphConverter::infer_fillinedges(const std::vector<int>&  permutation_ordering,
                                      const UndirectedGraph_t& g_org)
{
  PROFILE_SCOPE("graph infer_fillinedges");
  // define external properties:  vertex color (eliminated/not eliminated) white / grey
  //                              ordering
  enum ColorValue
  {
    White,
    Grey,
    Orange
  };
  // copy graph
  auto g = g_org;

  // std::vector<std::pair<std::size_t, std::size_t>> r;
  using ColorVector = std::vector<ColorValue>;

  using VIndexMap =
      typename boost::property_map<UndirectedGraph_t, boost::vertex_index_t>::type;
  using VBundleMap =
      typename boost::property_map<UndirectedGraph_t, boost::vertex_bundle_t>::type;

  using ColorMap = boost::iterator_property_map<typename ColorVector::iterator, VIndexMap>;
  using OrderingMap
      = boost::iterator_property_map<typename std::vector<int>::const_iterator, VIndexMap>;

  VIndexMap  v_index_map  = boost::get(boost::vertex_index, g);
  VBundleMap v_bundle_map = boost::get(boost::vertex_bundle, g);

  ColorVector vcolors(boost::num_vertices(g), ColorValue::White);   // all node colors to white

  // map with an internal property (not necessary if vecS but still helpful if the structure
  // changes)
  ColorMap    vcolor_map = boost::make_iterator_property_map(vcolors.begin(), v_index_map);
  OrderingMap vordering_map
      = boost::make_iterator_property_map(permutation_ordering.begin(), v_index_map);

  // for each e in permutation vector
  for (std::size_t nat_idx = 0; nat_idx < permutation_ordering.size(); ++nat_idx)
  {
    std::size_t ord_idx = permutation_ordering[nat_idx];
    // std::cout << "ord idx " << v_bundle_map[ord_idx].key_id << '\n';

    // vertex of interest
    // std::size_t vn = boost::get(v_index_map,ord_idx);
    auto VoI = boost::vertex(ord_idx, g); // FIX: here is the failure point when VertexList=setS
    // connect all white neighbours of VoI between themselves
    // step1: some neighbours might have been eliminated alreaady: retain only the white vertices:
    // and make them orange
    for (auto [Vn_i, Vn_end] = boost::adjacent_vertices(VoI, g); Vn_i != Vn_end; ++Vn_i)
    {
      auto NoI = *Vn_i;
      // if (vcolor_map[v_index_map[NoI]] == ColorValue::White)
      if (boost::get(vcolor_map, NoI) == ColorValue::White)
      {
        // std::cout << "  Make " << v_bundle_map[v_index_map[NoI]].key_id << " orange (neighbour of "
        //           << v_bundle_map[v_index_map[VoI]].key_id << ")\n";
        boost::put(vcolor_map,NoI, Orange);
      }
      // orange is a temporary color -> they will be white afterwards
    }
    // step2: for each ajacent vertex of VoI, connect with all other orange neighbours (if
    // unconnected), then become white again
    for (auto [Vn_i, Vn_end] = boost::adjacent_vertices(VoI, g); Vn_i != Vn_end; ++Vn_i)
    {
      auto NoI = *Vn_i;
      if (boost::get(vcolor_map, NoI) == ColorValue::Orange)
      {
        // std::cout << "  Connect " << v_bundle_map[NoI].key_id << " with every neighbours of "
        //           << v_bundle_map[VoI].key_id << "\n";
        // this vertex becomes white again
        // vcolor_map[v_index_map[NoI]] = White;
        boost::put(vcolor_map,NoI, White);
        // test if edge exist between NoI and others orange adj_of_VoI
        for (auto [Vn2_i, Vn2_end] = boost::adjacent_vertices(VoI, g); Vn2_i != Vn2_end; ++Vn2_i)
        {
          auto NoI2 = *Vn2_i;
          // if NoI2 is orange
          if ( boost::get(vcolor_map, NoI2) == ColorValue::Orange)
          {
            auto [ed, EdgeExistsAlready] = boost::edge(NoI, NoI2, g);
            GraphConverter::MRFEdgeBundle edge_prop;
            edge_prop.fillInEdge = true;
            if (!EdgeExistsAlready) boost::add_edge(NoI,NoI2,edge_prop,g);
            // // if no edge exists between NoI & NoI2
            // auto [ed, EdgeExistsAlready] = boost::edge(NoI, NoI2, g);
            // if (!EdgeExistsAlready)
            // {
            //   // std::cout << "     new edge : \t ";
            //   r.emplace_back(source(ed, g), target(ed, g));   // TODO: add in the graph
            //   // std::cout << v_bundle_map[v_index_map[source(ed, g)]].key_id << ", "
            //   //           << v_bundle_map[v_index_map[target(ed, g)]].key_id << '\n';
            //   // update graph
            //   boost::add_edge(NoI,NoI2,g);
            // }
          }
          // orange is a temporary color -> they will be white afterwards
        }
      }
    }
    // step3: grey out VoI
    // vcolor_map[VoI] = ColorValue::Grey;
    boost::put(vcolor_map,VoI, ColorValue::Grey);
  }
  // std::cout << " added pairs : " << r.size() << '\n';

  return g;
}

// // internal
// std::tuple<std::size_t, std::vector<std::size_t> > find_appropriate_edge_clique(const GraphConverter::CliqueTree_t & ct, const std::vector<std::size_t> & keys)
// {
//   PROFILE_FUNCTION();
//   // maybe there is a way to "rangify"
//
//   std::size_t clique_index;
//
//   std::vector<std::size_t> sepset;
//   
//   using VertexCTBundleMap = typename boost::property_map<GraphConverter::CliqueTree_t, boost::vertex_bundle_t>::const_type;
//   using VertexCTIndexMap = typename boost::property_map<GraphConverter::CliqueTree_t, boost::vertex_index_t>::const_type;
//   VertexCTBundleMap v_bundle_map = boost::get(boost::vertex_bundle, ct);
//   VertexCTIndexMap v_index_map = boost::get(boost::vertex_index, ct);
//
//   // for each vertex in ct reverse order, register & count the size of common keys, until it 
//   auto [clique_first, clique_end] = boost::vertices(ct);
//   // most times (not always), the last clique is the one pushed last, so we will proceed in reverse other
//   // auto candidate_clique = *(clique_end-1);
//   //
//   // while( size of common key don't diminish && next node has more neighbours (out_degrees)) // if it's the same size: good, continue, we migth either reach a big sepset, or worst case, we increase clique tree branches by choosing a deeper node (look at a cross-like factor graph for a good understanding)
//   // {
//   //   // for every out_edges
//   //   //   test sepset, count
//   // }
//
//   // // first version: while loop (65 ms)
//   // while (clique_first != clique_end)
//   // {
//   //   auto ci = *clique_first;
//   //   std::vector<std::size_t> sepset_candidate;
//   //   std::ranges::set_intersection( boost::get(v_bundle_map,ci).keys, keys, std::back_inserter(sepset_candidate)  ) ;
//   //
//   //   if (sepset_candidate.size() > sepset.size())
//   //   {
//   //     sepset=sepset_candidate;
//   //     clique_index = boost::get(v_index_map,ci);
//   //   }
//   //
//   //   ++clique_first;
//   // }
//
//   // second version: brut force for loop (on m3500: 109 ms, 68ms if par policy, 58ms if par_unseq policy)
//   auto lambda_sepset_candidancy_evaluate = [&]
//     (auto a
//       , auto b) ->bool 
//   {
//     std::vector<std::size_t> keys_cap_a, keys_cap_b;
//     std::ranges::set_intersection( boost::get(v_bundle_map,a).keys, keys, std::back_inserter(keys_cap_a)  ) ;
//     std::ranges::set_intersection( boost::get(v_bundle_map,b).keys, keys, std::back_inserter(keys_cap_b)  ) ;
//     return ( keys_cap_a.size() < keys_cap_b.size() );
//   };
//   auto best_clique_it = 
//     std::max_element(std::execution::par_unseq,clique_first, clique_end, lambda_sepset_candidancy_evaluate );
//   // push in the sepset (this is a double recompution of the sepset)
//   std::ranges::set_intersection( boost::get(v_bundle_map,*best_clique_it).keys, keys, std::back_inserter(sepset)  ) ;
//
//   // third version: traverse from most recent clique and evaluate separator
//   // while (size doesnt diminish)
//
//   // fourth version:  as clique are created,  evaluate somehow if we are connected in a junction
//
//   return std::make_tuple(clique_index,sepset);
// }





std::tuple<std::size_t, std::vector<std::size_t>> find_out_best_neighbour_clique_and_update_mapping
  (
    const std::vector<std::size_t> & new_clique_keysIdx_in_Gp
    , const std::size_t new_cliqueIdx_ct 
    , const GraphConverter::CliqueTree_t & ct
    ,  std::unordered_map<std::size_t,std::vector<std::size_t>> & keysIdxGp_2_cliquesIdxCt
    , bool PreferBigJunction = true // if true, performance decreases from 14.8 to 15.4ms (4% slower)
    )
{
  // if (keysIdxGp_2_cliquesIdxCt.empty())
  //   throw std::runtime_error("big map is empty");
  PROFILE_FUNCTION();
  // const auto v_bundle_map_CT = boost::get(boost::vertex_bundle, ct);
  typename boost::property_map<GraphConverter::CliqueTree_t, boost::vertex_bundle_t>::const_type
    v_bundle_map_CT = boost::get(boost::vertex_bundle, ct);;
  using score_t = std::size_t;
  using clique_idx_t  = std::size_t;
  std::unordered_map<clique_idx_t, score_t> candidates_clique_to_score={};
  // find all the cliques that have nonzeros intersection
  for (const auto new_clique_keyIdx_Gp : new_clique_keysIdx_in_Gp)
  {
#if ENABLE_DEBUG_TRACE_TMP == 2
    std::cout << " finding all the cliques that have x"<< new_clique_keyIdx_Gp << "\n";
#endif
    // if the key exist in the mapping
    // add the cliques that key is into in the potential candidate
    if (auto it = keysIdxGp_2_cliquesIdxCt.find(new_clique_keyIdx_Gp); it != keysIdxGp_2_cliquesIdxCt.end())
    {
      auto list_of_cliquesIdxCt = it->second;
      // now this clique also
      it->second.push_back(new_cliqueIdx_ct);
#if ENABLE_DEBUG_TRACE_TMP == 2
      std::cout << "   there are " <<  list_of_cliquesIdxCt.size() << '\n';
#endif
      for (const auto cliqueIdxCt : list_of_cliquesIdxCt)
      {
#if ENABLE_DEBUG_TRACE_TMP == 2
          std::cout << "     c" << cliqueIdxCt << "\n";
#endif
        // if this candidate clique exists already, increment its score
        if (auto it_can = candidates_clique_to_score.find(cliqueIdxCt) ; it_can !=candidates_clique_to_score.end())
        {
          it_can->second++;
        }
        else // if this candidate clique doesn't exist, create it with a score of 1
        {
          candidates_clique_to_score.insert_or_assign(cliqueIdxCt, 1);
        }
      }
    }
    else // key doesn't exist in the CT yet, in anticipation we give it the new clique idx in ct 
    {
#if ENABLE_DEBUG_TRACE_TMP == 2
      std::cout << "   there are none ! (Inserting in big map) \n";
#endif
      keysIdxGp_2_cliquesIdxCt.insert_or_assign(new_clique_keyIdx_Gp, std::vector<std::size_t>{ new_cliqueIdx_ct } );
    }
  }

  // deduce best candidate (max score)
#if ENABLE_DEBUG_TRACE_TMP == 2
  std::cout << "  clique candidates [with score]: (";
  for (auto p : candidates_clique_to_score)
  {
    std::cout << " c" << std::to_string(p.first) << "[" << p.second << "]" << ", ";
  }
  std::cout << ")\n";
#endif
  // choosing the best candidate this new clique will connect to
  auto best_it = std::max_element(candidates_clique_to_score.begin(), candidates_clique_to_score.end(), 
      [PreferBigJunction,&v_bundle_map_CT,&ct](const auto & c1, const auto & c2 )
      { 
        if (!PreferBigJunction)
        {
          return c1.second < c2.second; 
        }
        else 
        // more complex system that choose, among valid solutions, the one with most neighbour to help promote big junctions
        // so that it helps create more paralellism.
        // TODO: somehow limit the clique degrees given a cap on number of threads (given as a parameter)
        //       This would not garantee an upper bound on clique degrees (because c1.second < c2.second still required for RIP)
        //       Unless (if we really want to enfore that), we have non-max clique (spreading a maxclique into several cliques)
        {
          if (c1.second < c2.second)
          {
            return true;
          }
          else if ( c1.second > 0 && c1.second == c2.second)
          {
            // count vertex out_degrees return c1.out_degrees < c2.out_degrees
          // boost::degree(boost::get(v_bundle_map_CT, c1.first), ct);
            uint c1_degree = boost::out_degree( boost::vertex(c1.first,ct) , ct);
            uint c2_degree = boost::out_degree( boost::vertex(c2.first,ct) , ct);
#if ENABLE_DEBUG_TRACE_TMP == 2
            if (c1_degree < c2_degree)
            {
              std::cout << "    prefer c" << c2.first << " ( " << c2_degree << " neighs) over c" << c1.first << " ( "<< c1_degree << " neighs)\n";
            }
#endif
            return c1_degree < c2_degree;
          }
          else return false;
        }
      });
#if ENABLE_DEBUG_TRACE_TMP == 2
  std::cout << "  The best clique is: c"; 
  std::cout << best_it->first << "[" << best_it->second << "]\n";
#endif
  auto best_cliqueIdx_CT = best_it->first;

  // TODO: assert that at least one of the key in the mapping didn't exist (because of running intersection property)
  // TODO: assert that at least one of the key in the mapping did exist (because of running intersection property)

  // compute the sepset (edge)
  std::vector<std::size_t> sepset;
#if ENABLE_DEBUG_TRACE_TMP == 2
  std::cout << "  Intersection between c" << best_cliqueIdx_CT << " and c" << new_cliqueIdx_ct << " : {";
  for (auto k : boost::get(v_bundle_map_CT, best_cliqueIdx_CT).keys) std::cout << " x"<< k <<" ";
  std::cout << "}  -  {";
  for (auto k : new_clique_keysIdx_in_Gp) std::cout << " x"<< k <<" ";
  std::cout << "}\n"; 
#endif
  // WARNING: for set_intersection to work, the set have to be sorted (this is done upstream)
  std::ranges::set_intersection( 
        boost::get(v_bundle_map_CT, best_cliqueIdx_CT).keys
      , new_clique_keysIdx_in_Gp
      , std::back_inserter(sepset));
  // idx clique in CT
  return std::make_tuple(best_it->first, sepset) ;
  // return {0, {0,0}};
}


//------------------------------------------------------------------//
//                    Maximum Cardinality Search                    //
//------------------------------------------------------------------//
// TODO: also return symbolic tree of costs -> implies need for more info in g ??
// TODO: return the mapping key to cliqueIdx(s)
GraphConverter::CliqueTree_t GraphConverter::MaxCardinalitySearch(const UndirectedGraph_t & g)
{
  PROFILE_FUNCTION();
  CliqueTree_t clique_tree;

  // declare colors map (grey/white)
  enum ColorValue { White, Grey };
  using ColorVector = std::vector<ColorValue>;
  using VIndexMap = typename boost::property_map<UndirectedGraph_t, boost::vertex_index_t>::type;
  using VBundleMap = typename boost::property_map<UndirectedGraph_t, boost::vertex_bundle_t>::const_type;
  using ColorMap = boost::iterator_property_map<typename ColorVector::iterator, VIndexMap>;
  using CardinalityMap = boost::iterator_property_map<typename std::vector<int>::iterator, VIndexMap>;
  VIndexMap v_index_map = boost::get(boost::vertex_index, g);
  VBundleMap v_bundle_map = boost::get(boost::vertex_bundle, g);
  ColorVector vcolors(boost::num_vertices(g), ColorValue::White);
  ColorMap v_color_map = boost::make_iterator_property_map(vcolors.begin(), v_index_map);

  // declare cardinality map (int)
  std::vector<int> vcardinality(boost::num_vertices(g), 0);
  CardinalityMap v_cardinality_map = boost::make_iterator_property_map(vcardinality.begin(), v_index_map);

  // pick first node of cover graph (arbitrary)
  auto [vi_it,vend_it] = boost::vertices(g);
  auto vi = *vi_it;
  decltype(vi) vi_prev;

  // Declare returned graph (the clique tree) TODO: and symbolic tree of cost
  // declare c_{i-1} empty
  // boost::graph_traits<CliqueTree_t>::vertex_descriptor ci_prev;
  boost::vertex_property_type<CliqueTree_t>::type ci_prev, ci;
  
  // mapping of key idx (in G' the cover graph) to list clique(s) idx (in clique tree) that contain said key
  std::unordered_map<std::size_t, std::vector<std::size_t>> keysIdxGp_2_cliquesIdxCt = {};

  std::size_t clique_idx=0; // clique idx, incremented each time a new clique is created
  for (auto vii_it = vi_it;vii_it != vend_it; ++vii_it) // for-loop 
  {
    // reset c_i, as an empty clique
    ci = boost::vertex_property_type<CliqueTree_t>::type(); 


    // loop the neighbours of vi 
    for (auto [vni_it, vnend_it] = boost::adjacent_vertices(vi,g) ; vni_it != vnend_it; ++vni_it)
    {
      auto vni = * vni_it;
      // if white, increment cardinality
      if ( boost::get(v_color_map, vni) == ColorValue::White )
      {
        boost::put(v_cardinality_map, vni, boost::get(v_cardinality_map, vni)+1 );
      }
      // if grey append to c_i 
      else
      {
        ci.m_base.keys.push_back( boost::get(v_index_map, vni) );
      }
    }

    // OPTIONALLY: test that {vi \cup c_i} is complete


    if ( (ci.m_base.keys.size() <= ci_prev.m_base.keys.size() ) 
        && vii_it != vi_it) // 1st iteration won't trigger
    {
      ci_prev.m_value= clique_idx; // idx
      ci_prev.m_base.keys.push_back(boost::get(v_index_map, vi_prev));
      std::ranges::sort(ci_prev.m_base.keys); //sort (necessary for set_intersection)
      boost::add_vertex(ci_prev,clique_tree); 
      // #if ENABLE_DEBUG_TRACE_TMP
      //       std::cout << "New clique formed: c" << std::to_string(clique_idx) << " ( ";
      //       for (auto k : ci_prev.m_base.keys)
      //       {
      //         std::cout << boost::get(v_bundle_map,boost::vertex(k, g)).key_id << ", ";
      //       }
      //       std::cout << ")\n"; 
      // #endif
      // auto [neighbour_clique_idx, sepset] = find_appropriate_edge_clique(clique_tree,ci_prev.m_base.keys);
      using EdgeProp = typename boost::edge_property_type<CliqueTree_t>::type;
      if ( boost::num_vertices(clique_tree) > 1 )
      {
        auto [neighbour_clique_idx, sepset] = find_out_best_neighbour_clique_and_update_mapping(ci_prev.m_base.keys, ci_prev.m_value, clique_tree, keysIdxGp_2_cliquesIdxCt);
        EdgeProp edge_prop; edge_prop.separator_keys=sepset;
        boost::add_edge(clique_idx, boost::get(v_index_map, neighbour_clique_idx), edge_prop  , clique_tree);
        // #if ENABLE_DEBUG_TRACE_TMP
        //         std::cout << "  Edge between  c" << neighbour_clique_idx << " - c" << clique_idx << " \n"; 
        //         std::cout << "  Sepset: {"; for (auto s: sepset) { std::cout << "x" << s << ", "; } std::cout << "}\n";
        // #endif
      } // else (first cclique), don't add edge, but register the key to clique mapping
      else
      {
        // #if ENABLE_DEBUG_TRACE_TMP==2
        //         std::cout << "  special treatment for first clique \n";
        // #endif
        for( auto key : ci_prev.m_base.keys )
        {
          // #if ENABLE_DEBUG_TRACE_TMP==2
          //           std::cout << "  insert x" << key << " in big map\n";
          // #endif
          keysIdxGp_2_cliquesIdxCt.insert_or_assign(key, std::vector<std::size_t>{clique_idx});
        }
      }
      clique_idx++;
    }

    // mark vi as grey, set its cardinality to 0 (to simplify max cardinality search below) 
    boost::put(v_color_map, vi, ColorValue::Grey);
    boost::put(v_cardinality_map, vi, 0);

    if (vii_it != vend_it -1)
    {
      // save vi as v_{i-1} choose next vi = the node that has max cardinality (if there exists a white neighbour, which wont be the case on last node i=size)
      vi_prev = vi;
      vi = boost::vertex( std::distance(vcardinality.begin() ,std::max_element(vcardinality.begin(),vcardinality.end()) ), g );
      // #if ENABLE_DEBUG_TRACE_TMP
      //       std::cout << "vi : " << boost::get(v_bundle_map, vi).key_id << '\n';
      // #endif
      // save c_i as c_{i-1}
      ci_prev = ci;

    }
  }

  // add last clique
  ci.m_value = clique_idx;
  ci.m_base.keys.push_back(boost::get(v_index_map, vi));
  std::ranges::sort(ci.m_base.keys); //sort (necessary for set_intersection)
  boost::add_vertex(ci,clique_tree);
  // #if ENABLE_DEBUG_TRACE_TMP
  //       std::cout << "New (last) clique formed: c" << std::to_string(clique_idx) << " ( ";
  //       for (auto k : ci.m_base.keys)
  //       {
  //         std::cout << boost::get(v_bundle_map,boost::vertex(k, g)).key_id << ", ";
  //       }
  //       std::cout << ")\n";
  // #endif
   // auto [neighbour_clique_idx, sepset] = find_appropriate_edge_clique(clique_tree,ci.m_base.keys);
  using EdgeProp = typename boost::edge_property_type<CliqueTree_t>::type;
  if ( boost::num_vertices(clique_tree) > 1 )
  {
    auto [neighbour_clique_idx, sepset] = find_out_best_neighbour_clique_and_update_mapping(ci.m_base.keys, ci.m_value, clique_tree, keysIdxGp_2_cliquesIdxCt);
    EdgeProp edge_prop; edge_prop.separator_keys=sepset;
    boost::add_edge(clique_idx, boost::get(v_index_map, neighbour_clique_idx), edge_prop  , clique_tree);
    // #if ENABLE_DEBUG_TRACE_TMP
    //   std::cout << "  Edge between  c" << neighbour_clique_idx << " - c" << clique_idx << " \n"; 
    //   std::cout << "  Sepset: {"; for (auto s: sepset) { std::cout << "x" << s << ", "; } std::cout << "}\n";
    // #endif
  } // else (last clique is also first and therefore only clique), don't add edge, but register the key to clique mapping
  else
  {
    // #if ENABLE_DEBUG_TRACE_TMP==2
    //   std::cout << "  special treatment for first clique \n";
    // #endif
    for( auto key : ci_prev.m_base.keys )
    {
    // #if ENABLE_DEBUG_TRACE_TMP==2
    //   std::cout << "  insert x" << key << " in big map\n";
    // #endif
    keysIdxGp_2_cliquesIdxCt.insert_or_assign(key, std::vector<std::size_t>{clique_idx});
    }
  }

  return clique_tree;
}

