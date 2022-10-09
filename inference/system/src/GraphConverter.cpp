#include "system/GraphConverter.h"
#include "system/GraphConverter.hpp"
#include <boost/graph/graph_traits.hpp>

using namespace sam::Inference;

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

// internal
std::tuple<std::size_t, std::vector<std::size_t> > find_appropriate_edge_clique(const GraphConverter::CliqueTree_t & ct, const std::vector<std::size_t> & keys)
{
  // maybe there is a way to "rangify"

  std::size_t clique_index;

  std::vector<std::size_t> sepset;
  
  using VertexCTBundleMap = typename boost::property_map<GraphConverter::CliqueTree_t, boost::vertex_bundle_t>::const_type;
  using VertexCTIndexMap = typename boost::property_map<GraphConverter::CliqueTree_t, boost::vertex_index_t>::const_type;
  VertexCTBundleMap v_bundle_map = boost::get(boost::vertex_bundle, ct);
  VertexCTIndexMap v_index_map = boost::get(boost::vertex_index, ct);

  // for each vertex in ct reverse order, register & count the size of common keys, until it 
  auto [clique_first, clique_end] = boost::vertices(ct);
  // most times (not always), the last clique is the one pushed last, so we will proceed in reverse other
  // auto candidate_clique = *(clique_end-1);
  //
  // while( size of common key don't diminish && next node has more neighbours (out_degrees)) // if it's the same size: good, continue, we migth either reach a big sepset, or worst case, we increase clique tree branches by choosing a deeper node (look at a cross-like factor graph for a good understanding)
  // {
  //   // for every out_edges
  //   //   test sepset, count
  // }

  // first version
  while (clique_first != clique_end)
  {
    auto ci = *clique_first;
    std::vector<std::size_t> sepset_candidate;
    std::ranges::set_intersection( boost::get(v_bundle_map,ci).keys, keys, std::back_inserter(sepset_candidate)  ) ;

    if (sepset_candidate.size() > sepset.size())
    {
      sepset=sepset_candidate;
      clique_index = boost::get(v_index_map,ci);
    }

    ++clique_first;
  }

  return std::make_tuple(clique_index,sepset);
}

// TODO: also return symbolic tree of costs
GraphConverter::CliqueTree_t GraphConverter::MaxCardinalitySearch(const UndirectedGraph_t & g)
{
  PROFILE_FUNCTION();
  CliqueTree_t clique_tree;

  // declare colors map (grey/white)
  enum ColorValue { White, Grey };
  using ColorVector = std::vector<ColorValue>;
  using VIndexMap = typename boost::property_map<UndirectedGraph_t, boost::vertex_index_t>::type;
  using ColorMap = boost::iterator_property_map<typename ColorVector::iterator, VIndexMap>;
  using CardinalityMap = boost::iterator_property_map<typename std::vector<int>::iterator, VIndexMap>;
  VIndexMap v_index_map = boost::get(boost::vertex_index, g);
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

    std::size_t clique_idx=0; // clique idx, incremented each time a new clique is created
   for (;vi_it != vend_it; ++vi_it) // for-loop 
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


    if ( ci.m_base.keys.size() <= ci_prev.m_base.keys.size() ) // 1st iteration won't trigger
    {
      ci_prev.m_value= clique_idx; // idx
      ci_prev.m_base.keys.push_back(boost::get(v_index_map, vi_prev));
      boost::add_vertex(ci_prev,clique_tree); 
      auto [neighbour_clique_idx, sepset] = find_appropriate_edge_clique(clique_tree,ci_prev.m_base.keys);
      using EdgeProp = typename boost::edge_property_type<CliqueTree_t>::type;
      EdgeProp edge_prop; edge_prop.separator_keys=sepset;
      boost::add_edge(clique_idx, boost::get(v_index_map, neighbour_clique_idx), edge_prop  , clique_tree);
      clique_idx++;
    }
    // if c_i.size() <= c_{i-1}.size()
    //    c_{i-1}.add(vi-1)
    //    add vertex to clique tree and factors (difficult)

    // mark vi as grey
    boost::put(v_color_map, vi, ColorValue::Grey);

    // save vi as v_{i-1} choose next vi = the node that has max cardinality (if there exists a white neighbour, which wont be the case on last node i=size)
    vi_prev = vi;
    // vi = // FIX: vi = max cardinality element amongst white  (use ranges)
    // save c_i as c_{i-1}
    ci_prev = ci;
  }

   // add last clique
   ci.m_value = clique_idx;
   ci.m_base.keys.push_back(boost::get(v_index_map, vi));
   boost::add_vertex(ci,clique_tree);
   auto [neighbour_clique_idx, sepset] = find_appropriate_edge_clique(clique_tree,ci.m_base.keys);
   using EdgeProp = typename boost::edge_property_type<CliqueTree_t>::type;
   EdgeProp edge_prop; edge_prop.separator_keys=sepset;
   boost::add_edge(clique_idx, boost::get(v_index_map, neighbour_clique_idx), edge_prop  , clique_tree);

  return clique_tree;
}

