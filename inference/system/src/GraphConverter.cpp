#include "system/GraphConverter.h"
#include "system/GraphConverter.hpp"

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

GraphConverter::CliqueTree_t GraphConverter::MaxCardinalitySearch(const UndirectedGraph_t & cover_graph)
{
  return GraphConverter::CliqueTree_t();
}
