#include "system/HybridConverter.h"

using DispatchContainer_t = sam::Inference::SystemConverter::DispatchContainer_t;
using DispatchInfos_t     = sam::Inference::SystemConverter::KeyDispatchInfos;

/**
 * @brief infer fill in edges in the dispatch container that lead to a chordal graph given a
 * permutation ordering
 *
 * @param permutation_vector
 * @param dispatch_container
 * @return vector of fill in edges, each edge is described by a pair of string : the keys id
 */
std::vector<std::pair<std::string, std::string>>
    sam::Inference::HybridConverter::infer_fillinedges(const std::vector<int>&    permutation_vector,
                      const DispatchContainer_t& dispatch_container)
{
  PROFILE_FUNCTION();
  std::vector<std::pair<std::string, std::string>> fillin_edges;

  // copy the dispatch container (because its size will shrink as we elimiate + some fill in edge
  // gets added)
  auto dispatch_container_cpy(dispatch_container);

  DispatchContainer_t::nth_index<0>::type& container_by_keyid   = dispatch_container_cpy.get<0>();
  DispatchContainer_t::nth_index<1>::type& container_by_nat_idx = dispatch_container_cpy.get<1>();


  for (int nat_idx : permutation_vector)
  {
    auto it_KoI_dispatch = container_by_nat_idx.find(nat_idx);   // save iterator for update pattern
    DispatchInfos_t KoI_dispatch_tmp = *it_KoI_dispatch;
    auto            KoI              = KoI_dispatch_tmp.key_id;
    // std::cout << "\t" << KoI << " elim\n";

    if (KoI_dispatch_tmp.neighbours.size() > 0)
    {
      // std::cout << "\t  Have these must completly connected : { ";
      // for (auto NoI : KoI_dispatch_tmp.neighbours) { std::cout << NoI << ", "; }
      // std::cout << " }\n";

      // connect all neighbours between themselves
      // make sure NoI is connected with others neighbours of KoI
      // for (auto NoI : KoI_dispatch_tmp.neighbours)
      while (KoI_dispatch_tmp.neighbours.size() > 1)
      {
        auto NoI = *KoI_dispatch_tmp.neighbours.begin();
        // std::cout << "\t\t " << NoI << ":\n";
        // erase NoI from neighbours of KoI
        KoI_dispatch_tmp.neighbours.erase(NoI);
        // erase KoI from neighbours of NoI (modify container)
        auto it_NoI_dispatch  = container_by_keyid.find(NoI);
        auto NoI_dispatch_tmp = *it_NoI_dispatch;
        NoI_dispatch_tmp.neighbours.erase(KoI);      // cpy
        for (auto n : KoI_dispatch_tmp.neighbours)   // we will connect NoI with n
        {
          auto [it, EdgeCreated] = NoI_dispatch_tmp.neighbours.insert(n);
          //
          if (EdgeCreated)
          {
            // std::cout << "\t\t\tFill-in edge added: " << NoI << "-" << n << "\t (+)\n";
            // FIX: add neighbour in NoI and in n
            fillin_edges.emplace_back(NoI, n);
            // add also the fill-in from n to NoI
            auto it_n           = container_by_keyid.find(n);
            auto n_dispatch_tmp = *it_n;   // cpy
            n_dispatch_tmp.neighbours.insert(NoI);
            container_by_keyid.replace(it_n, n_dispatch_tmp);
          }
          // else { std::cout << "\t\t\tEdge " << NoI << "-" << n << " exists\n"; }
        }
        // modification to the copied container: addition of unconnected neighbours + removing of
        // KoI as neighbour
        container_by_keyid.replace(it_NoI_dispatch, NoI_dispatch_tmp);
      }
      // one neighbour left to deal with
      auto NoI = *KoI_dispatch_tmp.neighbours.begin();
      // std::cout << "\t\t " << NoI << ": last neigh is connected with the others\n";
      auto it_NoI_dispatch  = container_by_keyid.find(NoI);
      auto NoI_dispatch_tmp = *it_NoI_dispatch;
      NoI_dispatch_tmp.neighbours.erase(KoI);   // cpy
      container_by_keyid.replace(it_NoI_dispatch, NoI_dispatch_tmp);

      // std::cout << "\t\t Done connecting neighbours of " << KoI << '\n';
    }
    else
    {
      // no neighbours ? -> it has to be the last key to be eliminated
      // otherwise it is an error (or the graph has 1 element isolated)
      if (dispatch_container_cpy.size() > 1)
        throw std::runtime_error(
            KoI
            + " has 0 neighbour (not connected with remaining nodes). Corner case not supported.");
    }
    dispatch_container_cpy.erase(KoI);
  }
  return fillin_edges;
}

/**
 * @brief amd ordering given the sparse semantic hessian
 *
 * @param N number of keys, or size of the semantic hessian, or number of columns of the semantic
 * jacobian
 * @param hessian_outer_indexes compressed sparse matrix storage : array of outer indexes
 * @param hessian_inner_indexes compressed sparse matrix storage : array of inner indexes
 * @return stl vector of the permutation from the 'natural' semantic order (order in which the keys
 * are registered)
 */
std::vector<int> sam::Inference::HybridConverter::amd_order_permutation(int              N,
                                       const int* const hessian_outer_indexes,
                                       const int* const hessian_inner_indexes)
{
  PROFILE_SCOPE("amd ordering");
  std::vector<int> permutation_vector(N);
  permutation_vector.resize(N);
  amd_order(N,
            hessian_outer_indexes,
            hessian_inner_indexes,
            &permutation_vector[0],
            (double*)NULL,
            (double*)NULL);
  return permutation_vector;
}
