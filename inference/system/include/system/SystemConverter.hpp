#pragma once

#include "utils/config.h"
#include "utils/utils.h"

#include <algorithm>
#include <execution>
#include <future>
#include <iomanip>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index_container.hpp>
// Utility methods and structures that serves both matrix systems and graph systems

namespace bmi = ::boost::multi_index;

namespace sam::Inference::SystemConverter
{

  struct KeyDispatchInfos
  {
    // 'natural' means the order is decided by the data structure holding key's marginal
    // 'semantic' refer to . E.g. [x0, x1,l0, l1, x2] has l1 natural semantic idx at 3
    // 'scalar' is the idx in terms of scalar, accounting for the dimensions of the keys
    // E.g. [x0, x1,l0, l1, x2] has l1 natural scalar idx at 8 if x* are dim 3 and l* are dim 2
    //       This is useful when converting in matrix
    std::size_t                     natural_semantic_idx;
    std::size_t                     natural_scalar_idx;
    std::size_t                     key_dim;
    std::string                     key_id;
    std::unordered_set<std::string> neighbours;   // TODO: for fun: safe_unordered_set with a mutex
  };

  // map that holds the dispatch info for all keys
  // using Keys_Affectation_t = std::unordered_map<std::string, KeyDispatchInfos>;
  using DispatchContainer_t = bmi::multi_index_container<typename SystemConverter::KeyDispatchInfos,
          bmi::indexed_by<
                    bmi::hashed_unique<
                      bmi::member<typename SystemConverter::KeyDispatchInfos, std::string, &SystemConverter::KeyDispatchInfos::key_id>
                    >
                    ,bmi::ordered_unique<
                      bmi::member<typename SystemConverter::KeyDispatchInfos, std::size_t, &SystemConverter::KeyDispatchInfos::natural_semantic_idx>
                      >
                    ,bmi::random_access<> 
            >
            >;

  namespace Scalar
  {
    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t M(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple)
    {
      return std::apply(
          [](const auto&... vect_of_wf)
          {
            return ((std::remove_cvref_t<decltype(vect_of_wf)>::value_type::Factor_t::kM
                     * vect_of_wf.size())
                    + ...);
          },
          wfactors_tuple);
    }

    template <typename TUPLE_VECT_WMARGINAL_T>
    std::size_t N(const TUPLE_VECT_WMARGINAL_T& tuple_vect_wmarginals)
    {
      return std::apply(
          [](const auto&... vect_of_wmarg)
          {
            return (
                (std::remove_cvref_t<decltype(vect_of_wmarg)>::value_type::Marginal_t::KeyMeta_t::kN
                 * vect_of_wmarg.size())
                + ...);
          },
          tuple_vect_wmarginals);
    }

    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>
        FactorTypeIndexesOffset(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple)
    {
      using array_t = std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>;

      array_t Sizes = std::apply(
          [](const auto&... vectofwf)
          {
            return array_t {// size = dimension of factor (measure) * number of factors
                            std::remove_cvref_t<decltype(vectofwf)>::value_type::Factor_t::kM
                            * vectofwf.size()...};
          },
          wfactors_tuple);

      array_t Offsets = {};
      std::partial_sum(Sizes.begin(),
                       Sizes.end() - 1,
                       Offsets.begin() + 1);   // TODO: exclusive_scan

#if ENABLE_DEBUG_TRACE
      std::cout << "Sizes (scalar) of vector of factor, by type: \n";
      std::stringstream ss;
      ss << "[ ";
      for (auto idx : Sizes) ss << idx << ", ";
      ss.seekp(-2, std::ios_base::end);
      ss << " ]\n";
      std::cout << ss.str();
      std::cout << "Starting (scalar) indexes of vector of factor, by type : \n";
      std::stringstream ss2;
      ss2 << "[ ";
      for (auto idx : Offsets) ss2 << idx << ", ";
      ss2.seekp(-2, std::ios_base::end);
      ss2 << " ]\n";
      std::cout << ss2.str();
#endif

      return Offsets;
    }

    template <typename TUPLE_VECT_WMARGINAL_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_VECT_WMARGINAL_T>>
        MarginalTypeIndexesOffset(const TUPLE_VECT_WMARGINAL_T& tuple_vect_wmarginals)
    {
      using array_t = std::array<std::size_t, std::tuple_size_v<TUPLE_VECT_WMARGINAL_T>>;

      array_t Sizes = std::apply(
          [](const auto&... vect_of_wmarg)
          {
            return array_t {
                // size = dimension of key * number of keys
                std::remove_cvref_t<decltype(vect_of_wmarg)>::value_type::Marginal_t::KeyMeta_t::kN
                * vect_of_wmarg.size()...};
          },
          tuple_vect_wmarginals);

      array_t Offsets = {};
      std::partial_sum(Sizes.begin(),
                       Sizes.end() - 1,
                       Offsets.begin() + 1);   // TODO: exclusive_scan

#if ENABLE_DEBUG_TRACE
      std::cout << "Sizes (scalar) of marginal vects, by type: \n";
      std::stringstream ss;
      ss << "[ ";
      for (auto idx : Sizes) ss << idx << ", ";
      ss.seekp(-2, std::ios_base::end);
      ss << " ]\n";
      std::cout << ss.str();
      std::cout << "Starting (scalar) indexes of vector of factor, by type : \n";
      std::stringstream ss2;
      ss2 << "[ ";
      for (auto idx : Offsets) ss2 << idx << ", ";
      ss2.seekp(-2, std::ios_base::end);
      ss2 << " ]\n";
      std::cout << ss2.str();
#endif

      return Offsets;
    }

  }   // namespace Scalar

  namespace Semantic
  {
    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t M(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple)
    {
      return std::apply([](const auto&... vect_of_wf) { return (vect_of_wf.size() + ...); },
                        wfactors_tuple);
    }

    template <typename TUPLE_VECT_WMARGINAL_T>
    std::size_t N(const TUPLE_VECT_WMARGINAL_T& tuple_vect_wmarginals)
    {
      return std::apply([](const auto&... vect_of_wmarg) { return (vect_of_wmarg.size() + ...); },
                        tuple_vect_wmarginals);
    }

    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>
        FactorTypeIndexesOffset(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple)
    {
      using array_t = std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>;

      array_t Sizes
          = std::apply([](const auto&... vectofwf) { return array_t {vectofwf.size()...}; },
                       wfactors_tuple);

      array_t Offsets = {};
      std::partial_sum(Sizes.begin(),
                       Sizes.end() - 1,
                       Offsets.begin() + 1);   // TODO: exclusive_scan

#if ENABLE_DEBUG_TRACE
      std::cout << "Sizes (scalar) of vector of factor, by type: \n";
      std::stringstream ss;
      ss << "[ ";
      for (auto idx : Sizes) ss << idx << ", ";
      ss.seekp(-2, std::ios_base::end);
      ss << " ]\n";
      std::cout << ss.str();
      std::cout << "Starting (scalar) indexes of vector of factor, by type : \n";
      std::stringstream ss2;
      ss2 << "[ ";
      for (auto idx : Offsets) ss2 << idx << ", ";
      ss2.seekp(-2, std::ios_base::end);
      ss2 << " ]\n";
      std::cout << ss2.str();
#endif

      return Offsets;
    }

    template <typename TUPLE_VECT_WMARGINAL_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_VECT_WMARGINAL_T>>
        MarginalTypeIndexesOffset(const TUPLE_VECT_WMARGINAL_T& tuple_vect_wmarginals)
    {
      using array_t = std::array<std::size_t, std::tuple_size_v<TUPLE_VECT_WMARGINAL_T>>;

      array_t Sizes = std::apply([](const auto&... vect_of_wmarg)
                                 { return array_t {vect_of_wmarg.size()...}; },
                                 tuple_vect_wmarginals);

      array_t Offsets = {};
      std::partial_sum(Sizes.begin(),
                       Sizes.end() - 1,
                       Offsets.begin() + 1);   // TODO: exclusive_scan

#if ENABLE_DEBUG_TRACE
      std::cout << "Sizes (scalar) of marginal vects, by type: \n";
      std::stringstream ss;
      ss << "[ ";
      for (auto idx : Sizes) ss << idx << ", ";
      ss.seekp(-2, std::ios_base::end);
      ss << " ]\n";
      std::cout << ss.str();
      std::cout << "Starting (scalar) indexes of vector of factor, by type : \n";
      std::stringstream ss2;
      ss2 << "[ ";
      for (auto idx : Offsets) ss2 << idx << ", ";
      ss2.seekp(-2, std::ios_base::end);
      ss2 << " ]\n";
      std::cout << ss2.str();
#endif

      return Offsets;
    }
  }   // namespace Semantic

  template <typename VECT_WFT_COLLECTION, typename VECT_WMARG_COLLECTION>
  DispatchContainer_t compute_keys_affectation(const VECT_WFT_COLLECTION&   tup_vwf,
                                              const VECT_WMARG_COLLECTION& tup_vwm)
  {
    PROFILE_FUNCTION();
    // result
    DispatchContainer_t resultmap;
    // small optimisation: regroup all these ? really micro
    auto M_type_idx_offsets          = Scalar::FactorTypeIndexesOffset(tup_vwf);
    auto N_type_idx_offsets          = Scalar::MarginalTypeIndexesOffset(tup_vwm);
    auto semantic_M_type_idx_offsets = Semantic::FactorTypeIndexesOffset(tup_vwf);
    auto semantic_N_type_idx_offsets = Semantic::MarginalTypeIndexesOffset(tup_vwm);

    // loop over all the marginals (to define the 'vertices')
    std::apply(
        [&](auto... N_idx_base)
        {
          std::apply(
              [&](auto... semantic_N_idx_base)
              {
                std::apply(
                    [&](const auto&... vwm)
                    {
                      PROFILE_SCOPE("Fill indexes");
                      auto lambda = [&](const auto& a_vwm,
                                        std::size_t scalar_idx_base,
                                        std::size_t semantic_idx_base)
                      {
                        std::size_t j = 0;
                        for (const auto& wm : a_vwm)
                        {
                          constexpr std::size_t kN
                              = std::remove_cvref_t<decltype(wm)>::Marginal_t::KeyMeta_t::kN;
                          KeyDispatchInfos keydispatchinfos;
                          keydispatchinfos.natural_semantic_idx = semantic_idx_base + j;
                          keydispatchinfos.natural_scalar_idx   = scalar_idx_base + j * kN;
                          keydispatchinfos.key_dim              = kN;
                          keydispatchinfos.key_id               = wm.key_id;
                          resultmap.insert(keydispatchinfos);
                          j++;
                        }
                      };
                      (lambda(vwm, N_idx_base, semantic_N_idx_base), ...);
                    },
                    tup_vwm);
              },
              semantic_N_type_idx_offsets);
        },
        N_type_idx_offsets);

    // atm)
    // note: perhaps it would be interesting to add the self key in the set ?(no use case for that
    std::unordered_map<std::string, std::unordered_set<std::string>> tmp_set;
    // loop over all the marginals (to define the 'edges')
    std::apply(
        [&](const auto&... vwf)
        {
          PROFILE_SCOPE("discover neighbours");
          // profiling: PERFORMANCE: (M3500)
          //  going from unordered_map to boost multi_index with <hashed unique>
          //  made the discover neighbours scope to go from 1.185 ms to 2.188 ms  :(
          //  multi index with <ordered_unique>  => 3.160 ms, and most other methods performance were adversly affected (2x)
          ((std::for_each(vwf.begin(),
                          vwf.end(),
                          [&](const auto& wf)
                          {
                            // // static version (is no faster :(  )
                            // std::apply([&](const auto & ... r_kcc)
                            //     {
                            //          auto lambda = [&](const auto & ar_kcc)
                            //          {
                            //             std::apply([&](const auto & ... l_kcc)
                            //              {
                            //                auto lambda2 = [&](const auto & ar_kcc,const auto &
                            //                al_kcc)
                            //                {
                            //                  using r_kcc_t =
                            //                  std::remove_cvref_t<decltype(ar_kcc)>; using l_kcc_t
                            //                  = std::remove_cvref_t<decltype(al_kcc)>; if
                            //                  constexpr (!std::is_same_v<r_kcc_t,l_kcc_t>)
                            //                  {
                            //                    // if user input is correct, no need to check if
                            //                    string are different
                            //                    resultmap[ar_kcc.key_id].neighbours.insert(al_kcc.key_id);
                            //                    resultmap[al_kcc.key_id].neighbours.insert(ar_kcc.key_id);
                            //                  }
                            //                };
                            //                ((lambda2(ar_kcc,l_kcc)),...);
                            //              }, wf.factor.keys_set);
                            //          };
                            //      ((lambda(r_kcc)),...);
                            //     }
                            //     ,wf.factor.keys_set);

                            // slightly faster than static method above
                            auto keys = wf.factor.get_array_keys_id();
                            for (const auto& key_id_of_interest : keys)
                            {
                              // loop over neighbours (including self)
                              for (const auto& other_key_in_factor : keys)
                              {
                                // had the other key in the list of neighbours (unless it is self)
                                if (other_key_in_factor != key_id_of_interest)
                                {
                                  // might double count, but has no effect on the set
                                  auto it_key_id_of_interest_infos = resultmap.find(key_id_of_interest);
                                  // explicit copy (because the elements of boost multi index container are immutable on access)
                                  auto key_id_of_interest_infos_cpy = *it_key_id_of_interest_infos;
                                  auto& neighbours_set = key_id_of_interest_infos_cpy.neighbours;
                                  {
                                    // std::lock_guard<std::mutex> l(lock);
                                    neighbours_set.insert(other_key_in_factor);
                                  }
                                  // replace in the multi index container
                                  resultmap.replace(it_key_id_of_interest_infos, key_id_of_interest_infos_cpy);
                                  // tmp_set[key_id_of_interest].insert(other_key_in_factor);
                                }
                              }
                            }
                          })),
           ...);
        },
        tup_vwf);

    // // merge resultmap with tmp_set
    // auto ittmp = tmp_set.begin();
    // for (auto it = resultmap.begin(); it!=resultmap.end(); it++ )
    // {
    //   it->second.neighbours = ittmp->second;
    //   ittmp++;
    // }

    return resultmap;
  }

  // FIX: template unecessary: but once removed,  put this in a .cpp file to avoid ODR violation
  template <typename KEYDISPATCH_T>
  std::string stringify_key_dispatch_oneliner(const std::string&   key_id,
                                              const KEYDISPATCH_T& key_dispatch,
                                              int                  tab = 4)
  {
    std::stringstream ss;
    ss << std::setw(tab) << "[" << key_id
       << "] : { scalar_idx = " << key_dispatch.natural_scalar_idx
       << " , semantic_idx = " << key_dispatch.natural_semantic_idx << " , neighbours = { ";

    if (key_dispatch.neighbours.empty()) { ss << "none"; }
    else
    {
      for (const auto& neigh_id : key_dispatch.neighbours)   // traverse the set of neighbours id
      {
        ss << neigh_id << " ,";
      }
      ss.seekp(-2, std::ios_base::end);
    }
    ss << " } }";
    return ss.str();
  }

  template <typename KEY_AFFECTATIONS_T>
  std::string stringify_keys_affectation_blockliner(const KEY_AFFECTATIONS_T& keys_affectation,
                                                    int                       tab = 4)
  {
    std::stringstream ss;
    for (const auto& [key_id, key_dispatch] : keys_affectation)
    {
      ss << std::setw(tab) << stringify_key_dispatch_oneliner(key_id, key_dispatch) << '\n';
    }
    return ss.str();
  }

}   // namespace sam::Inference::SystemConverter
