#pragma once

#include "utils/config.h"
#include "utils/utils.h"
#include "system/SystemConverter.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>
#include <numeric>
#include <tuple>
#include <unordered_set>

// a set of helper functions to translate factor & marginal containers in
// the matrix world (indices, matrice sizes etc..)
// This then makes building a system Ax=b way easier, hidding the complexities
// from the system level

namespace sam::Inference::MatrixConverter
{
  using Keys_Affectation_t = typename SystemConverter::Keys_Affectation_t;
  // matrix for which one row or column is one scalar dimension (e.g. is a key's dimension is 3, it
  // takes 3 column)
  namespace Scalar
  {
    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t JacobianNNZ(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple)
    {
      return std::apply(
          [](const auto&... vect_of_wf)
          {
            return ((std::remove_cvref_t<decltype(vect_of_wf)>::value_type::Factor_t::
                         factor_process_matrix_t::SizeAtCompileTime
                     * vect_of_wf.size())
                    + ...);
          },
          wfactors_tuple);
    }

    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t HessianNNZ(const TUPLE_VECTORS_WFACTOR_T & wfactors_tuple
        // , const typename SystemConverter::KeyDispatchInfos & key_dispatcch_info)
    )
    {
      std::unordered_map<std::string, std::unordered_set<std::string>> edges_from{};
      // loop factors, add to the edges
      std::size_t nnz =0;
      std::apply([&edges_from,&nnz](const auto & ...vect_of_wf)
          {
          ((
            std::for_each(
              vect_of_wf.begin(),
              vect_of_wf.end(),
              [&edges_from,&nnz](const auto & wf)
              { 
                // The challenge is to avoid double count.
                // In the hessian a case can be the contribution of several factors:
                // for instance factors \phi(x0) and \phi(x0,x1) both contributes
                // to 
                // So the nnz is not simply found by looping naively the factor, we have 
                // to check if an area is already counted. This is done through the 
                // variable edges_from that is a map of sets (key: a variable, set: the neighbours variables)
                // The return value of insertion in the set signals that a 

                // for each factor
                //    for each kcc
                //        - insert in map[kcc.id,{}] 
                //       for each other_kcc
                //          - insert in set: [kcc.id, [other_kcc] ]
                //          - if insertion took place
                //             then nnz +=  kcc::kN*other_kcc::kN

                std::apply(
                    [&](const auto & ...kcc)
                    {
                      // expression definition
                      auto lambda = [&](const auto & akcc)
                      {
                        edges_from[akcc.key_id];
                        std::apply(
                            [&](const auto & ...other_kcc)
                            {
                              // expression definition
                              auto lambda2 = [&](const auto & another_kcc){
                                  auto [it, insertionTookPlace] = edges_from[akcc.key_id].insert(another_kcc.key_id);
                                  if (insertionTookPlace)
                                  {
                                    nnz+= std::remove_cvref_t<decltype(akcc)>::KeyMeta_t::kN * std::remove_cvref_t<decltype(another_kcc)>::KeyMeta_t::kN;
                                  }
                              };
                              // expression application with expansion
                              (lambda2(other_kcc), ...);
                            },wf.factor.keys_set );
                      };
                      // expression application with expansion
                      (lambda(kcc), ...);
                    },wf.factor.keys_set);
              })
           ),...);
          }
          ,wfactors_tuple);
      return nnz;
    }

  }   // namespace Scalar

  // matrix for which one column = one key  (e.g. for A or A^T.A)
  //                  one row  = one factor (e.g. for A)
  // consequently, on a MxN matrix
  //                  M is the total number of factors
  //                  N the total number keys
  namespace Semantic
  {
    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t JacobianNNZ(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple)
    {
      return std::apply([](const auto&... vect_of_wf) { 
          return ((std::remove_cvref_t<decltype(vect_of_wf)>::value_type::Factor_t::kNbKeys*
              vect_of_wf.size()) + ...); },
                        wfactors_tuple);
    }

    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t HessianNNZ(const TUPLE_VECTORS_WFACTOR_T & wfactors_tuple)
    {
      // std::vector<Eigen::Triplet<double>> semantic_triplet_A;
      std::unordered_map<std::string, std::unordered_set<std::string>> edges_from{};
      // loop factors, add to the edges
      std::apply([&edges_from](const auto & ...vect_of_wf)
          {
            ((
            std::for_each(
                vect_of_wf.begin(),
                vect_of_wf.end(),
                [&edges_from](const auto & wf)
                {
                  auto keys_id = wf.factor.get_array_keys_id();
                  for (const auto & key_id : keys_id)
                  {
                    auto & mapped_key_neigbors = edges_from[key_id];
                    mapped_key_neigbors.insert( std::begin(keys_id), std::end(keys_id));
                  }
                })
            ),...);
          }
          ,wfactors_tuple);

      // accumulate
      std::size_t nnz = std::accumulate(std::begin(edges_from)
                    , std::end(edges_from)
                    , 0
                    , [] (int value, const auto & pair)
                         { return value + pair.second.size(); }
                     );
      return nnz;
    }
  }   // namespace Semantic


  namespace Sparse
  {
    template <typename MAT>
    static void emplace_matrix_in_triplets(const MAT&                           A,
                                        const std::size_t                    starting_column,
                                        const std::size_t                    starting_row,
                                        std::vector<Eigen::Triplet<double>>& triplets_out)
    {
      constexpr std::size_t M  = MAT::RowsAtCompileTime;
      constexpr std::size_t Nk = MAT::ColsAtCompileTime;
      // int offset_cols = sys_col_idx;
      // int offset_rows = line_counter;
      auto A_flattened = A.reshaped();   // make it one dimension
      for (std::size_t i = 0; i < MAT::SizeAtCompileTime; i++)
      {
        std::size_t row = starting_row + (i % M);
        std::size_t col = starting_column + (i / M);
        triplets_out.emplace_back(row, col, A_flattened[i]);
      }
    }

    template <typename VECT_OF_WFT>
    static void lay_out_factors_to_sparse_triplets(
        const VECT_OF_WFT& vect_of_wfactors,
        std::size_t        M_FT_idx_offset,
        const Keys_Affectation_t & keys_affectation,
        std::vector<Eigen::Triplet<double>>& sparseA_triplets_out,
        Eigen::VectorXd&                     b_out)
    {
      using WFT = typename VECT_OF_WFT::value_type;
      using FT  = typename WFT::Factor_t;
      std::string scope_name
          = "lay out factors of type " + std::string(FT::kFactorLabel) + " in triplet";
      PROFILE_SCOPE(scope_name.c_str(), sam_utils::JSONLogger::Instance());

      for (auto it_wf = vect_of_wfactors.begin(); it_wf != vect_of_wfactors.end(); it_wf++)
      {
        auto factor = it_wf->factor;
        // get Ai and bi (computations of Ai,bi not done here)
        auto matrices_Aik = it_wf->get_current_point_data().Aiks;
        auto bi           = it_wf->get_current_point_data().bi;

        // declaring a triplets for matrices_Aik values to be associated with their
        // row/col indexes in view of its future integration into the system matrix A
        std::vector<Eigen::Triplet<double>> Ai_triplets;
        Ai_triplets.reserve(FT::kN * FT::kM);

        // for each element of the tuple:
        //    start_column_idx = keytype_idx_offset + iterator_distance * kN
        std::array<std::size_t, FT::kNbKeys> array_of_start_column_idx = std::apply(
            [&](const auto&... kcc) -> std::array<std::size_t, FT::kNbKeys>
            {
              return { keys_affectation.find(kcc.key_id)->second.natural_scalar_idx ... };
            },
            factor.keys_set);

#if ENABLE_DEBUG_TRACE
        std::stringstream ss;
        for (auto e : factor.get_array_keys_id()) ss << e << ", ";
        ss.seekp(-2, std::ios_base::end);
        ss << " --\n";
        ss << "Previous [ ";
        for (auto e : tuple_of_start_column_idx) ss << e << ", ";
        ss.seekp(-2, std::ios_base::end);
        ss << " ]\n";
        std::cout << ss.str();
        // reset stream
        ss.str(std::string());
        ss.clear();
        ss << "New [ ";
        for (auto e : array_of_start_column_idx) ss << e << ", ";
        ss.seekp(-2, std::ios_base::end);
        ss << " ]\n";
        std::cout << ss.str();
#endif


        // start_row_idx = (the idx offset that depends on factor type) + iterator_distance * kM
        std::size_t factor_iterator_distance = std::distance(vect_of_wfactors.begin(), it_wf);
        std::size_t start_row_idx            = M_FT_idx_offset + factor_iterator_distance * FT::kM;

        // placing those matrices in Ai_triplets
        std::apply(
            [&start_row_idx, &array_of_start_column_idx, &Ai_triplets](const auto&... Aik)
            {
              std::apply(
                  [&](auto... start_column_idx) {
                    ((emplace_matrix_in_triplets(Aik, start_column_idx, start_row_idx, Ai_triplets)),
                     ...);
                  },
                  array_of_start_column_idx);
            },
            matrices_Aik);

        // QUESTION: is that a race condition if we write b at different places concurrently
        b_out.block<FT::kM, 1>(start_row_idx, 0) = bi;
        // push Ai triplets into sparseA_triplets . WARNING: race condition on sparseA_triplets if
        // parallel policy
        sparseA_triplets_out.insert(std::end(sparseA_triplets_out),
                                    std::begin(Ai_triplets),
                                    std::end(Ai_triplets));
      }
    }

    template <typename TUPLE_VECTORS_WFACTOR_T, typename TUPLE_VECTORS_WMARGINALS_T> // WARNING: marginal refactor: map -> vector
    static std::tuple<Eigen::VectorXd, Eigen::SparseMatrix<double>> compute_b_A(
        const TUPLE_VECTORS_WFACTOR_T& factor_collection,
        const TUPLE_VECTORS_WMARGINALS_T&   vectors_of_wmarginals,
        const Keys_Affectation_t & keys_affectation,
        std::size_t                    M,
        std::size_t                    N,
        std::size_t                    scalar_jacobian_NNZ,
        const std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>&
            natural_scalar_M_offsets)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      // declare A, b, and triplets for A data
      Eigen::SparseMatrix<double>         A(M, N);
      Eigen::VectorXd                     b(M);
      std::vector<Eigen::Triplet<double>> sparseA_triplets;
      sparseA_triplets.reserve(scalar_jacobian_NNZ);   // expected number of nonzeros elements

      // 
      std::apply(
          [&](const auto&... vect_of_wfactors)
          {
            std::apply(
                [&](const auto&... start_row_idx)
                {
                  ((lay_out_factors_to_sparse_triplets<std::remove_cvref_t<decltype(vect_of_wfactors)>>
                                             (vect_of_wfactors,
                                              start_row_idx,
                                              keys_affectation,
                                              sparseA_triplets,
                                              b)), ...);
                },
                natural_scalar_M_offsets);
          },
          factor_collection);

      // set A from triplets, clear the triplets
      A.setFromTriplets(sparseA_triplets.begin(), sparseA_triplets.end());
      return {b, A};
    }

    namespace Semantic
    {
      template <typename TUPLE_VECTORS_WFACTOR_T> // WARNING: marginal refactor: map -> vector
      static Eigen::SparseMatrix<int> spyJacobian( const TUPLE_VECTORS_WFACTOR_T& factor_collection,
                                                   const std::unordered_map<std::string,typename SystemConverter::KeyDispatchInfos>& keys_affectation,
                                                   std::size_t                    semantic_M,
                                                   std::size_t                    semantic_N,
                                                   std::size_t                    semantic_jacobian_NNZ,
                                                   const std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>& M_semantic_type_idx_offsets)
      {
        PROFILE_FUNCTION( sam_utils::JSONLogger::Instance());
        // declare A, b, and triplets for A data
        Eigen::SparseMatrix<int>         spy_matrix_jacobian(semantic_M, semantic_N);
        std::vector<Eigen::Triplet<int>> sparse_semantic_A_triplets;
        sparse_semantic_A_triplets.reserve(semantic_jacobian_NNZ);   // expected number of nonzeros elements

        // 
        std::apply(
            [&](const auto&... vect_of_wfactors)
            {
              std::apply(
                  [&](const auto&... type_row_idx_offset)
                  {
                    // auto lambda = [&](const auto & vwf, auto row_offset)
                    // {
                    //   for (auto it_wf = vwf.begin(); it_wf != vwf.end(); it_wf++)
                    //   {
                    //     using VECT_OF_WFT = std::remove_cvref_t<decltype(vwf)>;
                    //     using WFT = typename VECT_OF_WFT::value_type;
                    //     using FT  = typename WFT::Factor_t;
                    //     auto factor = it_wf->factor;
                    //     std::array<std::size_t, FT::kNbKeys> array_of_column_idx = 
                    //                 std::apply(
                    //                     [&](const auto & ...kcc)
                    //                     {
                    //                           // pre declare my expression
                    //                           auto lambda2 = [&](const auto& akcc, const auto& marginal_data_tuple) -> std::size_t
                    //                           {
                    //                             using keymeta_t = typename std::remove_cvref_t<decltype(akcc)>::KeyMeta_t;
                    //                             constexpr std::size_t tuple_idx
                    //                                 = MARGINAL_COLLECTION_T::template get_correct_tuple_idx<keymeta_t>();
                    //                             auto        it = std::get<tuple_idx>(marginal_data_tuple).find(akcc.key_id);   // WARNING: mark marginal.find usage (marginal vectorisation). Use map of keydispatch instead.
                    //                             std::size_t iterator_distance
                    //                                 = std::distance(std::get<tuple_idx>(marginal_data_tuple).begin(), it); // WARNING: linear cost // WARNING: marginal refactor: map -> vector
                    //                             return N_semantic_type_idx_offsets[tuple_idx] + iterator_distance;
                    //                           };
                    //                           return std::array<std::size_t, FT::kNbKeys> { lambda2(kcc, marginal_collection.data_map_tuple)...};
                    //                     }
                    //                     ,factor.keys_set);
                    //     std::size_t row_idx = row_offset + std::distance( vwf.begin() ,it_wf );
                    //
                    //     for (auto col_idx: array_of_column_idx)
                    //     {
                    //         sparse_semantic_A_triplets.emplace_back(row_idx,col_idx,1);
                    //     }
                    //   }
                    // };
                    // (lambda(vect_of_wfactors, type_row_idx_offset) ,...);
                    // compet
                    // --
                    // for each wf,
                    //  for each kcc in wf.keyset
                    //    emplace_back( row_idx, keys_affectations[kcc.key].natural_semantic_idx , 1 );
                    // --
                    // end compet
                    (
                     (
                      std::for_each(vect_of_wfactors.begin(),vect_of_wfactors.end(),
                        [&](const auto& wf)
                        {
                          for (const auto & key_id : wf.factor.get_array_keys_id())
                          {
                            if (auto it = keys_affectation.find(key_id); it != keys_affectation.end() )
                            {
                              const auto &key_id_dispatch {it->second};
                              sparse_semantic_A_triplets.emplace_back(type_row_idx_offset,key_id_dispatch.natural_semantic_idx,1 );
                            }
                          }
                          // std::apply([&](const auto &...kcc)
                          //   {
                          //     (
                          //      ( sparse_semantic_A_triplets.emplace_back(type_row_idx_offset,keys_affectation[kcc.key_id].natural_semantic_idx,1 ) )
                          //      ,...);
                          //   },wf.factor.keys_set);
                        })
                     )
                     ,...);
                  },
                  M_semantic_type_idx_offsets);
            },
            factor_collection);

        // set A from triplets, clear the triplets
        spy_matrix_jacobian.setFromTriplets(sparse_semantic_A_triplets.begin(), sparse_semantic_A_triplets.end());
        return spy_matrix_jacobian;
      }

      
      // necessary for graph systems (we might do ordering of the graph this structure)
      // Eigen::Sparse<int> spyHessian()
      // {
      //     return A.t A;
      // }

    }


  }   // namespace Sparse

  namespace Dense
  {

  }


}   // namespace sam::Inference::MatrixConverter
