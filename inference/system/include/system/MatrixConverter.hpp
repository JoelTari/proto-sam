#pragma once

#include "system/SystemConverter.hpp"
#include "utils/config.h"
#include "utils/utils.h"

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
    std::size_t
        HessianNNZ(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple
                   // , const typename SystemConverter::KeyDispatchInfos & key_dispatcch_info)
        )
    {
      std::unordered_map<std::string, std::unordered_set<std::string>> edges_from {};
      // loop factors, add to the edges
      std::size_t nnz = 0;
      std::apply(
          [&edges_from, &nnz](const auto&... vect_of_wf)
          {
            ((std::for_each(
                 vect_of_wf.begin(),
                 vect_of_wf.end(),
                 [&edges_from, &nnz](const auto& wf)
                 {
                   // The challenge is to avoid double count.
                   // In the hessian a case can be the contribution of several factors:
                   // for instance factors \phi(x0) and \phi(x0,x1) both contributes
                   // to
                   // So the nnz is not simply found by looping naively the factor, we have
                   // to check if an area is already counted. This is done through the
                   // variable edges_from that is a map of sets (key: a variable, set: the
                   // neighbours variables) The return value of insertion in the set signals that a

                   // for each factor
                   //    for each kcc
                   //        - insert in map[kcc.id,{}]
                   //       for each other_kcc
                   //          - insert in set: [kcc.id, [other_kcc] ]
                   //          - if insertion took place
                   //             then nnz +=  kcc::kN*other_kcc::kN

                   std::apply(
                       [&](const auto&... kcc)
                       {
                         // expression definition
                         auto lambda = [&](const auto& akcc)
                         {
                           edges_from[akcc.key_id];
                           std::apply(
                               [&](const auto&... other_kcc)
                               {
                                 // expression definition
                                 auto lambda2 = [&](const auto& another_kcc)
                                 {
                                   auto [it, insertionTookPlace]
                                       = edges_from[akcc.key_id].insert(another_kcc.key_id);
                                   if (insertionTookPlace)
                                   {
                                     nnz += std::remove_cvref_t<decltype(akcc)>::KeyMeta_t::kN
                                            * std::remove_cvref_t<
                                                decltype(another_kcc)>::KeyMeta_t::kN;
                                   }
                                 };
                                 // expression application with expansion
                                 (lambda2(other_kcc), ...);
                               },
                               wf.factor.keys_set);
                         };
                         // expression application with expansion
                         (lambda(kcc), ...);
                       },
                       wf.factor.keys_set);
                 })),
             ...);
          },
          wfactors_tuple);
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
      return std::apply(
          [](const auto&... vect_of_wf)
          {
            return ((std::remove_cvref_t<decltype(vect_of_wf)>::value_type::Factor_t::kNbKeys
                     * vect_of_wf.size())
                    + ...);
          },
          wfactors_tuple);
    }

    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t HessianNNZ(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple)
    {
      // std::vector<Eigen::Triplet<double>> semantic_triplet_A;
      std::unordered_map<std::string, std::unordered_set<std::string>> edges_from {};
      // loop factors, add to the edges
      std::apply(
          [&edges_from](const auto&... vect_of_wf)
          {
            ((std::for_each(vect_of_wf.begin(),
                            vect_of_wf.end(),
                            [&edges_from](const auto& wf)
                            {
                              auto keys_id = wf.factor.get_array_keys_id();
                              for (const auto& key_id : keys_id)
                              {
                                auto& mapped_key_neigbors = edges_from[key_id];
                                mapped_key_neigbors.insert(std::begin(keys_id), std::end(keys_id));
                              }
                            })),
             ...);
          },
          wfactors_tuple);

      // accumulate
      std::size_t nnz
          = std::accumulate(std::begin(edges_from),
                            std::end(edges_from),
                            0,
                            [](int value, const auto& pair) { return value + pair.second.size(); });
      return nnz;
    }
  }   // namespace Semantic


  namespace Sparse
  {
    template <typename MAT>
    void emplace_matrix_in_triplets(const MAT&                           A,
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
    void lay_out_factors_to_sparse_triplets(
        const VECT_OF_WFT&                   vect_of_wfactors,
        std::size_t                          M_FT_idx_offset,
        const Keys_Affectation_t&            keys_affectation,
        std::vector<Eigen::Triplet<double>>& sparseA_triplets_out,
        Eigen::VectorXd&                     b_out)
    {
      using WFT = typename VECT_OF_WFT::value_type;
      using FT  = typename WFT::Factor_t;
      std::string scope_name
          = "lay out factors of type " + std::string(FT::kFactorLabel) + " in triplet";
      PROFILE_SCOPE(scope_name.c_str());

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
            { return {keys_affectation.find(kcc.key_id)->second.natural_scalar_idx...}; },
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
                    ((emplace_matrix_in_triplets(Aik,
                                                 start_column_idx,
                                                 start_row_idx,
                                                 Ai_triplets)),
                     ...);
                  },
                  array_of_start_column_idx);
            },
            matrices_Aik);

        // QUESTION: is that a race condition if we write b at different places concurrently
        b_out.block<FT::kM, 1>(start_row_idx, 0) = bi;
        // push Ai triplets into sparseA_triplets 
        sparseA_triplets_out.insert(std::end(sparseA_triplets_out),
                                    std::begin(Ai_triplets),
                                    std::end(Ai_triplets));
      }
    }

    template <typename TUPLE_VECTORS_WFACTOR_T,
              typename TUPLE_VECTORS_WMARGINALS_T>
    std::tuple<Eigen::VectorXd, Eigen::SparseMatrix<double>>
        compute_b_A(const TUPLE_VECTORS_WFACTOR_T&    factor_collection,
                    const TUPLE_VECTORS_WMARGINALS_T& vectors_of_wmarginals,
                    const Keys_Affectation_t&         keys_affectation,
                    std::size_t                       M,
                    std::size_t                       N,
                    std::size_t                       scalar_jacobian_NNZ,
                    const std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>&
                        natural_scalar_M_offsets)
    {
      PROFILE_FUNCTION();
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
                  ((lay_out_factors_to_sparse_triplets<
                       std::remove_cvref_t<decltype(vect_of_wfactors)>>(vect_of_wfactors,
                                                                        start_row_idx,
                                                                        keys_affectation,
                                                                        sparseA_triplets,
                                                                        b)),
                   ...);
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
      template <typename TUPLE_VECTORS_WFACTOR_T> 
      Eigen::SparseMatrix<int> spyJacobian(
          const TUPLE_VECTORS_WFACTOR_T& factor_collection,
          const Keys_Affectation_t& keys_affectation,
          const std::size_t semantic_M,
          const std::size_t semantic_N,
          const std::size_t semantic_jacobian_NNZ,
          const std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>& M_semantic_type_idx_offsets)
      {
        PROFILE_FUNCTION();
        // declare A, b, and triplets for A data
        Eigen::SparseMatrix<int>         spy_matrix_jacobian(semantic_M, semantic_N);
        std::vector<Eigen::Triplet<int>> sparse_semantic_A_triplets;
        sparse_semantic_A_triplets.reserve(
            semantic_jacobian_NNZ);   // expected number of nonzeros elements

        //
        std::apply(
            [&](const auto&... vect_of_wfactors)
            {
              std::apply(
                  [&](const auto&... type_row_idx_offset)
                  {
                    ((std::for_each(vect_of_wfactors.begin(),
                                    vect_of_wfactors.end(),
                                    [&](const auto& wf)
                                    {
                                      for (const auto& key_id : wf.factor.get_array_keys_id())
                                      {
                                        if (auto it = keys_affectation.find(key_id);
                                            it != keys_affectation.end())
                                        {
                                          const auto& key_id_dispatch {it->second};
                                          sparse_semantic_A_triplets.emplace_back(
                                              type_row_idx_offset,
                                              key_id_dispatch.natural_semantic_idx,
                                              1);
                                        }
                                      }
                                    })),
                     ...);
                  },
                  M_semantic_type_idx_offsets);
            },
            factor_collection);

        // set A from triplets, clear the triplets
        spy_matrix_jacobian.setFromTriplets(sparse_semantic_A_triplets.begin(),
                                            sparse_semantic_A_triplets.end());
        return spy_matrix_jacobian;
      }


      // spy of the Hessian (based of spyJacobian)
      template <typename TUPLE_VECTORS_WFACTOR_T> 
      Eigen::SparseMatrix<int> spyHessian(
          const TUPLE_VECTORS_WFACTOR_T& factor_collection,
          const Keys_Affectation_t& keys_affectation,
          const std::size_t semantic_M,
          const std::size_t semantic_N,
          const std::size_t semantic_jacobian_NNZ,
          const std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>& M_semantic_type_idx_offsets)
      {
          auto A = spyJacobian(factor_collection, keys_affectation,semantic_M,semantic_N,semantic_jacobian_NNZ,M_semantic_type_idx_offsets);
          return spyHessian(A);
      }

      Eigen::SparseMatrix<int> spyHessian(const Eigen::SparseMatrix<int> & spy_jacobian);

      Eigen::SparseMatrix<int> spyHessian(const Keys_Affectation_t & keys_affectation, const std::size_t nnz = 0);

    }   // namespace Semantic


  }   // namespace Sparse

  namespace Dense
  {
    template <typename TUPLE_VECTORS_WFACTOR_T,
              typename TUPLE_VECTORS_WMARGINALS_T>
    std::tuple<Eigen::VectorXd, Eigen::SparseMatrix<double>>
        compute_b_A(const TUPLE_VECTORS_WFACTOR_T&    factor_collection,
                    const TUPLE_VECTORS_WMARGINALS_T& vectors_of_wmarginals,
                    const Keys_Affectation_t&         keys_affectation,
                    std::size_t                       M,
                    std::size_t                       N,
                    const std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>&
                        natural_scalar_M_offsets)
    {
      // FIX: urgent continue here
    }

  }


}   // namespace sam::Inference::MatrixConverter
