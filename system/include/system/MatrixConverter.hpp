#pragma once

#include <array>
#include <numeric>
#include <tuple>
#include <Eigen/Sparse>
#include <Eigen/Dense>

#include "system/config.h"

// a set of helper functions to translate factor & marginal containers in
// the matrix world (indices, matrice sizes etc..)
// This then makes building a system Ax=b way easier, hidding the complexities 
// from the system level

namespace sam::Inference::MatrixConverter // FIX: inference
{
  // matrix for which one row or column is one scalar dimension (e.g. is a key's dimension is 3, it takes 3 column)
  namespace Scalar
  {
    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t M(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple)
    {
      return std::apply([](const auto & ...vect_of_wf)
          { 
            return  ((std::remove_cvref_t<decltype(vect_of_wf)>::value_type::Factor_t::kM  * 
                  vect_of_wf.size()) + ...); 
          },wfactors_tuple);
    }

    template <typename TUPLE_MAP_WMARGINAL_T>
    std::size_t N(const TUPLE_MAP_WMARGINAL_T& tuple_map_wmarginals)
    {
      return std::apply([](const auto & ...map_of_wmarg)
          { 
            return  ((std::remove_cvref_t<decltype(map_of_wmarg)>::mapped_type::Marginal_t::KeyMeta_t::kN  *
                        map_of_wmarg.size()) + ...); 
          },tuple_map_wmarginals );
    }
    
    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>
     FactorTypeIndexesOffset(const TUPLE_VECTORS_WFACTOR_T & wfactors_tuple)
    {
      using array_t = std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>> ;

      array_t Sizes 
        = std::apply([](const auto & ... vectofwf)
            {
              return array_t
              {
                // size = dimension of factor (measure) * number of factors
                 std::remove_cvref_t<decltype(vectofwf)>::value_type::Factor_t::kM *
                 vectofwf.size() ...
              };
            },wfactors_tuple);

      array_t Offsets ={} ;
      std::partial_sum(Sizes.begin(), Sizes.end()-1, Offsets.begin()+1);

#if ENABLE_DEBUG_TRACE
      std::cout << "Sizes (scalar) of vector of factor, by type: \n";
      std::stringstream ss;
      ss << "[ ";
      for (auto idx :  Sizes  ) ss << idx <<", ";
      ss.seekp(-2,std::ios_base::end); 
      ss<< " ]\n";
      std::cout << ss.str();
      std::cout << "Starting (scalar) indexes of vector of factor, by type : \n";
      std::stringstream ss2;
      ss2 << "[ ";
      for (auto idx :  Offsets  ) ss2 << idx <<", ";
      ss2.seekp(-2,std::ios_base::end); 
      ss2<< " ]\n";
      std::cout << ss2.str();
#endif

      return Offsets;
    }

    template <typename TUPLE_MAP_WMARGINAL_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_MAP_WMARGINAL_T>>
    MarginalTypeIndexesOffset(const TUPLE_MAP_WMARGINAL_T & tuple_map_wmarginals)
    {
      using array_t = std::array<std::size_t, std::tuple_size_v<TUPLE_MAP_WMARGINAL_T>> ;

      array_t Sizes 
        = std::apply([](const auto & ... map_of_wmarg)
            {
              return array_t
              {
                // size = dimension of key * number of keys
                 std::remove_cvref_t<decltype(map_of_wmarg)>::mapped_type::Marginal_t::KeyMeta_t::kN
                 * map_of_wmarg.size() ...
              };
            },tuple_map_wmarginals);

      array_t Offsets ={} ;
      std::partial_sum(Sizes.begin(), Sizes.end()-1, Offsets.begin()+1);

#if ENABLE_DEBUG_TRACE
      std::cout << "Sizes (scalar) of marginal maps, by type: \n";
      std::stringstream ss;
      ss << "[ ";
      for (auto idx :  Sizes  ) ss << idx <<", ";
      ss.seekp(-2,std::ios_base::end); 
      ss<< " ]\n";
      std::cout << ss.str();
      std::cout << "Starting (scalar) indexes of vector of factor, by type : \n";
      std::stringstream ss2;
      ss2 << "[ ";
      for (auto idx :  Offsets  ) ss2 << idx <<", ";
      ss2.seekp(-2,std::ios_base::end); 
      ss2<< " ]\n";
      std::cout << ss2.str();
#endif

      return Offsets;
    }

    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t JacobianNNZ(const TUPLE_VECTORS_WFACTOR_T & wfactors_tuple)
    {
      return std::apply([](const auto & ...vect_of_wf)
                  { 
                    return  ((std::remove_cvref_t<decltype(vect_of_wf)>::value_type::Factor_t::factor_process_matrix_t::SizeAtCompileTime  
                                * vect_of_wf.size()) + ...); 
                  },wfactors_tuple);
    }

    // TODO: more difficult, perhaps easier to expand from Semantic::HessianNNZ() ?
    // std::size_t HessianNNZ()

  }

  // matrix for which one column = one key  (e.g. for A or A^T.A)
  //                  one row  = one factor (e.g. for A)
  // consequently, on a MxN matrix                  
  //                  M is the total number of factors
  //                  N the total number keys
  namespace Semantic
  {
    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t M(const TUPLE_VECTORS_WFACTOR_T& wfactors_tuple)
    {
      return std::apply([](const auto & ...vect_of_wf)
          { 
            return  ( vect_of_wf.size() + ...); 
          },wfactors_tuple);
    }

    template <typename TUPLE_MAP_WMARGINAL_T>
    std::size_t N(const TUPLE_MAP_WMARGINAL_T& tuple_map_wmarginals)
    {
      return std::apply([](const auto & ...map_of_wmarg)
          { 
            return  ( map_of_wmarg.size() + ...); 
          },tuple_map_wmarginals );
    }
    
    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>
     FactorTypeIndexesOffset(const TUPLE_VECTORS_WFACTOR_T & wfactors_tuple)
    {
      using array_t = std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>> ;

      array_t Sizes 
        = std::apply([](const auto & ... vectofwf)
            {
              return array_t { vectofwf.size() ... };
            },wfactors_tuple);

      array_t Offsets ={} ;
      std::partial_sum(Sizes.begin(), Sizes.end()-1, Offsets.begin()+1);

#if ENABLE_DEBUG_TRACE
      std::cout << "Sizes (scalar) of vector of factor, by type: \n";
      std::stringstream ss;
      ss << "[ ";
      for (auto idx :  Sizes  ) ss << idx <<", ";
      ss.seekp(-2,std::ios_base::end); 
      ss<< " ]\n";
      std::cout << ss.str();
      std::cout << "Starting (scalar) indexes of vector of factor, by type : \n";
      std::stringstream ss2;
      ss2 << "[ ";
      for (auto idx :  Offsets  ) ss2 << idx <<", ";
      ss2.seekp(-2,std::ios_base::end); 
      ss2<< " ]\n";
      std::cout << ss2.str();
#endif

      return Offsets;
    }

    template <typename TUPLE_MAP_WMARGINAL_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_MAP_WMARGINAL_T>>
    MarginalTypeIndexesOffset(const TUPLE_MAP_WMARGINAL_T & tuple_map_wmarginals)
    {
      using array_t = std::array<std::size_t, std::tuple_size_v<TUPLE_MAP_WMARGINAL_T>> ;

      array_t Sizes 
        = std::apply([](const auto & ... map_of_wmarg)
            {
              return array_t { map_of_wmarg.size() ... };
            },tuple_map_wmarginals);

      array_t Offsets ={} ;
      std::partial_sum(Sizes.begin(), Sizes.end()-1, Offsets.begin()+1);

#if ENABLE_DEBUG_TRACE
      std::cout << "Sizes (scalar) of marginal maps, by type: \n";
      std::stringstream ss;
      ss << "[ ";
      for (auto idx :  Sizes  ) ss << idx <<", ";
      ss.seekp(-2,std::ios_base::end); 
      ss<< " ]\n";
      std::cout << ss.str();
      std::cout << "Starting (scalar) indexes of vector of factor, by type : \n";
      std::stringstream ss2;
      ss2 << "[ ";
      for (auto idx :  Offsets  ) ss2 << idx <<", ";
      ss2.seekp(-2,std::ios_base::end); 
      ss2<< " ]\n";
      std::cout << ss2.str();
#endif

      return Offsets;
    }

    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::size_t JacobianNNZ(const TUPLE_VECTORS_WFACTOR_T & wfactors_tuple)
    {
      return std::apply([](const auto & ...vect_of_wf)
                  { 
                    return  (  vect_of_wf.size() + ...); 
                  },wfactors_tuple);
    }

    // TODO: more difficult
    // std::size_t HessianNNZ()

  }


  namespace Sparse
  {
    template <typename MAT>
    static void lay_out_Aik_in_triplets(const MAT & Aik
        ,const std::size_t starting_column
        ,const std::size_t starting_row
        ,std::vector<Eigen::Triplet<double>>& Ai_triplets_out)
    {
      constexpr std::size_t M = MAT::RowsAtCompileTime;
      constexpr std::size_t Nk = MAT::ColsAtCompileTime;
      // int offset_cols = sys_col_idx;
      // int offset_rows = line_counter;
      auto spaghetti_Aik = Aik.reshaped(); // make it one dimension
      for (std::size_t i=0; i< MAT::SizeAtCompileTime; i++)
      {
        std::size_t row = starting_row + (i%M);
        std::size_t col = starting_column + (i/M);
        Ai_triplets_out.emplace_back(row,col,spaghetti_Aik[i]);
      }
    }

    template <typename VECT_OF_WFT, typename MARGINAL_COLLECTION_T>
    static void lay_out_factors_to_sparse_triplets
    (
     const VECT_OF_WFT & vect_of_wfactors
     , std::size_t M_FT_idx_offset
     , const std::array<std::size_t, std::tuple_size_v<typename MARGINAL_COLLECTION_T::Marginals_Data_t> > & N_type_idx_offsets
     , const MARGINAL_COLLECTION_T & marginal_collection
     , std::vector<Eigen::Triplet<double>> & sparseA_triplets_out
     , Eigen::VectorXd& b_out
    )
    {
        using WFT = typename VECT_OF_WFT::value_type;
        using FT = typename WFT::Factor_t;
        std::string scope_name = "lay out factors of type " + std::string(FT::kFactorLabel) + " in triplet";
        PROFILE_SCOPE( scope_name.c_str() ,sam_utils::JSONLogger::Instance());

        for (auto it_wf = vect_of_wfactors.begin(); it_wf!=vect_of_wfactors.end(); it_wf++)
        {
          auto factor = it_wf->factor;
          // get Ai and bi (computations of Ai,bi not done here)
          auto matrices_Aik = it_wf->get_current_point_data().Aiks;
          auto bi = it_wf->get_current_point_data().bi;

          // declaring a triplets for matrices_Aik values to be associated with their
          // row/col indexes in view of its future integration into the system matrix A
          std::vector<Eigen::Triplet<double>> Ai_triplets; 
          Ai_triplets.reserve(FT::kN*FT::kM);
          
          // for each element of the tuple:
          //    start_column_idx = keytype_idx_offset + iterator_distance * kN
          std::array<std::size_t, FT::kNbKeys> array_of_start_column_idx = 
            std::apply(
                [&](const auto & ... kcc)
                {
                  // pre declare my expression 
                  auto lambda = [&](const auto & akcc, const auto & marginal_data_tuple) -> std::size_t
                  {
                    using keymeta_t = typename std::remove_cvref_t<decltype(akcc)>::KeyMeta_t;
                    constexpr std::size_t tuple_idx = MARGINAL_COLLECTION_T::template get_correct_tuple_idx<keymeta_t>();
                    auto it = std::get<tuple_idx>(marginal_data_tuple).find(akcc.key_id);
                    std::size_t iterator_distance = std::distance( std::get<tuple_idx>(marginal_data_tuple).begin(), it );
                    return N_type_idx_offsets[tuple_idx] + iterator_distance * keymeta_t::kN;
                  };

                  return std::array<std::size_t, FT::kNbKeys>{ lambda(kcc, marginal_collection.data_map_tuple ) ... };
                }
                , factor.keys_set);

#if ENABLE_DEBUG_TRACE
          std::stringstream ss;
          for (auto e : factor.get_array_keys_id()) ss << e << ", ";
          ss.seekp(-2,std::ios_base::end);  ss << " --\n";
          ss << "Previous [ ";
          for (auto e :  tuple_of_start_column_idx  ) ss << e <<", ";
          ss.seekp(-2,std::ios_base::end); 
          ss<< " ]\n";
          std::cout << ss.str();
          // reset stream
          ss.str(std::string());ss.clear();
          ss << "New [ ";
          for (auto e :  array_of_start_column_idx  ) ss << e <<", ";
          ss.seekp(-2,std::ios_base::end); 
          ss<< " ]\n";
          std::cout << ss.str();
#endif

       

          // start_row_idx = (the idx offset that depends on factor type) + iterator_distance * kM
          std::size_t factor_iterator_distance = std::distance( vect_of_wfactors.begin() , it_wf );
          std::size_t start_row_idx = M_FT_idx_offset + factor_iterator_distance* FT::kM;
          
          // placing those matrices in Ai_triplets
          std::apply(
              [ &start_row_idx,&array_of_start_column_idx, &Ai_triplets](const auto & ...Aik)
              {
                std::apply(
                    [&](auto... start_column_idx)
                    {
                      (
                       (lay_out_Aik_in_triplets(Aik, start_column_idx ,start_row_idx,Ai_triplets))
                       ,...
                      );
                    }
                    ,array_of_start_column_idx);
              }
              , matrices_Aik);
           
          // QUESTION: is that a race condition if we write b at different places concurrently
          b_out.block<FT::kM,1>(start_row_idx,0) = bi;
          // push Ai triplets into sparseA_triplets . WARNING: race condition on sparseA_triplets if parallel policy
          sparseA_triplets_out.insert(std::end(sparseA_triplets_out),std::begin(Ai_triplets),std::end(Ai_triplets));
        }
    }

    template <typename TUPLE_VECTORS_WFACTOR_T, typename MARGINAL_COLLECTION_T>
    static std::tuple< Eigen::VectorXd, Eigen::SparseMatrix<double>> 
    compute_b_A(
        const TUPLE_VECTORS_WFACTOR_T & factor_collection,
        const MARGINAL_COLLECTION_T & marginal_collection
        , std::size_t M 
        , std::size_t N
        , std::size_t jacobian_NNZ
        , const std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>> & M_type_idx_offsets
        , const std::array<std::size_t, std::tuple_size_v<typename MARGINAL_COLLECTION_T::Marginals_Data_t>> & N_type_idx_offsets 
        )
    {
      // declare A, b, and triplets for A data
      Eigen::SparseMatrix<double> A(M,N);
      Eigen::VectorXd b(M);
      std::vector<Eigen::Triplet<double>> sparseA_triplets;
      sparseA_triplets.reserve(jacobian_NNZ); // expected number of nonzeros elements
      //------------------------------------------------------------------//
      //                fill triplets of A and vector of b                //
      //------------------------------------------------------------------//
      std::apply(
          [&](const auto & ...vect_of_wfactors)
          {
            std::apply(
                [&](const auto & ... start_row_idx)
                {
                    (
                     ( lay_out_factors_to_sparse_triplets
                        <std::remove_cvref_t<decltype(vect_of_wfactors)>,MARGINAL_COLLECTION_T>
                       (
                                                                vect_of_wfactors
                                                                , start_row_idx
                                                                , N_type_idx_offsets
                                                                , marginal_collection
                                                                ,sparseA_triplets
                                                                ,b
                                                               ) 
                     )
                     , ... );
                }, M_type_idx_offsets);
          } ,factor_collection);

      // set A from triplets, clear the triplets
      A.setFromTriplets(sparseA_triplets.begin(), sparseA_triplets.end());
      return {b,A};
    }

  }

  namespace Dense
  {

  }


}   // namespace sam::MatrixConverter
