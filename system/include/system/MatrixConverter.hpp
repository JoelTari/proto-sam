#pragma once

#include <array>
#include <numeric>
#include <tuple>

#include "system/config.h"

// a set of helper functions to translate factor & marginal containers in
// the matrix world (indices, matrice sizes etc..)
// This then makes building a system Ax=b way easier, hidding the complexities 
// from the system level

namespace sam::System::MatrixConverter
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
    
    // TODO: nnz

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

// #if ENABLE_DEBUG_TRACE
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
// #endif

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

// #if ENABLE_DEBUG_TRACE
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
// #endif

      return Offsets;
    }

    // TODO:
    // template <typename TUPLE_VECTORS_WFACTOR_T>
    // std::size_t JacobianNNZ(this->all_factor_tuple_)
    // {
    // }

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
    }

    template <typename TUPLE_MAP_WMARGINAL_T>
    std::size_t N(const TUPLE_MAP_WMARGINAL_T& tuple_map_wmarginals)
    {
      
    }

    template <typename TUPLE_VECTORS_WFACTOR_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_VECTORS_WFACTOR_T>>
     FactorTypeIndexesOffset(const TUPLE_VECTORS_WFACTOR_T & wfactors_tuple)
    {

    }

    template <typename TUPLE_MAP_WMARGINAL_T>
    std::array<std::size_t, std::tuple_size_v<TUPLE_MAP_WMARGINAL_T>>
    MarginalTypeIndexesOffset(const TUPLE_MAP_WMARGINAL_T & tuple_map_wmarginals)
    {

    }
  }


}   // namespace sam::MatrixConverter
