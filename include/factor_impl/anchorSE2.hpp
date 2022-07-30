#ifndef FACTOR_IMPL_ANCHOR_SE2_H_
#define FACTOR_IMPL_ANCHOR_SE2_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-SE2.h"
#include "factor_impl/measure-meta-absolute-pose-SE2.h"

namespace __UniqueSE2KeyConduct
{
  inline static constexpr const char anchor_role[] = "unique var";

  
  struct UniqueSE2KeyConduct_t 
    : KeyContextualConduct
      <UniqueSE2KeyConduct_t,MetaKeyPose_SE2_t,MetaMeasureAbsolutePoseSE2_t,anchor_role>
  {
    using BaseKeyCC_t = KeyContextualConduct
      <UniqueSE2KeyConduct_t,MetaKeyPose_SE2_t,MetaMeasureAbsolutePoseSE2_t,anchor_role>;
    using key_process_matrix_t = typename BaseKeyCC_t::key_process_matrix_t;

    key_process_matrix_t compute_Aik_at(const Key_t & Xk) const
    {
      key_process_matrix_t Ai;
      return Ai;
    }
    // the ctors
    UniqueSE2KeyConduct_t(const std::string& key_id, const measure_cov_t& rho) : BaseKeyCC_t(key_id, rho)
    {}
    UniqueSE2KeyConduct_t(const std::string& key_id, const measure_cov_t& rho, std::shared_ptr<Key_t> init_point_view) : BaseKeyCC_t(key_id, rho, init_point_view)
    {}
  };
}   // namespace
using UniqueSE2KeyConduct_t = __UniqueSE2KeyConduct::UniqueSE2KeyConduct_t;

namespace __AnchorSE2Factor
{
  inline static constexpr const char anchorLabel[] = "anchor SE2";
  class AnchorSE2Factor
      : public BaseFactor<AnchorSE2Factor, anchorLabel,MetaMeasureAbsolutePoseSE2_t , UniqueSE2KeyConduct_t>
  {
    public:
    using BaseFactor_t = BaseFactor<AnchorSE2Factor, anchorLabel,MetaMeasureAbsolutePoseSE2_t , UniqueSE2KeyConduct_t>;
    friend BaseFactor_t;
    static_assert(std::is_same_v<UniqueSE2KeyConduct_t::key_process_matrix_t, factor_process_matrix_t>  ); // because only 1 key

    AnchorSE2Factor(const std::string&                                    factor_id,
                 const measure_t &                                 mes_vect,
                 const measure_cov_t&                                  measure_cov,
                 const std::array<std::string, kNbKeys>& keys_id,
                 const composite_state_ptr_t & tuple_of_init_point_ptrs)
        : BaseFactor_t(factor_id, mes_vect, measure_cov, keys_id, tuple_of_init_point_ptrs)
    {
#if ENABLE_DEBUG_TRACE
      std::cout << "\t::  Factor " << factor_id << " (AnchorSE2Factor) created.\n";
#endif
    }
    static_assert( std::is_same_v<measure_t, UniqueSE2KeyConduct_t::Key_t> , "measure and key types should be the same for anchor SE2 factor.");

    // init point guesser
    /**
     * @brief guess initial point: this function is trivial as the initial point could be the measure...
     *
     * @param x_init_ptr_optional_tup  already avaible tuple (single element tuple...) of optional init point (nullopt if not available)
     * @param z measure (an SE2 element)
     */
    static std::optional< composite_state_ptr_t >
        guess_init_key_points_impl( const composite_of_opt_state_ptr_t &
                                  x_init_ptr_optional_tup,
            const measure_t& z)
    {
      if (std::get<0>(x_init_ptr_optional_tup).has_value())
        // if the optional mean value inside the tuple is given, we report this value as initial
        // guess
        return std::make_tuple(std::get<0>(x_init_ptr_optional_tup).value());
      else   // make a new state in the heap from the measurement
      {
        // static_assert( std::is_same_v<decltype(z), UniqueSE2KeyConduct_t::Key_t> ,"measure and key_t not the same." );
        std::shared_ptr<typename UniqueSE2KeyConduct_t::Key_t> xinit_ptr = std::make_shared<typename UniqueSE2KeyConduct_t::Key_t>(z);
        return std::make_tuple(xinit_ptr);
      }
      // NOTE: it never returns std::nullopt, that's normal in this situation
    }
    
    std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_at_impl(const composite_state_ptr_t & X) const
    {
      // In this factor we use the r-minus convention

      // extract Xk (X is a tuple of 1 element...)
      auto Xk = *std::get<0>(X);
      // pre declaring the jacobian
      using Aik_t = std::tuple_element_t<0, matrices_Aik_t>;
      Aik_t J_ZrmX_X;
      // compute bi = -  r(X) = - rho * ( Z (r-) X )
      // and fill the Jacobian
      criterion_t bi = -this->rho*this->z.rminus(Xk, {}, J_ZrmX_X).coeffs();
      // compute tuple of the Aiks (just one in this factor)
      Aik_t Aik = this->rho * J_ZrmX_X;
      // 
      return {bi, {Aik}};
    }

    criterion_t compute_r_of_x_at_impl(const composite_state_ptr_t & X) const
    {
      auto Xk = *std::get<0>(X);
      return this->rho* this->z.rminus(Xk).coeffs();
    }


  };
}   // namespace
using AnchorSE2Factor = __AnchorSE2Factor::AnchorSE2Factor;


#endif
