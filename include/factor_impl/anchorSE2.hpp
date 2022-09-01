#ifndef FACTOR_IMPL_ANCHOR_SE2_H_
#define FACTOR_IMPL_ANCHOR_SE2_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-SE2.h"
#include "factor_impl/measure-meta-absolute-pose-SE2.h"



namespace details_sam::Factor{
  namespace AnchorSE2Impl{
    inline static constexpr const char anchor_role_str[] = "anchored";

    using namespace ::sam::Meta;
    using namespace ::details_sam::Conduct;

    struct PriorSE2KeyConduct 
      : KeyContextualConduct
        <PriorSE2KeyConduct,Key::PoseSE2,Measure::AbsolutePoseSE2,anchor_role_str>
    {
      using BaseKeyCC_t = KeyContextualConduct
        <PriorSE2KeyConduct,Key::PoseSE2,Measure::AbsolutePoseSE2,anchor_role_str>;
      // ctor inherited
      using BaseKeyCC_t::BaseKeyCC_t;
      using key_process_matrix_t = typename BaseKeyCC_t::key_process_matrix_t;

      key_process_matrix_t compute_Hik_at_impl(const Key_t & Xk) const
      {
        // non sequitur
        key_process_matrix_t Hik;
        // FIX: add the computations here anyway.
        throw std::runtime_error("not implemented yet");
        return Hik;
      }
      // the ctors
      // PriorSE2KeyConduct(const std::string& key_id) : BaseKeyCC_t(key_id)
      // {}
    };

    inline static constexpr const char anchor_label[] = "anchor SE2";

    namespace exports{

      class AnchorSE2 :
        public sam::Factor::BaseFactor<AnchorSE2, anchor_label, PriorSE2KeyConduct>
      {

          public:
          using BaseFactor_t = BaseFactor<AnchorSE2, anchor_label, PriorSE2KeyConduct >;
          friend BaseFactor_t;
          static_assert(std::is_same_v<PriorSE2KeyConduct::key_process_matrix_t, factor_process_matrix_t>  ); // because only 1 key

          AnchorSE2(const std::string&                                    factor_id,
                       const measure_t &                                 mes_vect,
                       const measure_cov_t&                                  measure_cov,
                       const std::array<std::string, kNbKeys>& keys_id)
              : BaseFactor_t(factor_id, mes_vect, measure_cov, keys_id)
          {
          }

          static_assert( std::is_same_v<measure_t, PriorSE2KeyConduct::Key_t> , "measure and key types should be the same for anchor SE2 factor.");

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
              // static_assert( std::is_same_v<decltype(z), PriorSE2KeyConduct_t::Key_t> ,"measure and key_t not the same." );
              std::shared_ptr<typename PriorSE2KeyConduct::Key_t> xinit_ptr = std::make_shared<typename PriorSE2KeyConduct::Key_t>(z);
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

    }
  }
}

namespace sam::Factor{
  using namespace details_sam::Factor::AnchorSE2Impl::exports;
}


#endif
