/* 
 * Copyright 2023 AKKA Technologies and LAAS-CNRS (joel.tari@akka.eu) 
 * 
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by 
 * the European Commission - subsequent versions of the EUPL (the "Licence"); 
 * You may not use this work except in compliance with the Licence. 
 * You may obtain a copy of the Licence at: 
 * 
 * https://joinup.ec.europa.eu/software/page/eupl 
 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the Licence is distributed on an "AS IS" basis, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the Licence for the specific language governing permissions and 
 * limitations under the Licence. 
 */
 
#pragma once

#include "factor/factor_interface.h"
#include "key-spatial-SE2/key-spatial-SE2.h"
#include "measure-velocity-SE2/measure-velocity-SE2.h"



namespace details_sam::Factor{
  namespace MotionModelSE2Impl{
    inline static constexpr const char subsequent_role_str[] = "subsequent key X_{i+1}";
    inline static constexpr const char antecedent_role_str[] = "antecedent key X_{i}";

    using namespace ::sam::Meta;
    using namespace ::details_sam::Conduct;

    struct SubsequentSE2KeyConduct 
      : KeyContextualConduct
        <SubsequentSE2KeyConduct,Key::SpatialSE2,Measure::VelocitySE2,subsequent_role_str>
    {
      using BaseKeyCC_t = KeyContextualConduct
        <SubsequentSE2KeyConduct,Key::SpatialSE2,Measure::VelocitySE2,subsequent_role_str>;
      // inherited ctors
      using BaseKeyCC_t::BaseKeyCC_t;
      using key_process_matrix_t = typename BaseKeyCC_t::key_process_matrix_t;

      key_process_matrix_t compute_Hik_at_impl(const Key_t & Xk) const
      {
        // non sequitur
        // FIX: add the computations here anyway. Optionaly raise some errors if called
        throw std::runtime_error("not implemented yet");
        key_process_matrix_t Hik;
        return Hik;
      }
      // the ctors
      // SubsequentSE2KeyConduct(const std::string& key_id, const measure_cov_t& rho) : BaseKeyCC_t(key_id, rho)
      // {}
      // SubsequentSE2KeyConduct(const std::string& key_id, const measure_cov_t& rho, std::shared_ptr<Key_t> init_point_view) : BaseKeyCC_t(key_id, rho, init_point_view)
      // {}
    };

    struct AntecedentSE2KeyConduct 
      : KeyContextualConduct
        <AntecedentSE2KeyConduct,Key::SpatialSE2,Measure::VelocitySE2,antecedent_role_str>
    {
      using BaseKeyCC_t = KeyContextualConduct
        <AntecedentSE2KeyConduct,Key::SpatialSE2,Measure::VelocitySE2,antecedent_role_str>;
      using BaseKeyCC_t::BaseKeyCC_t;
      using key_process_matrix_t = typename BaseKeyCC_t::key_process_matrix_t;

      key_process_matrix_t compute_Hik_at_impl(const Key_t & Xk) const
      {
        // non sequitur
        // FIX: add the computations here anyway. Optionaly raise some errors if called
        throw std::runtime_error("not implemented yet");
        key_process_matrix_t Hik;
        return Hik;
      }
      // // the ctors
      // AntecedentSE2KeyConduct(const std::string& key_id, const measure_cov_t& rho) : BaseKeyCC_t(key_id, rho)
      // {}
      // AntecedentSE2KeyConduct(const std::string& key_id, const measure_cov_t& rho, std::shared_ptr<Key_t> init_point_view) : BaseKeyCC_t(key_id, rho, init_point_view)
      // {}
    };

    inline static constexpr const char motion_model_label[] = "motion model u(t) from antecedent key SE2 to subsequent key SE2";

    namespace exports{

      class MotionModelSE2 :
        public ::sam::Factor::BaseFactor<MotionModelSE2, motion_model_label, SubsequentSE2KeyConduct, AntecedentSE2KeyConduct>
      {

          public:
          using BaseFactor_t = BaseFactor<MotionModelSE2, motion_model_label, SubsequentSE2KeyConduct, AntecedentSE2KeyConduct>;
          friend BaseFactor_t;
          static_assert( factor_process_matrix_t::ColsAtCompileTime 
              == 
              (SubsequentSE2KeyConduct::key_process_matrix_t::ColsAtCompileTime + AntecedentSE2KeyConduct::key_process_matrix_t::ColsAtCompileTime)
              ); // column numbers adds up

          MotionModelSE2(const std::string&                                    factor_id,
                       const measure_t &                                 mes_vect,
                       const measure_cov_t&                                  measure_cov,
                       const std::array<std::string, kNbKeys>& keys_id)
              : BaseFactor_t(factor_id, mes_vect, measure_cov, keys_id)
          {
          }

          static constexpr uint8_t kSubsequentKeyConductIdx = 0;
          static constexpr uint8_t kAntecedentKeyConductIdx = 1;

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
            // if subsequent exists but antecedent is unknown (unlikely though)
            if (std::get<kSubsequentKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                && !std::get<kAntecedentKeyConductIdx>(x_init_ptr_optional_tup).has_value())
            {
                
                std::shared_ptr<AntecedentSE2KeyConduct::Key_t> antecedent_init_point_ptr;
                std::shared_ptr<SubsequentSE2KeyConduct::Key_t> subsequent_init_point_ptr
                    = std::get<kSubsequentKeyConductIdx>(x_init_ptr_optional_tup).value();
                // X_i = X_{i+1} * (Exp u)^-1
                antecedent_init_point_ptr 
                  = std::make_shared<AntecedentSE2KeyConduct::Key_t>
                    (
                     (*subsequent_init_point_ptr)
                               .compose( 
                                         manif::SE2Tangentd(z)
                                         .exp()
                                         .inverse()
                                       )
                    );
                return std::make_tuple( subsequent_init_point_ptr, antecedent_init_point_ptr );
            }
            // subsequent Xi+1 doesnt exist but Xi does  NOTE: most common
            else if (!std::get<kSubsequentKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                && std::get<kAntecedentKeyConductIdx>(x_init_ptr_optional_tup).has_value())
            {
              std::shared_ptr<SubsequentSE2KeyConduct::Key_t> subsequent_init_point_ptr;
              std::shared_ptr<AntecedentSE2KeyConduct::Key_t> antecedent_init_point_ptr
                  = std::get<kAntecedentKeyConductIdx>(x_init_ptr_optional_tup).value();

              subsequent_init_point_ptr = std::make_shared<SubsequentSE2KeyConduct::Key_t>
                (*antecedent_init_point_ptr + manif::SE2Tangentd(z));
              return std::make_tuple( subsequent_init_point_ptr ,antecedent_init_point_ptr );
            }
            else if (std::get<kSubsequentKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                && std::get<kAntecedentKeyConductIdx>(x_init_ptr_optional_tup).has_value())
            {
              return std::make_tuple( std::get<kSubsequentKeyConductIdx>(x_init_ptr_optional_tup).value() 
                      ,std::get<kAntecedentKeyConductIdx>(x_init_ptr_optional_tup).value() 
                      );
            }
            else // unable to deduce any init point when no init point is available
              return std::nullopt;
          }
          
          // WARNING: this differs from my notes, perhaps the measure type should be in the skew-sym rather than vec3
          std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_at_impl(const composite_state_ptr_t & X) const
          {
            // extract Xk (X is a tuple of 1 element...)
            auto X_kp1 = *std::get<kSubsequentKeyConductIdx>(X);
            auto X_k = *std::get<kAntecedentKeyConductIdx>(X);
            // pre declaring the jacobian
            using Ai_kp1_t = std::tuple_element_t<0, matrices_Aik_t>;
            using Ai_k_t = std::tuple_element_t<1, matrices_Aik_t>;
            Ai_kp1_t J_rminus_X_kp1;
            Ai_k_t J_rminus_X_k;
            // compute bi = -  r(X) = - rho * ( Z (r-) X )
            // and fill the Jacobian
            criterion_t bi = -this->rho*(this->z - X_kp1.rminus( X_k , J_rminus_X_kp1, J_rminus_X_k)).coeffs();
            // NOTE: (z - Xkp1(r-)Xk ).coeffs() is same as (z.coeffs()  - Xkp1(r-)Xk .coeffs() )  <= linear operations of hat/vee
            // compute tuple of the Aiks (just one in this factor)
            Ai_kp1_t Ai_kp1 = - this->rho * J_rminus_X_kp1;
            Ai_k_t Ai_k = -this->rho * J_rminus_X_k;
            // 
            return {bi, {Ai_kp1, Ai_k}};
          }

          criterion_t compute_r_of_x_at_impl(const composite_state_ptr_t & X) const
          {
            auto X_kp1 = *std::get<kSubsequentKeyConductIdx>(X);
            auto X_k   = *std::get<kAntecedentKeyConductIdx>(X);
            return this->rho* (this->z - X_kp1.rminus(X_k)) .coeffs();
            // NOTE: (z - Xkp1(r-)Xk ).coeffs() is same as (z.coeffs()  - Xkp1(r-)Xk .coeffs() )  <= linear operations of hat/vee
          }
      };

    }
  }
}

namespace sam::Factor{
  using namespace details_sam::Factor::MotionModelSE2Impl::exports;
}

