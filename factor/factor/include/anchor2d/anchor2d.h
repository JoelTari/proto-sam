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

// #include "core/config.h"
#include "factor/factor_interface.h"
#include "key-spatial-2d/key-spatial-2d.h"
#include "measure-motion-2d/measure-motion-2d.h"

namespace details_sam::Factor {
  namespace Anchor2dImpl{
    // factor instantiation from templates
    // instantiate the (unique) key conduct
    inline static constexpr const char anchor_role_str[] = "anchored";
    
    // the matrix Hik, an improvement would be to use constexpr raw array and then transform in Matrix in the class, somehow.
    inline static const Eigen::Matrix<double,2,2> Hik_prior {{1,0},{0,1}};

    using namespace ::sam::Meta;
    using namespace ::details_sam::Conduct;

    using PriorKeyConduct = 
      LinearKeyContextualConduct
          < 
            Key::Spatial2d
            ,Measure::Motion2d
            ,anchor_role_str
            ,&Hik_prior
          >;

    inline static constexpr const char anchorLabel[] = "anchor 2d";

    namespace exports{

      class Anchor2d
          : public 
              ::sam::Factor::LinearEuclidianFactor
                <
                  Anchor2d
                  , anchorLabel
                  , PriorKeyConduct
                >
      {
        using BaseFactor_t = ::sam::Factor::LinearEuclidianFactor<Anchor2d, anchorLabel, PriorKeyConduct>;
        friend BaseFactor_t;
        static_assert(std::is_same_v<PriorKeyConduct::key_process_matrix_t, factor_process_matrix_t>  ); // because only 1 key
        static_assert(std::is_same_v<PriorKeyConduct::Key_t, criterion_t>); // because linear factor &&  size M = size N
        static_assert(std::is_same_v<std::tuple<PriorKeyConduct::key_process_matrix_t>, matrices_Aik_t>);

        public:

        // ctor
        Anchor2d(const std::string&                                    factor_id,
                     const measure_t&                                 mes_vect,
                     const measure_cov_t&                               measure_cov,
                     const std::array<std::string, kNbKeys>& keys_id)
            : BaseFactor_t(factor_id, mes_vect, measure_cov, keys_id)
        {
        }

        // init point guesser
        static std::optional<std::tuple< std::shared_ptr<PriorKeyConduct::Key_t> >>
            guess_init_key_points_impl( const composite_of_opt_state_ptr_t &
                // std::tuple<std::optional<  std::shared_ptr<UniqueKeyConduct_t::Key_t> >>
                                      x_init_ptr_optional_tup,
                const measure_t& z)
        {
          if (std::get<0>(x_init_ptr_optional_tup).has_value())
            // if the optional mean value inside the tuple is given, we report this value as initial
            // guess
            return std::make_tuple(std::get<0>(x_init_ptr_optional_tup).value());
          else   // make a new state in the heap from the measurement
          {
            auto xinit_ptr = std::make_shared<PriorKeyConduct::Key_t>(z);
            return std::make_tuple(xinit_ptr);
          }
          // NOTE: it never returns std::nullopt, that's normal in this situation
        }

      };
    }  // namespace exports
  }  // namespace impl
} // namespace details_sam::factor

// exposing only relevant structure to client
namespace sam::Factor{
  using namespace details_sam::Factor::Anchor2dImpl::exports;
}
