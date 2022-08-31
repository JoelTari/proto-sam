#ifndef FACTOR_IMPL_cartesian_obs_SE2_H_
#define FACTOR_IMPL_cartesian_obs_SE2_H_


#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-SE2.h"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/measure-meta-linear-translation.h"

namespace details_sam::Factor{
  namespace CartesianObsSE2Impl{
    inline static constexpr const char observer_role_str[] = "observer key";
    inline static constexpr const char sighted_role_str[] = "sighted landmark key";

    using namespace ::sam::Meta;
    using namespace ::details_sam::Conduct;

    struct SightedLandmarkKeyConduct 
      : KeyContextualConduct
        <SightedLandmarkKeyConduct,Key::Position2d,Measure::LinearTranslation2d,sighted_role_str>
    {
      using BaseKeyCC_t = KeyContextualConduct
        <SightedLandmarkKeyConduct,Key::Position2d,Measure::LinearTranslation2d,sighted_role_str>;
      using BaseKeyCC_t::BaseKeyCC_t;
      using key_process_matrix_t = typename BaseKeyCC_t::key_process_matrix_t;

      key_process_matrix_t compute_Hik_at(const Key_t & Xk) const
      {
        // non sequitur
        // FIX: add the computations here anyway.
        throw std::runtime_error("not implemented yet");
        key_process_matrix_t Hik;
        return Hik;
      }
    };

    struct ObserverSE2KeyConduct 
      : KeyContextualConduct
        <ObserverSE2KeyConduct,Key::PoseSE2,Measure::LinearTranslation2d,observer_role_str>
    {
      using BaseKeyCC_t = KeyContextualConduct
        <ObserverSE2KeyConduct,Key::PoseSE2,Measure::LinearTranslation2d,observer_role_str>;
      using BaseKeyCC_t::BaseKeyCC_t;
      using key_process_matrix_t = typename BaseKeyCC_t::key_process_matrix_t;

      key_process_matrix_t compute_Aik_at(const Key_t & Xk) const
      {
        // non sequitur
        // FIX: add the computations here anyway.
        throw std::runtime_error("not implemented yet");
        key_process_matrix_t Ai;
        return Ai;
      }
    };

    inline static constexpr const char cartesian_obs_SE2_label[] = "cartesian observation of a landmark from an SE2 pose";

    namespace exports{

      class LandmarkCartesianObsSE2 :
        public ::sam::Factor::BaseFactor<LandmarkCartesianObsSE2, cartesian_obs_SE2_label, SightedLandmarkKeyConduct, ObserverSE2KeyConduct>
      {

          public:
          using BaseFactor_t = BaseFactor<LandmarkCartesianObsSE2, cartesian_obs_SE2_label, SightedLandmarkKeyConduct, ObserverSE2KeyConduct>;
          friend BaseFactor_t;
          // static_assert( factor_process_matrix_t::ColsAtCompileTime 
          //     == 
          //     (SightedLandmarkKeyConduct::key_process_matrix_t::ColsAtCompileTime + ObserverSE2KeyConduct::key_process_matrix_t::ColsAtCompileTime)
          //     ); // column numbers adds up

          LandmarkCartesianObsSE2(const std::string&                                    factor_id,
                       const measure_t &                                 mes_vect,
                       const measure_cov_t&                                  measure_cov,
                       const std::array<std::string, kNbKeys>& keys_id)
              : BaseFactor_t(factor_id, mes_vect, measure_cov, keys_id)
          {
#if ENABLE_DEBUG_TRACE
            std::cout << "\t::  Factor " << factor_id << " (LandmarkCartesianObsSE2) created.\n";
#endif
          }

          static constexpr uint8_t kSightedKeyConductIdx = 0;
          static constexpr uint8_t kObserverKeyConductIdx = 1;

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
            // if sighted landmark exists but observer is unknown
            if (std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                && !std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
            {
                
                std::shared_ptr<ObserverSE2KeyConduct::Key_t> observer_init_point_ptr;
                std::shared_ptr<SightedLandmarkKeyConduct::Key_t> sighted_init_point_ptr
                    = std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).value();
                // its impossible to deduce the SE2 init pose in that situation
                return std::nullopt;  
            }
            else if (!std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
            {
              std::shared_ptr<SightedLandmarkKeyConduct::Key_t> sighted_init_point_ptr;
              std::shared_ptr<ObserverSE2KeyConduct::Key_t> observer_init_point_ptr
                  = std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value();

              // b_sighted = X_observer act (Z)
              sighted_init_point_ptr = std::make_shared<SightedLandmarkKeyConduct::Key_t>
                (observer_init_point_ptr->act(z));
              return std::make_tuple( sighted_init_point_ptr ,observer_init_point_ptr );
            }
            // both sighted and observer already have init values (e.g. loop closure situation)
            else if (std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
            {
              return std::make_tuple( std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).value() 
                      ,std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value() 
                      );
            }
            else // unable to deduce an init point for the keys when not knowing anything
              return std::nullopt;
          }
          
          std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_at_impl(const composite_state_ptr_t & X) const
          {
            auto X_observer = *std::get<kObserverKeyConductIdx>(X);
            auto B_sighted = *std::get<kSightedKeyConductIdx>(X);
            // pre declaring the jacobian
            using process_matrix_Observer_t = std::tuple_element_t<kObserverKeyConductIdx, matrices_Aik_t>;
            using process_matrix_Sighted_t = std::tuple_element_t<kSightedKeyConductIdx, matrices_Aik_t>;

            Eigen::Matrix<double,3,3> J_Xinv_X; // FIX: oversight in the API, no types specified for matrix 3*3
            process_matrix_Observer_t J_Xinvb_Xinv;  // mat 2*3
            process_matrix_Sighted_t  J_Xinvb_b; // mat 2*2



            criterion_t bi = - this->rho*
                      (this->z - X_observer.inverse(J_Xinv_X).act(B_sighted, J_Xinvb_Xinv, J_Xinvb_b) );

            // compute tuple of the Aiks (just one in this factor)
            process_matrix_Observer_t Aik_Observer = - this->rho* J_Xinvb_Xinv*J_Xinv_X ;
            process_matrix_Sighted_t Aik_Sighted =  -this->rho*J_Xinvb_b;
            // 
            return {bi, {Aik_Sighted, Aik_Observer}};
          }

          criterion_t compute_r_of_x_at_impl(const composite_state_ptr_t & X) const
          {
            auto X_observer = *std::get<kObserverKeyConductIdx>(X);
            auto B_sighted = *std::get<kSightedKeyConductIdx>(X);
            return - this->rho* (
                      this->z - X_observer.inverse().act(B_sighted)
                );
          }
      };

    }
  }
}

namespace sam::Factor{
  using namespace details_sam::Factor::CartesianObsSE2Impl::exports;
}

#endif
