#ifndef FACTOR_IMPL_Pose_Matcher_SE2_H_
#define FACTOR_IMPL_Pose_Matcher_SE2_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-SE2.h"
#include "factor_impl/measure-meta-rigid-motion-SE2.h"



namespace details_sam::Factor{
  namespace PoseMatcherSE2Impl{
    inline static constexpr const char observer_role_str[] = "observer key";
    inline static constexpr const char sighted_role_str[] = "sighted key";

    using namespace ::sam::Meta;
    using namespace ::details_sam::Conduct;

    struct SightedSE2KeyConduct 
      : KeyContextualConduct
        <SightedSE2KeyConduct,Key::PoseSE2,Measure::RigidBodyMotionSE2,sighted_role_str>
    {
      using BaseKeyCC_t = KeyContextualConduct
        <SightedSE2KeyConduct,Key::PoseSE2,Measure::RigidBodyMotionSE2,sighted_role_str>;
      using key_process_matrix_t = typename BaseKeyCC_t::key_process_matrix_t;

      key_process_matrix_t compute_Aik_at(const Key_t & Xk) const
      {
        // non sequitur
        // FIX: add the computations here anyway. Optionaly raise some errors if called
        key_process_matrix_t Ai;
        return Ai;
      }
      // the ctors
      SightedSE2KeyConduct(const std::string& key_id, const measure_cov_t& rho) : BaseKeyCC_t(key_id, rho)
      {}
      SightedSE2KeyConduct(const std::string& key_id, const measure_cov_t& rho, std::shared_ptr<Key_t> init_point_view) : BaseKeyCC_t(key_id, rho, init_point_view)
      {}
    };

    struct ObserverSE2KeyConduct 
      : KeyContextualConduct
        <ObserverSE2KeyConduct,Key::PoseSE2,Measure::RigidBodyMotionSE2,observer_role_str>
    {
      using BaseKeyCC_t = KeyContextualConduct
        <ObserverSE2KeyConduct,Key::PoseSE2,Measure::RigidBodyMotionSE2,observer_role_str>;
      using key_process_matrix_t = typename BaseKeyCC_t::key_process_matrix_t;

      key_process_matrix_t compute_Aik_at(const Key_t & Xk) const
      {
        // non sequitur
        // FIX: add the computations here anyway. Optionaly raise some errors if called
        key_process_matrix_t Ai;
        return Ai;
      }
      // the ctors
      ObserverSE2KeyConduct(const std::string& key_id, const measure_cov_t& rho) : BaseKeyCC_t(key_id, rho)
      {}
      ObserverSE2KeyConduct(const std::string& key_id, const measure_cov_t& rho, std::shared_ptr<Key_t> init_point_view) : BaseKeyCC_t(key_id, rho, init_point_view)
      {}
    };

    inline static constexpr const char pose_matcher_label[] = "pose matcher SE2";

    namespace exports{

      class PoseMatcherSE2 :
        public ::sam::Factor::BaseFactor<PoseMatcherSE2, pose_matcher_label, Measure::RigidBodyMotionSE2, SightedSE2KeyConduct, ObserverSE2KeyConduct>
      {

          public:
          using BaseFactor_t = BaseFactor<PoseMatcherSE2, pose_matcher_label, Measure::RigidBodyMotionSE2, SightedSE2KeyConduct, ObserverSE2KeyConduct>;
          friend BaseFactor_t;
          static_assert( factor_process_matrix_t::ColsAtCompileTime 
              == 
              (SightedSE2KeyConduct::key_process_matrix_t::ColsAtCompileTime + ObserverSE2KeyConduct::key_process_matrix_t::ColsAtCompileTime)
              ); // column numbers adds up

          PoseMatcherSE2(const std::string&                                    factor_id,
                       const measure_t &                                 mes_vect,
                       const measure_cov_t&                                  measure_cov,
                       const std::array<std::string, kNbKeys>& keys_id,
                       const composite_state_ptr_t & tuple_of_init_point_ptrs)
              : BaseFactor_t(factor_id, mes_vect, measure_cov, keys_id, tuple_of_init_point_ptrs)
          {
#if ENABLE_DEBUG_TRACE
            std::cout << "\t::  Factor " << factor_id << " (PoseMatcherSE2) created.\n";
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
            // if sighted exists but observer is unknown 
            // e.g. scan matching odometry where observer is Xi+1, sighted is Xi
            if (std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                && !std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
            {
                
                std::shared_ptr<ObserverSE2KeyConduct::Key_t> observer_init_point_ptr;
                std::shared_ptr<SightedSE2KeyConduct::Key_t> sighted_init_point_ptr
                    = std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).value();
                // X_observer = X_sighted * Z^-1
                observer_init_point_ptr 
                  = std::make_shared<ObserverSE2KeyConduct::Key_t>
                    (
                     (*sighted_init_point_ptr)
                               .compose( z .inverse())
                    );
                return std::make_tuple( sighted_init_point_ptr, observer_init_point_ptr );
            }
            else if (!std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
                && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
            {
              std::shared_ptr<SightedSE2KeyConduct::Key_t> sighted_init_point_ptr;
              std::shared_ptr<ObserverSE2KeyConduct::Key_t> observer_init_point_ptr
                  = std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value();

              // X_sighted = X_observer * Z
              sighted_init_point_ptr = std::make_shared<SightedSE2KeyConduct::Key_t>
                (sighted_init_point_ptr->compose(z));
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
            else // unable to deduce an init point for the keys
              return std::nullopt;
          }
          
          std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_at_impl(const composite_state_ptr_t & X) const
          {
            auto X_observer = *std::get<kObserverKeyConductIdx>(X);
            auto X_sighted = *std::get<kSightedKeyConductIdx>(X);
            // pre declaring the jacobian
            using Aik_Observer_t = std::tuple_element_t<kObserverKeyConductIdx, matrices_Aik_t>;
            using Aik_Sighted_t = std::tuple_element_t<kSightedKeyConductIdx, matrices_Aik_t>;

            // WARNING: interpretration : types name are wrong here (but they both are 3x3 matrices anyway)
            Aik_Observer_t J_XobsInv_Xobs,J_XobsInvXsigh_Xobs;
            Aik_Sighted_t  J_XobsInvXsigh_Xsigh, J_ZmY_Y;

            auto Y = X_observer.inverse(J_XobsInv_Xobs).compose(X_sighted, J_XobsInvXsigh_Xobs , J_XobsInvXsigh_Xsigh );

            criterion_t bi = this->rho*(
                      this->z.rminus( Y, {}, J_ZmY_Y).coeffs()
                );

            // compute tuple of the Aiks (just one in this factor)
            Aik_Sighted_t Aik_Sighted = this->rho*J_ZmY_Y*J_XobsInvXsigh_Xsigh;
            Aik_Observer_t Aik_Observer = this->rho*J_ZmY_Y*J_XobsInvXsigh_Xobs *J_XobsInv_Xobs;
            // 
            return {bi, {Aik_Sighted, Aik_Observer}};
          }

          criterion_t compute_r_of_x_at_impl(const composite_state_ptr_t & X) const
          {
            auto X_observer = *std::get<kObserverKeyConductIdx>(X);
            auto X_sighted = *std::get<kSightedKeyConductIdx>(X);
            return this->rho* (
                      this->z.rminus( X_observer.inverse().compose(X_sighted) ).coeffs()
                );
          }
      };

    }
  }
}

namespace sam::Factor{
  using namespace details_sam::Factor::PoseMatcherSE2Impl::exports;
}


#endif
