#ifndef FACTOR_IMPL_LINEAR_TRANSLATION_H_
#define FACTOR_IMPL_LINEAR_TRANSLATION_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/measure-meta-linear-translation.h"


namespace
{
  inline static constexpr const char LinearTranslationLabel[] = "linear translation";
  inline static constexpr const char sighted_var[]           = "sighted";
  inline static constexpr const char observer_var[]           = "observer";

  // observer (xipp)
  struct ObserverKeyConduct
      : KeyContextualConduct<ObserverKeyConduct,
                             MetaKeyPosition_t,
                             MetaMeasureLinearTranslation_t::kM,
                             observer_var,
                             true>   // true for linear
  {
    inline static const key_process_matrix_t Hik {{-1, 0}, {0, -1}};
    const key_process_matrix_t               Aik;

    key_process_matrix_t compute_Aik_impl() const { return Aik; }

    // For linear systems
    ObserverKeyConduct(const std::string key_id, const measure_cov_t& rho, std::shared_ptr<Key_t>  init_point_ptr)
        : KeyContextualConduct(key_id, rho, init_point_ptr)
        , Aik(rho * Hik)
    {
    }
    
    // For NL cases
    ObserverKeyConduct(const std::string key_id, const measure_cov_t& rho)
    : KeyContextualConduct(key_id, rho)
    , Aik(rho * Hik)
    {
    }

  };
}   // namespace

namespace
{
  // sighted (xi)
  struct SightedKeyConduct
      : KeyContextualConduct<SightedKeyConduct,
                             MetaKeyPosition_t,
                             MetaMeasureLinearTranslation_t::kM,
                             sighted_var,
                             true>
  {
    inline static const key_process_matrix_t Hik {{1, 0}, {0, 1}};
    const key_process_matrix_t Aik;

    key_process_matrix_t compute_Aik_impl() const { return Aik; }

    // For linear systems
    SightedKeyConduct(const std::string key_id, const measure_cov_t& rho)
        : KeyContextualConduct(key_id, rho)
        , Aik(rho * Hik)
    {
    }

    // For NL cases
    SightedKeyConduct(const std::string key_id, const measure_cov_t& rho, std::shared_ptr<Key_t> init_point_ptr)
    : KeyContextualConduct(key_id, rho,init_point_ptr)
    , Aik(rho * Hik)
    {
    }

    // euclidian space
    static_assert(std::is_same_v<SightedKeyConduct::Key_t, SightedKeyConduct::part_state_vect_t>);
  };

}   // namespace

namespace
{
  class LinearTranslationFactor
      : public TrivialEuclidianFactor<LinearTranslationFactor,
                      LinearTranslationLabel,
                      MetaMeasureLinearTranslation_t,
                      SightedKeyConduct,
                      ObserverKeyConduct>
  {
    using BaseFactor_t = TrivialEuclidianFactor
                    <LinearTranslationFactor,
                      LinearTranslationLabel,
                      MetaMeasureLinearTranslation_t,
                      SightedKeyConduct,
                      ObserverKeyConduct>;
    friend BaseFactor_t;
    using SightedKey_process_matrix = typename SightedKeyConduct::key_process_matrix_t;
    using ObserverKey_process_matrix = typename ObserverKeyConduct::key_process_matrix_t;
    // passing some type definitions for convenience
    // using criterion_t = typename BaseFactor_t::criterion_t;
    // using measure_t = typename BaseFactor_t::measure_t;
    // using measure_cov_t = typename BaseFactor_t::measure_cov_t;
    // using matrices_Aik_t = typename  BaseFactor_t::matrices_Aik_t;
    // using composite_state_ptr_t = typename BaseFactor_t::composite_state_ptr_t;
    // using composite_of_opt_state_ptr_t = typename BaseFactor_t::composite_of_opt_state_ptr_t;

    // static_assert(std::is_same_v<SightedKey_process_matrix,)

    public:


    LinearTranslationFactor(
        const std::string&                                               factor_id,
        const criterion_t&                                            mes_vect,
        const measure_cov_t&                                             measure_cov,
        const std::array<std::string, kNbKeys>& keys_id,
        const composite_state_ptr_t & init_points_ptr)
        : BaseFactor_t(factor_id, mes_vect, measure_cov, keys_id, init_points_ptr)
    {
#if ENABLE_DEBUG_TRACE
      std::cout << "\t::  Factor " << factor_id << " created.\n";
#endif
    }
    static constexpr uint8_t kSightedKeyConductIdx = 0;
    static constexpr uint8_t kObserverKeyConductIdx = 1;

    // guess a potentially missing init key point from another by relying on the measurement
    // (used in NL systems if an init point must be set)
    static
    std::optional<
        std::tuple< std::shared_ptr<SightedKeyConduct::part_state_vect_t>, std::shared_ptr<ObserverKeyConduct::part_state_vect_t>>>
        guess_init_key_points_impl(
            // const std::tuple<std::optional<std::shared_ptr<SightedKeyConduct::part_state_vect_t>>,
            //                  std::optional<std::shared_ptr<ObserverKeyConduct::part_state_vect_t>>>&
            const composite_of_opt_state_ptr_t &
                                  x_init_ptr_optional_tup,
            const criterion_t& z)
    {
      // if both values are given, just echo the means
      if (std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
          && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
      {
        return std::make_tuple(std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).value(),
                               std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value());
      }
      // if sighted is given (eg x_{i-1}) but observer is not (eg x_{i}), use the measure to deduce
      // an init point for observer NOTE: this is the most likely scenario
      else if (std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
               && !std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
      {
        std::shared_ptr<ObserverKeyConduct::Key_t> observer_init_point_ptr;
        std::shared_ptr<SightedKeyConduct::Key_t> sighted_init_point_ptr
            = std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).value();


        observer_init_point_ptr = std::make_shared<ObserverKeyConduct::Key_t>(*sighted_init_point_ptr - z);

        return std::make_tuple(sighted_init_point_ptr, observer_init_point_ptr);
      }
      // if obverseR is given, and not the other one
      else if (!std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
               && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
      {
        std::shared_ptr<ObserverKeyConduct::Key_t> observer_init_point_ptr
            = std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value();
        std::shared_ptr<SightedKeyConduct::Key_t> sighted_init_point_ptr;

        sighted_init_point_ptr = std::make_shared<SightedKeyConduct::Key_t>(z - *observer_init_point_ptr);

        return std::make_tuple(sighted_init_point_ptr, observer_init_point_ptr);
      }
      // if no init on either key is given,
      else
      {
        return std::nullopt;
      }
    }

    private:
    
    // // making a friend so that we the next implementation method can stay private
    // friend criterion_t BaseFactor_t::compute_h_of_x_impl(const composite_state_ptr_t &X) const;

    criterion_t compute_h_of_x_impl(const composite_state_ptr_t & Xptr) const
    {
      // TODO: HACK: seems that there is a generic form for linear euclidian factor
      // Indeed, for any linear factor, we have h(x) = Sum_k ( Aik * xk  )
      return
        std::get<0>(this->keys_set).compute_Aik()* *std::get<0>(Xptr).get()
          +
          std::get<1>(this->keys_set).compute_Aik()* *std::get<1>(Xptr).get();
      // return criterion_t();
    }

    // private:
    // // defined at ctor
    // // NOTE: is that used ?? 
    // const factor_process_matrix_t roach
    //     = rho
    //       * factor_process_matrix_t {{1, 0, -1, 0},
    //                           {0, 1, 0, -1}};   // TODO: defined w.r.t to the partH of keyset
  };

}   // namespace


#endif
