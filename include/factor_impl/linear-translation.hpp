#ifndef FACTOR_IMPL_LINEAR_TRANSLATION_H_
#define FACTOR_IMPL_LINEAR_TRANSLATION_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/measure-meta-linear-translation.h"


namespace
{
  inline static constexpr const char LinearTranslationLabel[] = "linear translation";
  inline static constexpr const char observee_var[]           = "observee";
  inline static constexpr const char observer_var[]           = "observer";

  // observer (xipp)
  struct ObserverKeyConduct
      : KeyContextualConduct<ObserverKeyConduct,
                             MetaKeyPosition_t,
                             MetaMeasureLinearTranslation_t::kM,
                             observer_var,
                             true>   // true for linear
  {
    inline static const process_matrix_t partH {{-1, 0}, {0, -1}};
    const process_matrix_t               partA;

    process_matrix_t compute_part_A_impl() const { return partA; }

    // For linear systems
    ObserverKeyConduct(const std::string key_id, const measure_cov_t& rho, std::shared_ptr<part_state_vect_t>  init_point_ptr)
        : KeyContextualConduct(key_id, rho, init_point_ptr)
        , partA(rho * partH)
    {
    }
    
    // For NL cases
    ObserverKeyConduct(const std::string key_id, const measure_cov_t& rho)
    : KeyContextualConduct(key_id, rho)
    , partA(rho * partH)
    {
    }
  };
}   // namespace

namespace
{
  // observee (xi)
  struct ObserveeKeyConduct
      : KeyContextualConduct<ObserveeKeyConduct,
                             MetaKeyPosition_t,
                             MetaMeasureLinearTranslation_t::kM,
                             observee_var,
                             true>
  {
    const process_matrix_t partH {{1, 0}, {0, 1}};
    const process_matrix_t partA;

    process_matrix_t compute_part_A_impl() const { return partA; }

    // For linear systems
    ObserveeKeyConduct(const std::string key_id, const measure_cov_t& rho)
        : KeyContextualConduct(key_id, rho)
        , partA(rho * partH)
    {
    }

    // For NL cases
    ObserveeKeyConduct(const std::string key_id, const measure_cov_t& rho, std::shared_ptr<part_state_vect_t> init_point_ptr)
    : KeyContextualConduct(key_id, rho,init_point_ptr)
    , partA(rho * partH)
    {
    }
  };

}   // namespace

namespace
{
  class LinearTranslationFactor
      : public Factor<LinearTranslationFactor,
                      LinearTranslationLabel,
                      MetaMeasureLinearTranslation_t,
                      ObserveeKeyConduct,
                      ObserverKeyConduct>
  {
    public:
//     LinearTranslationFactor(
//         const std::string&                                               factor_id,
//         const measure_vect_t&                                            mes_vect,
//         const measure_cov_t&                                             measure_cov,
//         const std::array<std::string, LinearTranslationFactor::kNbKeys>& keys_id)
//         : Factor(factor_id, mes_vect, measure_cov, keys_id)
//     {
// #if ENABLE_DEBUG_TRACE
//       std::cout << "\t::  Factor " << factor_id << " created.\n";
// #endif
//     }

    LinearTranslationFactor(
        const std::string&                                               factor_id,
        const criterion_t&                                            mes_vect,
        const measure_cov_t&                                             measure_cov,
        const std::array<std::string, LinearTranslationFactor::kNbKeys>& keys_id,
        std::tuple< std::shared_ptr<ObserveeKeyConduct::part_state_vect_t>,std::shared_ptr<ObserverKeyConduct::part_state_vect_t> > init_points_ptr)
        : Factor(factor_id, mes_vect, measure_cov, keys_id, init_points_ptr)
    {
#if ENABLE_DEBUG_TRACE
      std::cout << "\t::  Factor " << factor_id << " created.\n";
#endif
    }
    static constexpr uint8_t kObserveeKeyConductIdx = 0;
    static constexpr uint8_t kObserverKeyConductIdx = 1;

    // guess a potentially missing init key point from another by relying on the measurement
    // (used in NL systems if an init point must be set)
    static
    std::optional<
        std::tuple< std::shared_ptr<ObserveeKeyConduct::part_state_vect_t>, std::shared_ptr<ObserverKeyConduct::part_state_vect_t>>>
        guess_init_key_points_impl(
            const std::tuple<std::optional<std::shared_ptr<ObserveeKeyConduct::part_state_vect_t>>,
                             std::optional<std::shared_ptr<ObserverKeyConduct::part_state_vect_t>>>&
                                  x_init_ptr_optional_tup,
            const criterion_t& z)
    {
      // if both values are given, just echo the means
      if (std::get<kObserveeKeyConductIdx>(x_init_ptr_optional_tup).has_value()
          && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
      {
        return std::make_tuple(std::get<kObserveeKeyConductIdx>(x_init_ptr_optional_tup).value(),
                               std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value());
      }
      // if observee is given (eg x_{i-1}) but observer is not (eg x_{i}), use the measure to deduce
      // an init point for observer NOTE: this is the most likely scenario
      else if (std::get<kObserveeKeyConductIdx>(x_init_ptr_optional_tup).has_value()
               && !std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
      {
        std::shared_ptr<ObserverKeyConduct::part_state_vect_t> observer_init_point_ptr;
        std::shared_ptr<ObserveeKeyConduct::part_state_vect_t> observee_init_point_ptr
            = std::get<kObserveeKeyConductIdx>(x_init_ptr_optional_tup).value();


        observer_init_point_ptr = std::make_shared<ObserverKeyConduct::part_state_vect_t>(*observee_init_point_ptr - z);

        return std::make_tuple(observee_init_point_ptr, observer_init_point_ptr);
      }
      // if obverseR is given, and not the other one
      else if (!std::get<kObserveeKeyConductIdx>(x_init_ptr_optional_tup).has_value()
               && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
      {
        std::shared_ptr<ObserverKeyConduct::part_state_vect_t> observer_init_point_ptr
            = std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value();
        std::shared_ptr<ObserveeKeyConduct::part_state_vect_t> observee_init_point_ptr;

        observee_init_point_ptr = std::make_shared<ObserveeKeyConduct::part_state_vect_t>(z - *observer_init_point_ptr);

        return std::make_tuple(observee_init_point_ptr, observer_init_point_ptr);
      }
      // if no init on either key is given,
      else
      {
        return std::nullopt;
      }
    }

    criterion_t compute_h_of_x_impl(const state_vector_t& x) const
    {
      return process_matrix_t {{1, 0, -1, 0}, {0, 1, 0, -1}} * x;
    }

    private:
    // defined at ctor
    const process_matrix_t roach
        = rho
          * process_matrix_t {{1, 0, -1, 0},
                              {0, 1, 0, -1}};   // TODO: defined w.r.t to the partH of keyset
  };

}   // namespace


#endif
