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

    ObserveeKeyConduct(const std::string key_id, const measure_cov_t& rho)
        : KeyContextualConduct(key_id, rho)
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
    LinearTranslationFactor(
        const std::string&                                               factor_id,
        const measure_vect_t&                                            mes_vect,
        const measure_cov_t&                                             measure_cov,
        const std::array<std::string, LinearTranslationFactor::kNbKeys>& keys_id)
        : Factor(factor_id, mes_vect, measure_cov, keys_id)
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
        std::tuple<ObserveeKeyConduct::part_state_vect_t, ObserverKeyConduct::part_state_vect_t>>
        guess_init_key_poinst_impl(
            const std::tuple<std::optional<ObserveeKeyConduct::part_state_vect_t>,
                             std::optional<ObserverKeyConduct::part_state_vect_t>>&
                                  x_init_optional_tup,
            const measure_vect_t& z)
    {
      // if both values are given, just echo the means
      if (std::get<kObserveeKeyConductIdx>(x_init_optional_tup).has_value()
          && std::get<kObserverKeyConductIdx>(x_init_optional_tup).has_value())
      {
        return std::make_tuple(std::get<kObserveeKeyConductIdx>(x_init_optional_tup).value(),
                               std::get<kObserverKeyConductIdx>(x_init_optional_tup).value());
      }
      // if observee is given (eg x_{i-1}) but observer is not (eg x_{i}), use the measure to deduce
      // an init point for observer NOTE: this is the most likely scenario
      else if (std::get<kObserveeKeyConductIdx>(x_init_optional_tup).has_value()
               && !std::get<kObserverKeyConductIdx>(x_init_optional_tup).has_value())
      {
        ObserverKeyConduct::part_state_vect_t observer_init_point;
        ObserveeKeyConduct::part_state_vect_t observee_init_point
            = std::get<kObserveeKeyConductIdx>(x_init_optional_tup).value();

        observer_init_point = observee_init_point - z;

        return std::make_tuple(observee_init_point, observer_init_point);
      }
      // if obverseR is given, and not the other one
      else if (!std::get<kObserveeKeyConductIdx>(x_init_optional_tup).has_value()
               && std::get<kObserverKeyConductIdx>(x_init_optional_tup).has_value())
      {
        ObserverKeyConduct::part_state_vect_t observer_init_point
            = std::get<kObserverKeyConductIdx>(x_init_optional_tup).value();
        ObserveeKeyConduct::part_state_vect_t observee_init_point;

        observee_init_point = z - observer_init_point;

        return std::make_tuple(observee_init_point, observer_init_point);
      }
      // if no init on either key is given,
      else
      {
        return std::nullopt;
      }
    }

    measure_vect_t compute_h_of_x_impl(const state_vector_t& x) const
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
