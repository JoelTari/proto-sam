#ifndef FACTOR_IMPL_LINEAR_TRANSLATION_H_
#define FACTOR_IMPL_LINEAR_TRANSLATION_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/measure-meta-linear-translation.h"


namespace __LinearTranslationKeyConducts
{
  inline static constexpr const char sighted_var[]           = "sighted";
  inline static constexpr const char observer_var[]           = "observer";

  // the process matrices
  inline static const Eigen::Matrix<double, 2, 2> Hik_Observer {{-1,0},{0,-1}};
  inline static const Eigen::Matrix<double, 2, 2> Hik_Sighted {{1,0},{0,1}};
  // HACK: matrices Hik_ are passed in-template as the address of the above declarations
  using ObserverKeyConduct_t = LinearKeyContextualConduct<MetaKeyPosition_t , MetaMeasureLinearTranslation_t , observer_var, &Hik_Observer>;

  using SightedKeyConduct_t = LinearKeyContextualConduct<MetaKeyPosition_t, MetaMeasureLinearTranslation_t, sighted_var, &Hik_Sighted>;
}   // namespace
using ObserverKeyConduct_t = typename __LinearTranslationKeyConducts::ObserverKeyConduct_t;
using SightedKeyConduct_t = typename __LinearTranslationKeyConducts::SightedKeyConduct_t;

namespace __LinearTranslationFactor
{
  inline static constexpr const char LinearTranslationLabel[] = "linear translation";
  class LinearTranslationFactor
      : public LinearEuclidianFactor<LinearTranslationFactor,
                      LinearTranslationLabel,
                      MetaMeasureLinearTranslation_t,
                      SightedKeyConduct_t,
                      ObserverKeyConduct_t>
  {
    using BaseFactor_t = LinearEuclidianFactor
                    <LinearTranslationFactor,
                      LinearTranslationLabel,
                      MetaMeasureLinearTranslation_t,
                      SightedKeyConduct_t,
                      ObserverKeyConduct_t>;
    friend BaseFactor_t;
    using SightedKey_process_matrix = typename SightedKeyConduct_t::key_process_matrix_t;
    using ObserverKey_process_matrix = typename ObserverKeyConduct_t::key_process_matrix_t;

    public:


    LinearTranslationFactor(
        const std::string&                                               factor_id,
        const measure_t&                                            mes_vect,
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
    // std::optional<
    //     std::tuple< std::shared_ptr<SightedKeyConduct_t::part_state_vect_t>, std::shared_ptr<ObserverKeyConduct_t::part_state_vect_t>>>
    std::optional<composite_state_ptr_t>
        guess_init_key_points_impl(
            // const std::tuple<std::optional<std::shared_ptr<SightedKeyConduct_t::part_state_vect_t>>,
            //                  std::optional<std::shared_ptr<ObserverKeyConduct_t::part_state_vect_t>>>&
            const composite_of_opt_state_ptr_t &
                                  x_init_ptr_optional_tup,
            const measure_t& z)
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
        std::shared_ptr<ObserverKeyConduct_t::Key_t> observer_init_point_ptr;
        std::shared_ptr<SightedKeyConduct_t::Key_t> sighted_init_point_ptr
            = std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).value();


        observer_init_point_ptr = std::make_shared<ObserverKeyConduct_t::Key_t>(*sighted_init_point_ptr - z);

        return std::make_tuple(sighted_init_point_ptr, observer_init_point_ptr);
      }
      // if obverseR is given, and not the other one
      else if (!std::get<kSightedKeyConductIdx>(x_init_ptr_optional_tup).has_value()
               && std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).has_value())
      {
        std::shared_ptr<ObserverKeyConduct_t::Key_t> observer_init_point_ptr
            = std::get<kObserverKeyConductIdx>(x_init_ptr_optional_tup).value();
        std::shared_ptr<SightedKeyConduct_t::Key_t> sighted_init_point_ptr;

        sighted_init_point_ptr = std::make_shared<SightedKeyConduct_t::Key_t>(z - *observer_init_point_ptr);

        return std::make_tuple(sighted_init_point_ptr, observer_init_point_ptr);
      }
      // if no init on either key is given,
      else
      {
        return std::nullopt;
      }
    }
  };

}   // namespace
using LinearTranslationFactor = __LinearTranslationFactor::LinearTranslationFactor;


#endif
