#ifndef FACTOR_IMPL_LINEAR_TRANSLATION_H_
#define FACTOR_IMPL_LINEAR_TRANSLATION_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/measure-meta-linear-translation.h"


namespace{
inline static constexpr const char LinearTranslationLabel[] = "linear translation";
inline static constexpr const char observee_var[]           = "observee";
inline static constexpr const char observer_var[]           = "observer";

// observer (xipp)
struct ObserverKeyConduct
    : KeyContextualConduct<ObserverKeyConduct,
                           MetaKeyPosition_t,
                           MetaMeasureLinearTranslation_t::kM,
                           observer_var>
{
  inline static const process_matrix_t H {{-1, 0}, {0, -1}};
  const process_matrix_t               partA;

  process_matrix_t compute_part_A_impl() const { return partA; }

  measure_vect_t compute_part_h_of_part_x_impl(const part_state_vect_t & part_x)
  {
    // OPTIMIZE: this is the same for every linear KeyCC
    return H*part_x;
  }

  ObserverKeyConduct(const std::string key_id, const measure_cov_t& rho)
      : KeyContextualConduct(key_id, rho)
      , partA(rho * H)
  {
  }
};
}

namespace
{
// observee (xi)
struct ObserveeKeyConduct
    : KeyContextualConduct<ObserveeKeyConduct,
                           MetaKeyPosition_t,
                           MetaMeasureLinearTranslation_t::kM,
                           observee_var>
{
  const process_matrix_t H {{1, 0}, {0, 1}};
  const process_matrix_t partA;

  process_matrix_t compute_part_A_impl() const { return partA; }

  ObserveeKeyConduct(const std::string key_id, const measure_cov_t& rho)
      : KeyContextualConduct(key_id, rho)
      , partA(rho * H)
  {
  }
};

}

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
      const std::string&    factor_id,
      const measure_vect_t& mes_vect,
      const measure_cov_t&  measure_cov,
      const std::array<std::string, LinearTranslationFactor::kNbKeys>& keys_id)
      : Factor(factor_id, mes_vect, measure_cov, keys_id)
  {
#if ENABLE_DEBUG_TRACE
    std::cout << "\t::  Factor " << factor_id << " created.\n";
#endif
  }
};

}


#endif
