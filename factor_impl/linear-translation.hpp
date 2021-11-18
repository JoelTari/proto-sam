#ifndef FACTOR_IMPL_LINEAR_TRANSLATION_H_
#define FACTOR_IMPL_LINEAR_TRANSLATION_H_

#include "factorV3.h"
#include "key-meta-position.h"
#include "measure-meta-linear-translation.h"


static constexpr const char LinearTranslationLabel[] = "linear translation";
static constexpr const char observee_var[]           = "observee";
static constexpr const char observer_var[]           = "observer";

// observer (xipp)
struct ObserverKeyConduct
    : KeyContextualConduct<ObserverKeyConduct,
                           MetaKeyPosition_t,
                           MetaMeasureLinearTranslation_t::kM,
                           observer_var>
{
  inline static const process_matrix_t H {{-1, 0}, {0, -1}};
  const process_matrix_t partA;

  process_matrix_t compute_part_A_impl() const
  {
    return partA;
  }

  ObserverKeyConduct(const std::string key_id, const measure_cov_t & rho):
      KeyContextualConduct(key_id,rho)
      , partA(rho*H)
    {}
};

// observee (xi)
struct ObserveeKeyConduct
    : KeyContextualConduct<ObserveeKeyConduct,
                           MetaKeyPosition_t,
                           MetaMeasureLinearTranslation_t::kM,
                           observee_var>
{
  const process_matrix_t H {{1, 0}, {0, 1}};
  const process_matrix_t partA;

  process_matrix_t compute_part_A_impl() const
  {
    return partA;
  }

  ObserveeKeyConduct(const std::string key_id, const measure_cov_t & rho):
      KeyContextualConduct(key_id,rho)
      , partA(rho*H)
    {}
};

class LinearTranslationFactor
    : public FactorV3<LinearTranslationFactor,
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
      : FactorV3(factor_id, mes_vect, measure_cov, keys_id)
  {
  }
};


#endif
