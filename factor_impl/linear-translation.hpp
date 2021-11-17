#ifndef FACTOR_IMPL_LINEAR_TRANSLATION_H_
#define FACTOR_IMPL_LINEAR_TRANSLATION_H_

#include "key-meta-position.h"
#include "measure-meta-linear-translation.h"
#include "factorV3.h"


static constexpr const char LinearTranslationLabel[] = "linear translation";
static constexpr const char observee_var[]           = "observee";
static constexpr const char observer_var[]           = "observer";
class LinearTranslationFactor
    : public FactorV3<LinearTranslationFactor,
                      LinearTranslationLabel,
                      MetaMeasureLinearTranslation_t,
                      StrTie<observee_var, MetaKeyPosition_t>,
                      StrTie<observer_var, MetaKeyPosition_t>>
{
    public:
  LinearTranslationFactor(const std::string&                               factor_id,
           const measure_vect_t&                            mes_vect,
           const measure_cov_t&                             measure_cov,
           const std::array<std::string, LinearTranslationFactor::kNbKeys >& keys_id):
  FactorV3(factor_id,
           mes_vect,
           measure_cov,
           keys_id)
  {
  }
  const Eigen::Matrix2d mymat {{1, 2}, {3, 4}};
};


#endif
