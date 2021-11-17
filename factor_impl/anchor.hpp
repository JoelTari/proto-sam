#ifndef FACTOR_IMPL_ANCHOR_H_
#define FACTOR_IMPL_ANCHOR_H_

#include "key-meta-position.h"
#include "measure-meta-absolute-position.h"
#include "factorV3.h"

// factor instantiation from templates
static constexpr const char anchorLabel[] = "anchor";
static constexpr const char anchor_var[]  = "unique var";
class AnchorFactor
    : public FactorV3<AnchorFactor,
                      anchorLabel,
                      MetaMeasureAbsolutePosition_t,
                      StrTie<anchor_var, MetaKeyPosition_t>>
{
    public:
  AnchorFactor(const std::string&                               factor_id,
           const measure_vect_t&                            mes_vect,
           const measure_cov_t&                             measure_cov,
           const std::array<std::string, AnchorFactor::kNbKeys >& keys_id):
  FactorV3(factor_id,
           mes_vect,
           measure_cov,
           keys_id)
  {
  }
  const Eigen::Matrix2d mymat {{1, 2}, {3, 4}};
};


#endif
