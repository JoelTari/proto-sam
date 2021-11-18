#ifndef FACTOR_IMPL_ANCHOR_H_
#define FACTOR_IMPL_ANCHOR_H_

#include "factorV3.h"
#include "key-meta-position.h"
#include "measure-meta-absolute-position.h"

// factor instantiation from templates
// instantiate the (unique) key conduct
static constexpr const char anchor_var[] = "unique var";
struct UniqueKeyConduct
    : KeyContextualConduct<UniqueKeyConduct,
                           MetaKeyPosition_t,
                           MetaMeasureAbsolutePosition_t::kM,
                           anchor_var>
{
  const process_matrix_t H {{1, 0}, {0, 1}};

  process_matrix_t compute_part_A_impl()
  {
    process_matrix_t partA;
    // TODO: partA = rho*H which is constant (but not constexpr)
    // TODO: need to get rho here

    return A;
  }
};

static constexpr const char anchorLabel[] = "anchor";
class AnchorFactor
    : public FactorV3<AnchorFactor,
                      anchorLabel,
                      MetaMeasureAbsolutePosition_t,
                      UniqueKeyConduct>
{
  public:
  AnchorFactor(const std::string&    factor_id,
               const measure_vect_t& mes_vect,
               const measure_cov_t&  measure_cov,
               const std::array<std::string, AnchorFactor::kNbKeys>& keys_id)
      : FactorV3(factor_id, mes_vect, measure_cov, keys_id)
  {
  }
  // const Eigen::Matrix2d mymat {{1, 2}, {3, 4}};
};


#endif
