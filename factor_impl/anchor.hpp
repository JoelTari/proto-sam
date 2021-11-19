#ifndef FACTOR_IMPL_ANCHOR_H_
#define FACTOR_IMPL_ANCHOR_H_

#include "config.h"
#include "factor.h"
#include "key-meta-position.h"
#include "measure-meta-absolute-position.h"

namespace
{
// factor instantiation from templates
// instantiate the (unique) key conduct
inline static constexpr const char anchor_var[] = "unique var";
struct UniqueKeyConduct
    : KeyContextualConduct<UniqueKeyConduct,
                           MetaKeyPosition_t,
                           MetaMeasureAbsolutePosition_t::kM,
                           anchor_var>
{
  inline static const process_matrix_t H {
      {1, 0},
      {0, 1}};   // cant make it constexpr, but it's probably still compile time
  const process_matrix_t partA;

  process_matrix_t compute_part_A_impl() const
  {
    return partA;   // since it is linear, no need to do anything
  }
  
  measure_vect_t compute_part_h_of_part_x_impl(const part_state_vect_t & part_x)
  {
    // OPTIMIZE: this is the same for every linear KeyCC
    return H*part_x;
  }

  UniqueKeyConduct(const std::string key_id, const measure_cov_t& rho)
      : KeyContextualConduct(key_id, rho)
      , partA(rho * H)
  {
  }
};
}

namespace{
inline static constexpr const char anchorLabel[] = "anchor";
class AnchorFactor
    : public Factor<AnchorFactor,
                    anchorLabel,
                    MetaMeasureAbsolutePosition_t,
                    UniqueKeyConduct>
{
  public:
  AnchorFactor(const std::string&    factor_id,
               const measure_vect_t& mes_vect,
               const measure_cov_t&  measure_cov,
               const std::array<std::string, AnchorFactor::kNbKeys>& keys_id)
      : Factor(factor_id, mes_vect, measure_cov, keys_id)
  {
#if ENABLE_DEBUG_TRACE
    std::cout << "\t::  Factor " << factor_id << " created.\n";
#endif
  }
};

}


#endif
