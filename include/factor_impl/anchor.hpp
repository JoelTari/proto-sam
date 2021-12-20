#ifndef FACTOR_IMPL_ANCHOR_H_
#define FACTOR_IMPL_ANCHOR_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/measure-meta-absolute-position.h"

namespace
{
  // factor instantiation from templates
  // instantiate the (unique) key conduct
  inline static constexpr const char anchor_var[] = "unique var";
  struct UniqueKeyConduct
      : KeyContextualConduct<UniqueKeyConduct,
                             MetaKeyPosition_t,
                             MetaMeasureAbsolutePosition_t::kM,
                             anchor_var,
                             true>
  {
    inline static const process_matrix_t partH {
        {1, 0},
        {0, 1}};   // cant make it constexpr, but it's probably still compile time
    const process_matrix_t partA;

    process_matrix_t compute_part_A_impl() const
    {
      return partA;   // since it is linear, no need to do anything
    }

    UniqueKeyConduct(const std::string key_id, const measure_cov_t& rho)
        : KeyContextualConduct(key_id, rho)
        , partA(rho * partH)   // rho*H
    {
    }
  };
}   // namespace

namespace
{
  inline static constexpr const char anchorLabel[] = "anchor";
  class AnchorFactor
      : public Factor<AnchorFactor, anchorLabel, MetaMeasureAbsolutePosition_t, UniqueKeyConduct>
  {
    public:
    AnchorFactor(const std::string&                                    factor_id,
                 const measure_vect_t&                                 mes_vect,
                 const measure_cov_t&                                  measure_cov,
                 const std::array<std::string, AnchorFactor::kNbKeys>& keys_id)
        : Factor(factor_id, mes_vect, measure_cov, keys_id)
    {
#if ENABLE_DEBUG_TRACE
      std::cout << "\t::  Factor " << factor_id << " created.\n";
#endif
    }

    // init point guesser
    static std::optional<std::tuple<UniqueKeyConduct::part_state_vect_t>>
        guess_init_key_points_impl(
            const std::tuple<std::optional<UniqueKeyConduct::part_state_vect_t>>&
                                  x_init_optional_tup,
            const measure_vect_t& z)
    {
      // NOTE: easy for this factor, if the mean
      if (std::get<0>(x_init_optional_tup).has_value())
        // if the optional mean value inside the tuple is given, we report this value as initial
        // guess
        return std::make_tuple(std::get<0>(x_init_optional_tup).value());
      else   // unable to determine an init point for the only key
        return std::nullopt;
    }

    measure_vect_t compute_h_of_x_impl(const state_vector_t& x) const
    {
      return process_matrix_t {{1, 0}, {0, 1}} * x;
    }

    private:
    // defined at ctor
    const process_matrix_t roach
        = rho * process_matrix_t {{1, 0}, {0, 1}};   // TODO: defined w.r.t to the partH of keyset
  };

}   // namespace


#endif
