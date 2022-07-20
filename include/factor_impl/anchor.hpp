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

    // for NL cases
    UniqueKeyConduct(const std::string key_id, const measure_cov_t& rho,std::shared_ptr<part_state_vect_t> init_point)
        : KeyContextualConduct(key_id, rho, init_point)
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
      // TODO: remove parent_t
    using parent_t = Factor<AnchorFactor, anchorLabel, MetaMeasureAbsolutePosition_t, UniqueKeyConduct>;

    AnchorFactor(const std::string&                                    factor_id,
                 const criterion_t&                                 mes_vect,
                 const measure_cov_t&                                  measure_cov,
                 const std::array<std::string, AnchorFactor::kNbKeys>& keys_id,
                 std::tuple<  std::shared_ptr<UniqueKeyConduct::part_state_vect_t>> tuple_of_init_point_ptrs)
        : Factor(factor_id, mes_vect, measure_cov, keys_id, tuple_of_init_point_ptrs)
    {
#if ENABLE_DEBUG_TRACE
      std::cout << "\t::  Factor " << factor_id << " created.\n";
#endif
    }

    // init point guesser
    static std::optional<std::tuple< std::shared_ptr<UniqueKeyConduct::part_state_vect_t> >>
        guess_init_key_points_impl(
            std::tuple<std::optional<  std::shared_ptr<UniqueKeyConduct::part_state_vect_t> >>
                                  x_init_ptr_optional_tup,
            const criterion_t& z)
    {
      if (std::get<0>(x_init_ptr_optional_tup).has_value())
        // if the optional mean value inside the tuple is given, we report this value as initial
        // guess
        return std::make_tuple(std::get<0>(x_init_ptr_optional_tup).value());
      else   // make a new state in the heap from the measurement
      {
        // measure_vect_t and part_state_vect_t are the same (for this specific factor)
        static_assert(std::is_same_v<UniqueKeyConduct::part_state_vect_t, criterion_t>);
        auto xinit_ptr = std::make_shared<UniqueKeyConduct::part_state_vect_t>(z);
        return std::make_tuple(xinit_ptr);
      }
      // NOTE: never returns std::nullopt
    }

    criterion_t compute_h_of_x_impl(const state_vector_t& x) const
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
