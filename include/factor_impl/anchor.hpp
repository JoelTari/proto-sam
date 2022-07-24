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
    inline static const key_process_matrix_t Hik {
        {1, 0},
        {0, 1}};   // cant make it constexpr, but it's probably still compile time
    const key_process_matrix_t Aik;

    key_process_matrix_t compute_Aik_impl() const
    {
      return Aik;   // since it is linear, no need to do anything
    }

    UniqueKeyConduct(const std::string key_id, const measure_cov_t& rho)
        : KeyContextualConduct(key_id, rho)
        , Aik(rho * Hik)   // rho*H
    {
    }

    // for NL cases
    UniqueKeyConduct(const std::string key_id, const measure_cov_t& rho,std::shared_ptr<Key_t> init_point)
        : KeyContextualConduct(key_id, rho, init_point)
        , Aik(rho * Hik)   // rho*H
    {
    }
  };
}   // namespace

namespace
{
  inline static constexpr const char anchorLabel[] = "anchor";
  class AnchorFactor
      : public TrivialEuclidianFactor<AnchorFactor, anchorLabel, MetaMeasureAbsolutePosition_t, UniqueKeyConduct>
  {
    public:
    using BaseFactor_t = TrivialEuclidianFactor<AnchorFactor, anchorLabel, MetaMeasureAbsolutePosition_t, UniqueKeyConduct>;
    friend BaseFactor_t;
    // using key_process_matrix_t = typename UniqueKeyConduct::key_process_matrix_t;
    // using factor_process_matrix_t = typename parent_t::factor_process_matrix_t;
    // using criterion_t = typename BaseFactor_t::criterion_t;
    // using measure_t = typename BaseFactor_t::measure_t;
    // using measure_cov_t = typename BaseFactor_t::measure_cov_t;
    // using matrices_Aik_t = typename  BaseFactor_t::matrices_Aik_t;
    // using composite_state_ptr_t = typename BaseFactor_t::composite_state_ptr_t;
    // using composite_of_opt_state_ptr_t = typename BaseFactor_t::composite_of_opt_state_ptr_t;
    //
    static_assert(std::is_same_v<UniqueKeyConduct::key_process_matrix_t, factor_process_matrix_t>  ); // because only 1 key
    // static_assert(std::is_same_v<state_vector_t, UniqueKeyConduct::part_state_vect_t>  ); // because only 1 key
    static_assert(std::is_same_v<UniqueKeyConduct::Key_t, criterion_t>); // because linear factor &&  size M = size N

    AnchorFactor(const std::string&                                    factor_id,
                 const criterion_t&                                 mes_vect,
                 const measure_cov_t&                                  measure_cov,
                 const std::array<std::string, kNbKeys>& keys_id,
                 const composite_state_ptr_t & tuple_of_init_point_ptrs)
        : BaseFactor_t(factor_id, mes_vect, measure_cov, keys_id, tuple_of_init_point_ptrs)
    {
#if ENABLE_DEBUG_TRACE
      std::cout << "\t::  Factor " << factor_id << " created.\n";
#endif
    }

    // init point guesser
    static std::optional<std::tuple< std::shared_ptr<UniqueKeyConduct::Key_t> >>
        guess_init_key_points_impl( const composite_of_opt_state_ptr_t &
            // std::tuple<std::optional<  std::shared_ptr<UniqueKeyConduct::Key_t> >>
                                  x_init_ptr_optional_tup,
            const criterion_t& z)
    {
      if (std::get<0>(x_init_ptr_optional_tup).has_value())
        // if the optional mean value inside the tuple is given, we report this value as initial
        // guess
        return std::make_tuple(std::get<0>(x_init_ptr_optional_tup).value());
      else   // make a new state in the heap from the measurement
      {
        auto xinit_ptr = std::make_shared<UniqueKeyConduct::Key_t>(z);
        return std::make_tuple(xinit_ptr);
      }
      // NOTE: it never returns std::nullopt, that's normal in this situation
    }

    private:

    // // making a friend so that we the next implementation method can stay private
    // friend criterion_t BaseFactor_t::compute_h_of_x_impl(const composite_state_ptr_t &X) const;

    criterion_t compute_h_of_x_impl(const composite_state_ptr_t &  Xptr) const
    {
      // Xptr is a single tuple 
      return std::get<0>(this->keys_set).compute_Aik() * *std::get<0>(Xptr);
    }

    // private:
    // // defined at ctor
    // const process_matrix_t roach
    //     = rho * process_matrix_t {{1, 0}, {0, 1}};   // TODO: defined w.r.t to the Hik of keyset
  };

}   // namespace


#endif
