#ifndef FACTOR_IMPL_ANCHOR_H_
#define FACTOR_IMPL_ANCHOR_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/measure-meta-absolute-position.h"

namespace details_sam::Factor {
  namespace Anchor2dImpl{
    // factor instantiation from templates
    // instantiate the (unique) key conduct
    inline static constexpr const char anchor_role_str[] = "prior";
    
    // the matrix Hik, an improvement would be to use constexpr raw array and then transform in Matrix in the class, somehow.
    inline static const Eigen::Matrix<double,2,2> Hik_UniqueKeyConduct {{1,0},{0,1}};
    // HACK: matrix is passed in-template as the address of the above declaration

    using namespace ::sam::Meta;

    using UniqueKeyConduct = 
      LinearKeyContextualConduct
          < 
            Key::Position2d
            ,Measure::AbsolutePosition2d
            ,anchor_role_str
            ,&Hik_UniqueKeyConduct
          >;

    inline static constexpr const char anchorLabel[] = "anchor 2d";

    inline namespace exports{

      class Anchor2d
          : public LinearEuclidianFactor<Anchor2d, anchorLabel, Measure::AbsolutePosition2d, UniqueKeyConduct>
      {
        public:
        using BaseFactor_t = LinearEuclidianFactor<Anchor2d, anchorLabel, Measure::AbsolutePosition2d, UniqueKeyConduct>;
        friend BaseFactor_t;
        static_assert(std::is_same_v<UniqueKeyConduct::key_process_matrix_t, factor_process_matrix_t>  ); // because only 1 key
        static_assert(std::is_same_v<UniqueKeyConduct::Key_t, criterion_t>); // because linear factor &&  size M = size N

        Anchor2d(const std::string&                                    factor_id,
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
                // std::tuple<std::optional<  std::shared_ptr<UniqueKeyConduct_t::Key_t> >>
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
      };
    }  // namespace exports
  }  // namespace impl
} // namespace details_sam::factor

// exposing only relevant structure to client
namespace sam::Factor{
  using namespace details_sam::Factor::Anchor2dImpl::exports;
}

#endif
