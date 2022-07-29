#ifndef FACTOR_IMPL_ANCHOR_H_
#define FACTOR_IMPL_ANCHOR_H_

#include "core/config.h"
#include "core/factor.h"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/measure-meta-absolute-position.h"

namespace __UniqueKeyConduct
{
  // factor instantiation from templates
  // instantiate the (unique) key conduct
  inline static constexpr const char anchor_role[] = "unique var";
  // inline static const  Eigen::Matrix<double,2,2> H_anchor{{1,0},{0,1}};
  inline static constexpr std::size_t dimMes = MetaMeasureAbsolutePosition_t::kM;
  inline static constexpr std::size_t kN = MetaKeyPosition_t::kN;
  
  // the matrix Hik, an improvement would be to use constexpr raw array and then transform in Matrix in the class, somehow.
  inline static const Eigen::Matrix<double,2,2> Hik_UniqueKeyConduct {{1,0},{0,1}};
  // HACK: matrix is passed in-template as the address of the above declaration
  using UniqueKeyConduct_t = LinearKeyContextualConduct<MetaKeyPosition_t,dimMes,anchor_role,&Hik_UniqueKeyConduct>;

  // struct UniqueKeyConduct_t
  //   : LinearKeyContextualConduct
  //       <
  //         UniqueKeyConduct_t
  //         , MetaKeyPosition_t
  //         , dimMes
  //         , anchor_role
  //         // , std::array<double,kN>{1,0}
  //         // , std::array<double,kN>{0,1}
  //       >
  //   {
  //     inline static key_process_matrix_t get_Hik_impl()
  //     {
  //       return key_process_matrix_t{{1,0},{0,1}};
  //     }
  //     // ctors (boring): 
  //     using BaseLinearKcc_t = LinearKeyContextualConduct<UniqueKeyConduct_t,MetaKeyPosition_t,dimMes,anchor_role>;
  //     UniqueKeyConduct_t(const std::string& key_id,const measure_cov_t& rho)
  //       : BaseLinearKcc_t(key_id,rho){}
  //     UniqueKeyConduct_t ( const std::string& key_id ,const measure_cov_t& rho ,std::shared_ptr<Key_t> init_point_view)
  //       : BaseLinearKcc_t(key_id,rho,init_point_view) {}
  //   };
    
}   // namespace
using UniqueKeyConduct_t = __UniqueKeyConduct::UniqueKeyConduct_t;

namespace
{
  inline static constexpr const char anchorLabel[] = "anchor";
  class AnchorFactor
      : public LinearEuclidianFactor<AnchorFactor, anchorLabel, MetaMeasureAbsolutePosition_t, UniqueKeyConduct_t>
  {
    public:
    using BaseFactor_t = LinearEuclidianFactor<AnchorFactor, anchorLabel, MetaMeasureAbsolutePosition_t, UniqueKeyConduct_t>;
    friend BaseFactor_t;
    static_assert(std::is_same_v<UniqueKeyConduct_t::key_process_matrix_t, factor_process_matrix_t>  ); // because only 1 key
    static_assert(std::is_same_v<UniqueKeyConduct_t::Key_t, criterion_t>); // because linear factor &&  size M = size N

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
    static std::optional<std::tuple< std::shared_ptr<UniqueKeyConduct_t::Key_t> >>
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
        auto xinit_ptr = std::make_shared<UniqueKeyConduct_t::Key_t>(z);
        return std::make_tuple(xinit_ptr);
      }
      // NOTE: it never returns std::nullopt, that's normal in this situation
    }
  };

}   // namespace


#endif
