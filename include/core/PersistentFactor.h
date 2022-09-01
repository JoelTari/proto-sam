#pragma once

#include "core/config.h"
#include <vector>
#include "core/factor.h"

namespace sam::Factor
{

// all the factor data at one specefic point
template <typename FACTOR_T>
struct PersistentFactorData
{
  using matrices_Aik_t = typename FACTOR_T::matrices_Aik_t;
  using vector_bi_t = typename FACTOR_T::criterion_t;

  matrices_Aik_t Aiks;
  vector_bi_t bi;
  double norm;

  PersistentFactorData(const matrices_Aik_t & Aiks, const vector_bi_t & bi,
      double norm)
    : Aiks(Aiks), bi(bi), norm(norm)
  {
#if ENABLE_DEBUG_TRACE
    std::cout << "persistent data\n";
    std::cout << "bi : " << '\n'; 
    std::cout << bi      << '\n';
#endif
  }

};








  
// a wrapper for factor, with storing informations about a lin point
template <typename FACTOR_T>
class WrapperPersistentFactor
{
  public:
  using Factor_t = FACTOR_T;
  using composite_state_ptr_t = typename FACTOR_T::composite_state_ptr_t;
  using composite_of_opt_state_ptr_t = typename FACTOR_T::composite_of_opt_state_ptr_t;
  using measure_t = typename FACTOR_T::measure_t; 
  using measure_cov_t = typename FACTOR_T::measure_cov_t; 
  using matrices_Aik_t = typename FACTOR_T::matrices_Aik_t;


  WrapperPersistentFactor(const std::string&                      factor_id,
               const measure_t&                        z,
               const measure_cov_t&                    z_cov,
               const std::array<std::string, FACTOR_T::kNbKeys>& keys_id,
               const composite_state_ptr_t & tup_init_points_ptr)
    : factor(FACTOR_T(factor_id,z,z_cov,keys_id))
      ,state_point_view(tup_init_points_ptr)
      ,persistentData_(this->compute_persistent_data())
  {
    // print init state
#if ENABLE_DEBUG_TRACE
    std::cout << stringify_factor_blockliner(this->factor) << '\n';
    if constexpr (!FACTOR_T::isLinear)
    {
      // if all the state of the init pointer are VALID pointers
      if (   
          std::apply(
            [&tup_init_points_ptr](auto &&...Xptr )
            {
              return ( (Xptr != nullptr) && ...);
            }
            ,tup_init_points_ptr
          )
        )
      {
        std::cout << "init point(s) : \n";
        std::cout << 
        stringify_composite_state_blockliner(tup_init_points_ptr, this->keys_id,4,4);
      }
      else
      {
        std::cout << "no init point(s) given. \n";
      }
      std::cout << "------------ \n ";
    }
#endif
  }
    
  // shared with other structures in system
  const composite_state_ptr_t state_point_view;

  // factor (the wrapped thing)
  FACTOR_T factor;

  // state point has changed (e.g. the system updates it), the data must be updated
  auto compute_persistent_data() const
  {
    // TODO: rename the timer title of this scope URGENT:
    PROFILE_SCOPE( this->factor.factor_id.c_str() ,sam_utils::JSONLogger::Instance());
    // WARNING: not protected against race condition on the current state X
    // TODO:  dont recompute if factor AND system are linear (syst linear => template argument)
#if ENABLE_DEBUG_TRACE
    std::cout << "compute persistent data at creation\n";
    std::cout << stringify_factor_blockliner(this->factor);
#endif
    auto [bj,Ajks] = this->factor.compute_Ai_bi_at(this->state_point_view);
    double norm = this->factor.factor_norm_at(this->state_point_view);

    // Possibly later compute r(X) too. 
    // But note that it norm_at(X) computes r(X) first, which is redundant
    return PersistentFactorData<FACTOR_T>(Ajks,bj,norm);
  }

  PersistentFactorData<FACTOR_T> get_current_point_data() const
  {
    return persistentData_;
  }

  std::vector<double> norm_history;

    // // copy the current point, push it in history
    // // (do this before the current point gets changed/replaced)
    // void push_current_norm_history()
    // {
    //   composite_state_ptr_t copy =
    //     std::apply(
    //         [](const auto & ...current_point_ptr)
    //         {
    //           return std::make_tuple( std::make_shared<decltype(*current_point_ptr)>(*current_point_ptr) ... );
    //         }
    //         ,this->state_point_view
    //         );
    //   
    //   this->norm_history.push_back(copy);
    // }

    void clear_history()
    {
      this->norm_history.clear();
    }

    void set_persistent_data(const PersistentFactorData<FACTOR_T> & persistent_data)
    {
      persistentData_ = persistent_data;
    }

  private:
   PersistentFactorData<FACTOR_T> persistentData_;

};

// // specialize for euclidian, linear ?
// template <typename FT>
// class WrapperPersistentLinearFactor : public WrapperPersistentFactor<FT>
// {
//   static_assert(FT::isLinear);
//   // ctor inherited
//   using WrapperPersistentFactor::WrapperPersistentFactor;
//
//   auto compute_Ai_bi_linear()
//   {
//     return this->factor.compute_Ai_bi_linear();
//   }
// };

} // end namespace
