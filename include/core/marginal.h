#ifndef SAM_MARGINAL_H_
#define SAM_MARGINAL_H_

#include "core/meta.h"
#include "eigen3/Eigen/Dense"
#include "utils/tuple_patterns.h"

#include <cmath>
#include <string>
#include <optional>
#include <type_traits>

namespace 
{
template <typename KEYMETA>
class Marginal
{
  public:
  using KeyMeta_t = KEYMETA;
  // bool marked = false;
  Eigen::Vector<double, KEYMETA::kN>              mean;
  Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN> covariance;

  std::tuple<std::array<double, 2>, double> get_visual_2d_covariance() const
  {
    Eigen::EigenSolver    es(covariance,
                          true);   // true = compute the eigenvectors too (default is true anyway)
    std::array<double, 2> sigma {sqrt(es.eigenvalues()[0]), sqrt(es.eigenvalues()[1])};
    auto                  R = es.eigenvectors();

    double rot = std::atan2(R(1, 0), R(0, 0));

    return {sigma, rot};
  };

  // void mark(){marked=true;}
  // void unmark(){marked=false;}
};



template<typename MARGINAL_T, typename ... MARGINAL_Ts> 
class MarginalsContainer
{
  using marginals_containers_t = std::tuple<MARGINAL_T, MARGINAL_Ts...>;
  public:

    template<typename Q_MARGINAL_T>
    std::optional<Q_MARGINAL_T> find(const std::string& key_id)   // OPTIMIZE: std::optional<Q_MARGINAL_T&>
    {
      // static assert
      
      // get the correct tuple element
      constexpr size_t I = get_correct_tuple_idx<Q_MARGINAL_T>();

      // OPTIMIZE: pass a reference
      if (auto it {std::get<I>(data_map_tuple).find(key_id)};
        it != std::end(this->key_to_infos))
      {
          return it->second;
      }
      else
        return std::nullopt;
    }

    // void unmark_all();


  private:
    marginals_containers_t data_map_tuple;
    static constexpr const std::size_t kNbMarginals  {std::tuple_size_v<marginals_containers_t>};

    /**
    * @brief get an idx in a tuple statically (recursive until the meta matches)
    *
    * @tparam Q_MARGINAL_T
    * @tparam I
    *
    * @return 
    */
    template<typename Q_MARGINAL_T,std::size_t I = 0> 
    static constexpr std::size_t get_correct_tuple_idx()
    {
      static_assert(I < kNbMarginals);
      constexpr std::size_t Res = 0;
      if constexpr ( std::is_same_v<std::tuple_element_t<I,marginals_containers_t>,Q_MARGINAL_T> ) // maybe thats the keymeta that need compare
      {
          return I;
      }
      else
      {
          return get_correct_tuple_idx<I+1>();        
      }
    }

};

// specialization: if tuple of marginals is given, then extract whats inside the tuple and fallback to the struct above
template<typename MARGINAL_T, typename ... MARGINAL_Ts> 
class MarginalsContainer<std::tuple<MARGINAL_T,MARGINAL_Ts...> > :  MarginalsContainer<MARGINAL_T,MARGINAL_Ts...> // WOW !!
{};
  
}

#endif


