#ifndef SAM_MARGINAL_H_
#define SAM_MARGINAL_H_

#include "core/meta.h"
#include "utils/tuple_patterns.h"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>

namespace
{
  template <typename KEYMETA>
  class Marginal : KEYMETA
  {
    public:
    using KeyMeta_t = KEYMETA;
    using Mean_t = Eigen::Vector<double, KEYMETA::kN>;
    // bool marked = false;
    Mean_t              mean;
    Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN> covariance;

    std::tuple<std::array<double, 2>, double> get_visual_2d_covariance() const
    {
      // true = compute the eigenvectors too (default is true anyway)
      // Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN>>    es(covariance, true);   
      // std::cout << " The marginal cov:\n" << covariance << '\n';
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN>>    es(covariance);   
      std::array<double, 2> sigma {sqrt(es.eigenvalues()[0]), sqrt(es.eigenvalues()[1])};
      auto                  R = es.eigenvectors();

      double rot = std::atan2(R(1, 0), R(0, 0));

      return {sigma, rot};
    }
    // using type = typename Marginal<KEYMETA>;
    // TODO: a flag ?
    Marginal(const Eigen::Vector<double, KEYMETA::kN>&              xmap_marg,
             const Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN>& cov_marg)
        : mean(xmap_marg)
        , covariance(cov_marg) {};
  };


  // template<typename MARGINAL_T, typename ... MARGINAL_Ts>
  // TODO: compile check against duplicate keymeta
  template <typename KEYMETA_T, typename... KEYMETA_Ts>
  class MarginalsContainer
  {
    public:
    using type = MarginalsContainer<KEYMETA_T, KEYMETA_Ts...>;

    using marginals_containers_t
        = std::tuple<std::unordered_map<std::string, Marginal<KEYMETA_T>>,
                     std::unordered_map<std::string, Marginal<KEYMETA_Ts>>...>;
    static constexpr const std::size_t kNbMarginals {
        std::tuple_size_v<marginals_containers_t>};   // sizeof...KEYMETA_Ts + 1

    template <typename Q_KEYMETA_T>
    std::optional<Marginal<Q_KEYMETA_T>>
        findt(const std::string& key_id) 
    {
      // static assert
      static_assert(std::is_same_v<Q_KEYMETA_T,
                                   KEYMETA_T> || (std::is_same_v<Q_KEYMETA_T, KEYMETA_Ts> || ...));

      // get the correct tuple element
      constexpr std::size_t I = get_correct_tuple_idx<Q_KEYMETA_T>();

      // OPTIMIZE: pass a reference
      if (auto it {std::get<I>(this->data_map_tuple).find(key_id)};
          it != std::end(std::get<I>(this->data_map_tuple)))
      { return it->second; }
      else
        return std::nullopt;
    }

    template <typename Q_KEYMETA_T>
    std::optional<typename Marginal<Q_KEYMETA_T>::Mean_t>
        find_mean(const std::string& key_id) 
    {
      // static assert
      static_assert(std::is_same_v<Q_KEYMETA_T,
                                   KEYMETA_T> || (std::is_same_v<Q_KEYMETA_T, KEYMETA_Ts> || ...));

      // get the correct tuple element
      constexpr std::size_t I = get_correct_tuple_idx<Q_KEYMETA_T>();

      // OPTIMIZE: pass a reference
      if (auto it {std::get<I>(this->data_map_tuple).find(key_id)};
          it != std::end(std::get<I>(this->data_map_tuple)))
      { return it->second.mean; }
      else
        return std::nullopt;
    }

    template <typename Q_MARG_T>
    void insertt(const std::string & key_id, const Q_MARG_T & marg) // TODO: why not by idx too
    {
      // static assert the size of vect/cov
      constexpr std::size_t I = get_correct_tuple_idx_by_marg<Q_MARG_T>();
      // std::cout << "correct margcont idx : " << I << '\n';
      std::get<I>(this->data_map_tuple).insert({key_id,marg});
    }

    template <typename Q_KEYMETA_T>
    void insert(const std::string&                            key_id,
                const Eigen::Vector<double, Q_KEYMETA_T::kN>& xmap_marg,
                const Eigen::Matrix<double, Q_KEYMETA_T::kN, Q_KEYMETA_T::kN>&
                    sigmacov)   // TODO:  use perfect forwarding
    {
      Marginal<Q_KEYMETA_T> my_marg(xmap_marg, sigmacov);
      // static assert the size of vect/cov
      constexpr std::size_t I = get_correct_tuple_idx<Q_KEYMETA_T>();
      // std::cout << "correct margcont idx : " << I << '\n';
      std::get<I>(this->data_map_tuple).insert({key_id, my_marg});
    }


    marginals_containers_t data_map_tuple;

    protected:
    /**
     * @brief get an idx in a tuple statically (recursive until the meta matches)
     *
     * @tparam Q_MARGINAL_T
     * @tparam I
     *
     * @return
     */
    template <typename Q_KEYMETA_T, std::size_t I = 0>
    static constexpr std::size_t get_correct_tuple_idx()
    {
      static_assert(I < kNbMarginals);
      constexpr std::size_t Res = 0;
      if constexpr (std::is_same_v<typename std::tuple_element_t<I, marginals_containers_t>::
                                       mapped_type::KeyMeta_t,
                                   Q_KEYMETA_T>)   // maybe thats the keymeta that need compare
      { return I; }
      else
      {
        return get_correct_tuple_idx<I + 1>();
      }
    }
    template <typename Q_MARG_T, std::size_t I = 0>
    static constexpr std::size_t get_correct_tuple_idx_by_marg()
    {
      static_assert(I < kNbMarginals);
      constexpr std::size_t Res = 0;
      if constexpr (std::is_same_v<typename std::tuple_element_t<I, marginals_containers_t>::
                                       mapped_type,
                                   Q_MARG_T>)   // maybe thats the keymeta that need compare
      { return I; }
      else
      {
        return get_correct_tuple_idx_by_marg<I + 1>();
      }
    }
  };

  // specialization: if tuple of marginals is given, then extract whats inside the tuple and
  // fallback to the struct above
  template <typename KEYMETA_T, typename... KEYMETA_Ts>
  class MarginalsContainer<std::tuple<KEYMETA_T, KEYMETA_Ts...>>
      : public MarginalsContainer<KEYMETA_T, KEYMETA_Ts...>   // WOW !!
  {
  };
  template <typename KEYMETA_T>
  class MarginalsContainer<std::tuple<KEYMETA_T>> : public MarginalsContainer<KEYMETA_T>   // WOW !!
  {
  };

}   // namespace

#endif
