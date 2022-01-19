#ifndef SAM_MARGINAL_H_
#define SAM_MARGINAL_H_

#include "core/meta.h"
#include "utils/tuple_patterns.h"

#include <cmath>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>

namespace
{
  template <typename KEYMETA>
  class Marginal : KEYMETA    // Rename as guassin
  {
    public:
    using KeyMeta_t = KEYMETA;
    using Mean_t = Eigen::Vector<double, KEYMETA::kN>;
    using Mean_t_ptr = std::shared_ptr<Mean_t>;
    using Covariance_t = Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN>;
    using Sigmas_t = std::array<double, 2>;
    using VisualCovariance_t = std::tuple<Sigmas_t, double>;
    // bool marked = false;
    Mean_t_ptr              mean_ptr;
    Covariance_t covariance;
    

    // ctor
    Marginal(Mean_t_ptr mean_ptr,const Covariance_t& covariance = Covariance_t::Zero())
        : mean_ptr(mean_ptr)
        , covariance(covariance) 
    {};

    // visual covariance 
    VisualCovariance_t get_visual_2d_covariance() const // WARNING: works only for 2d
    {
      // true = compute the eigenvectors too (default is true anyway)
      // Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN>>    es(covariance, true);   
      // std::cout << " The marginal cov:\n" << covariance << '\n';
      Eigen::SelfAdjointEigenSolver<Covariance_t>    es(covariance);   
      std::array<double, 2> sigma {sqrt(es.eigenvalues()[0]), sqrt(es.eigenvalues()[1])};
      auto                  R = es.eigenvectors();

      double rot = std::atan2(R(1, 0), R(0, 0));

      return {sigma, rot};
    }
  };

  template <typename MARGINAL_T>
  struct MarginalHistory
  {
      using Marginal_t = MARGINAL_T;
      std::string var_id;
      std::vector<typename Marginal_t::Mean_t> iterative_means;
      std::vector<typename Marginal_t::VisualCovariance_t> iterative_covariances;

      MarginalHistory(  const std::string& var_id
                      , const typename Marginal_t::Mean_t & mean
                      , const typename Marginal_t::VisualCovariance_t & visual_covariance)
      :
        var_id(var_id),iterative_means({mean}),iterative_covariances({visual_covariance})
      {}

  };

  template <typename KEYMETA_T, typename... KEYMETA_Ts>
  struct MarginalsHistoriesContainer
  {
    using marginals_histories_t
        = std::tuple<std::unordered_map<std::string, MarginalHistory<Marginal<KEYMETA_T>>>,
                     std::unordered_map<std::string, MarginalHistory<Marginal<KEYMETA_Ts>>>...>; 
    
    marginals_histories_t marginal_history_tuple;

    static constexpr const std::size_t kNbMarginals { std::tuple_size_v<marginals_histories_t>};

    template <typename Q_MARG_T>
    void insert_new_marginal(const std::string & key_id, const std::shared_ptr<Q_MARG_T> marginal_ptr)
    {
      using KM_T = typename Q_MARG_T::KeyMeta_t;
      static_assert(std::is_same_v<KM_T,KEYMETA_T> || (std::is_same_v<KM_T,KEYMETA_Ts> || ...)  );

      constexpr std::size_t I = get_correct_tuple_idx_by_marg<KM_T>();

      // Create object marginal history object
      auto marginal_history = MarginalHistory<Q_MARG_T>(
                                                         key_id
                                                       , *marginal_ptr->mean_ptr
                                                       , marginal_ptr->get_visual_2d_covariance()
                                                       );

      std::get<I>(this->marginal_history_tuple).insert_or_assign(key_id, marginal_history ); // TODO: manage failure
    }
    
    template <typename Q_MARG_T>
    void push_marginal_history(const std::string & key_id, const std::shared_ptr<Q_MARG_T> marginal_ptr)
    {
      using KM_T = typename Q_MARG_T::KeyMeta_t;
      static_assert(std::is_same_v<KM_T,KEYMETA_T> || (std::is_same_v<KM_T,KEYMETA_Ts> || ...)  );

      constexpr std::size_t I = get_correct_tuple_idx_by_marg<KM_T>();

      // get marginal history ref
      auto marginal_history_it = std::get<I>(this->marginal_history_tuple).find(key_id);
      // TODO: assert(marginal_history_it != std::get<I>(this->marginal_history_tuple).end() );
      // push new data
      marginal_history_it->second.iterative_means.push_back( *(marginal_ptr->mean_ptr) );
      marginal_history_it->second.iterative_covariances.push_back( marginal_ptr->get_visual_2d_covariance() );

    }


    template <typename KM_T, std::size_t I = 0>
    static constexpr std::size_t get_correct_tuple_idx_by_marg()
    {
      static_assert(I < kNbMarginals);
      if constexpr (std::is_same_v<typename std::tuple_element_t<I, marginals_histories_t>::
                                       mapped_type::Marginal_t::KeyMeta_t, KM_T>)   // maybe thats the keymeta that need compare
      { return I; }
      else
      {
        return get_correct_tuple_idx_by_marg<KM_T,I + 1>();
      }
    }

  };

  template <typename KEYMETA_T, typename... KEYMETA_Ts>
  class MarginalsContainer
  {
    public:
    using type = MarginalsContainer<KEYMETA_T, KEYMETA_Ts...>;

    using marginals_containers_t
        = std::tuple<std::unordered_map<std::string, std::shared_ptr<Marginal<KEYMETA_T>>>,
                     std::unordered_map<std::string, std::shared_ptr<Marginal<KEYMETA_Ts>>>...>;

    static constexpr const std::size_t kNbMarginals { std::tuple_size_v<marginals_containers_t>};

    template <typename Q_KEYMETA_T>
    std::optional<std::shared_ptr<Marginal<Q_KEYMETA_T>>>
        find_marginal_ptr(const std::string& key_id) 
    {
      // static assert
      static_assert(std::is_same_v<Q_KEYMETA_T,
                                   KEYMETA_T> || (std::is_same_v<Q_KEYMETA_T, KEYMETA_Ts> || ...));

      // get the correct tuple element
      constexpr std::size_t I = get_correct_tuple_idx<Q_KEYMETA_T>();

      if (auto it {std::get<I>(this->data_map_tuple).find(key_id)};
          it != std::end(std::get<I>(this->data_map_tuple)))
      { return it->second; }
      else
        return std::nullopt;
    }

    template <typename Q_KEYMETA_T>
    std::optional<std::shared_ptr<typename Marginal<Q_KEYMETA_T>::Mean_t>>
        find_mean_ptr(const std::string& key_id) 
    {
      // static assert
      static_assert(std::is_same_v<Q_KEYMETA_T,
                                   KEYMETA_T> || (std::is_same_v<Q_KEYMETA_T, KEYMETA_Ts> || ...));

      // get the correct tuple element
      constexpr std::size_t I = get_correct_tuple_idx<Q_KEYMETA_T>();

      if (auto it {std::get<I>(this->data_map_tuple).find(key_id)};
          it != std::end(std::get<I>(this->data_map_tuple)))
      { return it->second->mean_ptr; }
      else
        return std::nullopt;
    }

    template <typename Q_MARG_T>
    void insert_in_marginal_container(const std::string & key_id, std::shared_ptr<Q_MARG_T> marg_ptr)
    {
      constexpr std::size_t I = get_correct_tuple_idx_by_marg<Q_MARG_T>();
      std::get<I>(this->data_map_tuple).insert_or_assign(key_id, marg_ptr);
    }

    // template <typename Q_MARG_T, typename ...Args>
    // void insertt(const std::string & key_id, const Args &...args)
    // {
    //   // static_assert( std::is_invocable_v<Q_MARG_T,args...> )
    //   // static assert the size of vect/cov
    //   constexpr std::size_t I = get_correct_tuple_idx_by_marg<Q_MARG_T>();
    //   // std::cout << "correct margcont idx : " << I << '\n';
    //   std::get<I>(this->data_map_tuple).insert_or_assign(key_id, std::make_unique<Q_MARG_T>(args...));
    // }

    // // overloads when a covariance is given
    // template <typename Q_KEYMETA_T>
    // void insert(const std::string&                            key_id,
    //             const Eigen::Vector<double, Q_KEYMETA_T::kN>& xmap_marg,
    //             const Eigen::Matrix<double, Q_KEYMETA_T::kN, Q_KEYMETA_T::kN>&
    //                 sigmacov)   // TODO:  use perfect forwarding
    // {
    //   Marginal<Q_KEYMETA_T> my_marg(xmap_marg, sigmacov);
    //   // static assert the size of vect/cov
    //   constexpr std::size_t I = get_correct_tuple_idx<Q_KEYMETA_T>();
    //   // std::cout << "correct margcont idx : " << I << '\n';
    //   std::get<I>(this->data_map_tuple).insert_or_assign(key_id, my_marg);
    // }


    marginals_containers_t data_map_tuple;
    // std::unordered_map<std::string,std::unique_ptr<double>> AAA;
    // static_assert(std::is_same_v<std::unique_ptr<double>::element_type,double>);

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
      // template metaprogramming is still horrible (written in the times of cpp17)
      if constexpr (std::is_same_v<typename std::tuple_element_t<I, marginals_containers_t>::mapped_type::element_type::KeyMeta_t,
                                   Q_KEYMETA_T>)
      { return I; }
      else
      {
        return get_correct_tuple_idx<Q_KEYMETA_T,I + 1>();
      }
    }
    template <typename Q_MARG_T, std::size_t I = 0>
    static constexpr std::size_t get_correct_tuple_idx_by_marg()
    {
      static_assert(I < kNbMarginals);
      if constexpr (std::is_same_v<typename std::tuple_element_t<I, marginals_containers_t>::
                                       mapped_type::element_type,
                                   Q_MARG_T>)   // maybe thats the keymeta that need compare
      { return I; }
      else
      {
        return get_correct_tuple_idx_by_marg<Q_MARG_T,I + 1>();
      }
    }

    // static assert that all KEYMETA are unique  TODO:
    // static_assert( !std::is_same_v<> )
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
  // specialization: if tuple of marginals is given, then extract whats inside the tuple and
  // fallback to the struct above
  template <typename KEYMETA_T, typename... KEYMETA_Ts>
  class MarginalsHistoriesContainer<std::tuple<KEYMETA_T, KEYMETA_Ts...>>
      : public MarginalsHistoriesContainer<KEYMETA_T, KEYMETA_Ts...>   // WOW !!
  {
  };
  template <typename KEYMETA_T>
  class MarginalsHistoriesContainer<std::tuple<KEYMETA_T>> : public MarginalsHistoriesContainer<KEYMETA_T>   // WOW !!
  {
  };

}   // namespace

#endif
