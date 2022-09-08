#pragma

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>

#include <Eigen/Dense>

#include "system/config.h"
#include "meta/meta_interface.h"
#include "utils/tuple_patterns.h"


namespace sam::Marginal
{
  template <typename KEYMETA>
  class BaseMarginal : KEYMETA    // Rename as guassin
  {
    public:
    using KeyMeta_t = KEYMETA;
    using Mean_t = typename KeyMeta_t::key_t;
    using Tangent_Space_t = typename KeyMeta_t::tangent_space_t;
    // using Mean_Distribution_t = Mean_t; // Distribution mean : by default same as Mean_t, but it's the implementer job's to override it
    // using Mean_Distribution_t_ptr = std::shared_ptr<Mean_Distribution_t>; // Distribution mean : by default same as Mean_t, but it's the implementer job's to override it
    // using Mean_t_ptr = std::shared_ptr<Mean_t>;
    using Covariance_t = Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN>;
    using OptCovariance_t = std::optional<Covariance_t>;
    using Sigmas_t = std::array<double, 2>;
    using VisualCovariance_t = std::tuple<Sigmas_t, double>;
    // bool marked = false;
    Mean_t  mean; // TODO: ACTION: mean distribution here ??
    OptCovariance_t covariance;
    

    // ctor
    BaseMarginal(Mean_t mean,const OptCovariance_t& covariance = std::nullopt) // TODO: ACTION: use distribution mean
        : mean(mean)
        , covariance(covariance) 
    {};
  };


  template <typename MARGINAL_T>
  class WrapperPersistentMarginal
  {
    public:
      using Marginal_t = MARGINAL_T;
      using Mean_t = typename Marginal_t::Mean_t;
      // using Mean_t_ptr = typename Marginal_t::Mean_t_ptr;
      using OptCovariance_t = typename Marginal_t::OptCovariance_t;

      WrapperPersistentMarginal(const std::string & marginal_id
          , const std::shared_ptr<Mean_t> & mean_ptr
          , const OptCovariance_t & opt_covariance = std::nullopt)
        : 
          shared_mean(mean_ptr)
          , marginal(*mean_ptr, opt_covariance)
          , marginal_histories({})
      {}
      
     std::vector<MARGINAL_T> marginal_histories;
    
     void save_and_replace(const MARGINAL_T & new_marginal)
     {
       this->marginal_histories.push_back(this->marginal);
       this->marginal = new_marginal;
       *(this->shared_mean) = new_marginal.mean;
     }

     void save_and_add(const typename MARGINAL_T::Tangent_Space_t & tangent_increment, const OptCovariance_t & opt_covariance = std::nullopt)
     {
      this->marginal_histories.push_back(this->marginal);
      this->marginal.mean += tangent_increment;
      this->marginal.covariance = opt_covariance;
       *(this->shared_mean) = this->marginal.mean;
     }

     void clear_history()
     {
       this->marginal_histories.clear();
     }

    private:
     const std::shared_ptr<Mean_t> shared_mean; // seen by other system class (wfactors...)
     MARGINAL_T marginal;
  };



  // /**
  //  * @brief Container of a set of marginals, of their histories
  //  *
  //  * @tparam KEYMETA_T key meta of first marginal type (there must be at least 1 type)
  //  * @tparam KEYMETA_Ts expansion of key metas of the rest of the marginal types
  //  * @param key_id identifier key of the marginal
  //  * @param marginal_ptr pointer to the marginal
  //  */
  // template <typename KEYMETA_T, typename... KEYMETA_Ts>
  // struct MarginalsHistoriesContainer
  // {
  //   // type define a tuple of map of MarginalHistory types of each type of marginal
  //   using marginals_histories_t
  //       = std::tuple<std::unordered_map<std::string, MarginalHistory<BaseMarginal<KEYMETA_T>>>,
  //                    std::unordered_map<std::string, MarginalHistory<BaseMarginal<KEYMETA_Ts>>>...>; 
  //   
  //   // maps (one for each marginal type) of marginal identifier (string) to its history (MarginalHistory<Marginal_T>)
  //   marginals_histories_t marginal_history_tuple;
  //
  //   // number of types of marginals
  //   static constexpr const std::size_t kNbMarginals { std::tuple_size_v<marginals_histories_t>};
  //
  //   /**
  //    * @brief Insert a new marginal into the container
  //    *
  //    * @tparam Q_MARG_T marginal type
  //    * @param key_id identifier key of the marginal
  //    * @param marginal_ptr pointer to the marginal value
  //    */
  //   template <typename Q_MARG_T>
  //   void insert_new_marginal(const std::string & key_id, const std::shared_ptr<Q_MARG_T> marginal_ptr)
  //   {
  //     using KM_T = typename Q_MARG_T::KeyMeta_t;
  //     // assert the inserted marginal type corresponds to existing marginal type in the container
  //     static_assert(std::is_same_v<KM_T,KEYMETA_T> || (std::is_same_v<KM_T,KEYMETA_Ts> || ...)  );
  //
  //     // template programming magic to get the tuple index
  //     constexpr std::size_t TUPLE_IDX = get_correct_tuple_idx_by_marg<KM_T>();
  //
  //     // Create object marginal history object
  //     // The first item is of the history is its current value (mean and cov)
  //     auto marginal_history = MarginalHistory<Q_MARG_T>(
  //                                                        key_id
  //                                                      , *marginal_ptr->mean_ptr // TODO: action : maybe it's mean_distribution type here
  //                                                      , marginal_ptr->get_visual_2d_covariance()
  //                                                      );
  //
  //     // select the correct map structure, and insert/assign the marginal history
  //     std::get<TUPLE_IDX>(this->marginal_history_tuple).insert_or_assign(key_id, marginal_history ); // TODO: manage failure
  //   }
  //   
  //   /**
  //    * @brief Push the moments (mean & cov) of the a marginal whose identifier key already exists 
  //    *
  //    * @tparam Q_MARG_T marginal type
  //    * @param key_id identifier key of the marginal
  //    * @param marginal_ptr point to the marginal value
  //    */
  //   template <typename Q_MARG_T>
  //   void push_marginal_history(const std::string & key_id, const std::shared_ptr<Q_MARG_T> marginal_ptr)
  //   {
  //     using KM_T = typename Q_MARG_T::KeyMeta_t;
  //     // check that we are inserting an existing type
  //     static_assert(std::is_same_v<KM_T,KEYMETA_T> || (std::is_same_v<KM_T,KEYMETA_Ts> || ...)  );
  //
  //     // template programming magic to get the tuple index
  //     constexpr std::size_t TUPLE_IDX = get_correct_tuple_idx_by_marg<KM_T>();
  //
  //     // get marginal history ref
  //     auto marginal_history_it = std::get<TUPLE_IDX>(this->marginal_history_tuple).find(key_id);
  //     // TODO: assert (run time) that the marginal key exists in the map
  //     // TODO: assert(marginal_history_it != std::get<I>(this->marginal_history_tuple).end() );
  //     // push new data
  //     marginal_history_it->second.iterative_means.push_back( *(marginal_ptr->mean_ptr) ); // TODO: ACTION: might be mean_distribution here
  //     marginal_history_it->second.iterative_covariances.push_back( marginal_ptr->get_visual_2d_covariance() );
  //   }
  //
  //
  //   /**
  //    * @brief static function to find the correct index of a marginal meta type in a tuple of various marginals
  //    *
  //    * @tparam KM_T keymeta type
  //    * @return index in tuple
  //    */
  //   template <typename KM_T, std::size_t TUPLE_IDX = 0>
  //   static constexpr std::size_t get_correct_tuple_idx_by_marg()
  //   {
  //     static_assert(TUPLE_IDX < kNbMarginals);
  //     if constexpr (std::is_same_v<typename std::tuple_element_t<TUPLE_IDX, marginals_histories_t>::
  //                                      mapped_type::Marginal_t::KeyMeta_t, KM_T>)
  //     { return TUPLE_IDX; }
  //     else
  //     {
  //       return get_correct_tuple_idx_by_marg<KM_T,TUPLE_IDX + 1>();
  //     }
  //   }
  //
  // };

  //------------------------------------------------------------------//
  //                       MARGINALS CONTAINER                        //
  //------------------------------------------------------------------//
  /**
   * @brief System level structure to hold the marginals
   *
   * @tparam KEYMETA_T first type of marginal (there has to be at least one type of marginal in the system)
   * @tparam KEYMETA_Ts expansion on the other types of marginals
   * @param key_id marginal key identifier
   */
  template <typename KEYMETA_T, typename... KEYMETA_Ts>
  class MarginalsContainer
  {
    public:
    using type = MarginalsContainer<KEYMETA_T, KEYMETA_Ts...>;

    using marginals_data_t
        = std::tuple<
                    std::unordered_map<std::string,  WrapperPersistentMarginal<BaseMarginal<KEYMETA_T>>>,
                    std::unordered_map<std::string, WrapperPersistentMarginal<BaseMarginal<KEYMETA_Ts>>>...
                      >;

    static constexpr std::size_t kNbMarginals { std::tuple_size_v<marginals_data_t>};

    template <typename Q_KEYMETA_T>
    std::optional<WrapperPersistentMarginal<BaseMarginal<Q_KEYMETA_T>>>
        find_wrapped_marginal(const std::string& key_id) const
    {
      // static assert
      static_assert(std::is_same_v<Q_KEYMETA_T,
                                   KEYMETA_T> || (std::is_same_v<Q_KEYMETA_T, KEYMETA_Ts> || ...));

      // get the correct tuple element
      constexpr std::size_t TUPLE_IDX = get_correct_tuple_idx<Q_KEYMETA_T>();

      if (auto it {std::get<TUPLE_IDX>(this->data_map_tuple).find(key_id)};
          it != std::end(std::get<TUPLE_IDX>(this->data_map_tuple)))
      { return it->second; }
      else
        return std::nullopt;
    }

    template <typename Q_WMARG_T>
    void insert_in_marginal_container(const Q_WMARG_T & wmarg)
    {
      constexpr std::size_t TUPLE_IDX = get_correct_tuple_idx_by_wmarg<Q_WMARG_T>();
      std::get<TUPLE_IDX>(this->data_map_tuple).insert_or_assign(wmarg.key_id, wmarg);
      // TODO: run time assertion: verify that the key_id does not exist in other marginal type (e.g. having "x0" as a pose and "x0" as something else in another part of the tuple)
    }


    void clear_histories()
    {
      // WARNING: it's quite slow to iterate a large map
      std::apply([](const auto & ...map_of_wmarg)
          {
            (
             ( clear_map_of_histories(map_of_wmarg) )
             ,...);
          }, this->data_map_tuple);
    }

    template <typename MAP_OF_WMARG>
    static void clear_map_of_histories(const MAP_OF_WMARG & map_of_wmarg)
    {
      for( auto & [id,wmarg] : map_of_wmarg )
        wmarg.clear_history();
    }

    // main structure
    marginals_data_t data_map_tuple;

    protected:
    /**
     * @brief get an idx in a tuple statically using the key meta type  (recursive until the meta matches)
     *
     * @tparam Q_MARGINAL_T
     * @tparam I
     *
     * @return
     */
    template <typename Q_KEYMETA_T, std::size_t TUPLE_IDX = 0>
    static constexpr std::size_t get_correct_tuple_idx()
    {
      static_assert(TUPLE_IDX < kNbMarginals);
      // template metaprogramming is still horrible (written in the times of cpp17)
      if constexpr (std::is_same_v<typename std::tuple_element_t<TUPLE_IDX, marginals_data_t>::mapped_type::KeyMeta_t,
                                   Q_KEYMETA_T>)
      { return TUPLE_IDX; }
      else
      {
        return get_correct_tuple_idx<Q_KEYMETA_T,TUPLE_IDX + 1>();
      }
    }
    /**
     * @brief get an idx in a tuple statically using the marginal type  (recursive until the meta matches)
     *
     * @tparam Q_MARGINAL_T
     * @tparam I
     *
     * @return
     */
    template <typename Q_WMARG_T, std::size_t TUPLE_IDX = 0>
    static constexpr std::size_t get_correct_tuple_idx_by_marg()
    {
      static_assert(TUPLE_IDX < kNbMarginals);
      if constexpr (std::is_same_v<typename std::tuple_element_t<TUPLE_IDX, marginals_data_t>::
                                       mapped_type,
                                   Q_WMARG_T>) 
      { return TUPLE_IDX; }
      else
      {
        return get_correct_tuple_idx_by_marg<Q_WMARG_T,TUPLE_IDX + 1>();
      }
    }

    // static assert that all KEYMETA are unique  TODO:
    // static_assert( !std::is_same_v<> )
  };


  // print all marginals in the marginal container
  template <typename TUPLE_MAP_WMARGINAL_T>
  std::string stringify_marginal_container_block(const TUPLE_MAP_WMARGINAL_T & marginals_data, int tabulation=4, int precision=4)
  {
    std::stringstream ss;
    std::apply(
                [&](const auto & ...map_of_wmarginals)
                {
                    // declaring the function
                    auto loop_map = [&](const auto & my_map)
                    {
                      for(const auto & [key_id, wmarginal] : my_map)
                      {
                        ss << std::setw(tabulation)
                           << "[ " << key_id << " ] : \t"
                           << stringify_marginal_oneliner(wmarginal.marginal, precision) << '\n';
                      }
                    };
                    (loop_map(map_of_wmarginals),...);
                }
                , marginals_data
              );
    return ss.str();
  }

  // print marginal, no cov
  template <typename MARGINAL_T>
  std::string stringify_marginal_oneliner(const MARGINAL_T & Xmarg, int precision=4)
  {
    using keymeta_t = typename MARGINAL_T::KeyMeta_t;
    std::stringstream ss;
    ss << keymeta_t::stringify_key_oneliner(Xmarg.mean,  precision);
    return ss.str(); 
  }

  // print marginal, with cov
  template <typename MARGINAL_T>
  std::string stringify_marginal_blockliner(const MARGINAL_T & Xmarg, int precision=4)
  {
    using keymeta_t = typename MARGINAL_T::KeyMeta_t;
    std::stringstream ss;
    ss << keymeta_t::stringify_key_oneliner(Xmarg.mean,  precision);
    if (Xmarg.covariance.has_value())
    {
      // TODO: embellish a little bit...
      ss << Xmarg.covariance.value() << "\n";
    }
    return ss.str(); 
  }


  // visual covariance 
  template <typename COV_T>
  std::tuple<std::pair<double, double>, double> get_visual_2d_covariance(const COV_T & covariance_matrix)
  {
    Eigen::SelfAdjointEigenSolver<COV_T>    es(covariance_matrix);   
    std::pair<double, double> sigma;
    sigma.first = sqrt(es.eigenvalues()[0]);
    sigma.second = sqrt(es.eigenvalues()[1]);
    // std::array<double, 2> sigma {sqrt(es.eigenvalues()[0]), sqrt(es.eigenvalues()[1])};
    auto                  R = es.eigenvectors();
    double rot = std::atan2(R(1, 0), R(0, 0));
    return {sigma, rot};
  }

  //------------------------------------------------------------------//
  //                     METAPROGRAMING UTILITIES                     //
  //------------------------------------------------------------------//
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
  // // specialization: if tuple of marginals is given, then extract whats inside the tuple and
  // // fallback to the struct above
  // template <typename KEYMETA_T, typename... KEYMETA_Ts>
  // class MarginalsHistoriesContainer<std::tuple<KEYMETA_T, KEYMETA_Ts...>>
  //     : public MarginalsHistoriesContainer<KEYMETA_T, KEYMETA_Ts...>   // WOW !!
  // {
  // };
  // template <typename KEYMETA_T>
  // class MarginalsHistoriesContainer<std::tuple<KEYMETA_T>> : public MarginalsHistoriesContainer<KEYMETA_T>   // WOW !!
  // {
  // };

}   // namespace
