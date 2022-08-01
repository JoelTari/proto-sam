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

namespace sam::Marginal
{
  template <typename KEYMETA>
  class BaseMarginal : KEYMETA    // Rename as guassin
  {
    public:
    using KeyMeta_t = KEYMETA;
    // URGENT: decouple type mean of dX (a vector) and the type of the mean of the distribution (not necessarily a vector, e.g. an element of SE(n) ...)
    using Mean_t = Eigen::Vector<double, KEYMETA::kN>;
    using Mean_Distribution_t = Mean_t; // Distribution mean : by default same as Mean_t, but it's the implementer job's to override it
    using Mean_Distribution_t_ptr = std::shared_ptr<Mean_Distribution_t>; // Distribution mean : by default same as Mean_t, but it's the implementer job's to override it
    using Mean_t_ptr = std::shared_ptr<Mean_t>;
    using Covariance_t = Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN>;
    using Sigmas_t = std::array<double, 2>;
    using VisualCovariance_t = std::tuple<Sigmas_t, double>;
    // bool marked = false;
    Mean_t_ptr              mean_ptr; // TODO: ACTION: mean distribution here ??
    Covariance_t covariance;
    

    // ctor
    BaseMarginal(Mean_t_ptr mean_ptr,const Covariance_t& covariance = Covariance_t::Zero()) // TODO: ACTION: use distribution mean
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

    // FIX: add method, or the DerivedMarginal doesnot work, so maybe overide the '+' operator
    // FIX: overloading the + operator must be done at implementation level
    // FIX: OR, one must declare a derivation such as class SE2Marginal : Marginal { /* overload '+' here */ }
    // FIX: We will also have to assume that a SEn marginal mean's is represented as vector in its Lie Algebra
    // void add (const Mean_t & increment)
    // {
    //   static_cast<DerivedMarginal*>(this)->add(increment);
    // }
  };

  /**
   * @brief Structure to hold Historical moments (means & covariance) of a marginal
   *
   * @tparam MARGINAL_T marginal type
   * @param var_id identifier key of the marginal
   * @param mean historical means value of the marginal
   * @param visual_covariance (2d) historical value visual covariances matrix of the marginal
   */
  template <typename MARGINAL_T>
  struct MarginalHistory
  {
      using Marginal_t = MARGINAL_T;
      std::string var_id;
      std::vector<typename Marginal_t::Mean_t> iterative_means;  // TODO: ACTION maybe it's mean distribution here ??
      std::vector<typename Marginal_t::VisualCovariance_t> iterative_covariances;

      MarginalHistory(  const std::string& var_id
                      , const typename Marginal_t::Mean_t & mean
                      , const typename Marginal_t::VisualCovariance_t & visual_covariance)
      :
        var_id(var_id),iterative_means({mean}),iterative_covariances({visual_covariance})
      {}

  };

  /**
   * @brief Container of a set of marginals, of their histories
   *
   * @tparam KEYMETA_T key meta of first marginal type (there must be at least 1 type)
   * @tparam KEYMETA_Ts expansion of key metas of the rest of the marginal types
   * @param key_id identifier key of the marginal
   * @param marginal_ptr pointer to the marginal
   */
  template <typename KEYMETA_T, typename... KEYMETA_Ts>
  struct MarginalsHistoriesContainer
  {
    // type define a tuple of map of MarginalHistory types of each type of marginal
    using marginals_histories_t
        = std::tuple<std::unordered_map<std::string, MarginalHistory<BaseMarginal<KEYMETA_T>>>,
                     std::unordered_map<std::string, MarginalHistory<BaseMarginal<KEYMETA_Ts>>>...>; 
    
    // maps (one for each marginal type) of marginal identifier (string) to its history (MarginalHistory<Marginal_T>)
    marginals_histories_t marginal_history_tuple;

    // number of types of marginals
    static constexpr const std::size_t kNbMarginals { std::tuple_size_v<marginals_histories_t>};

    /**
     * @brief Insert a new marginal into the container
     *
     * @tparam Q_MARG_T marginal type
     * @param key_id identifier key of the marginal
     * @param marginal_ptr pointer to the marginal value
     */
    template <typename Q_MARG_T>
    void insert_new_marginal(const std::string & key_id, const std::shared_ptr<Q_MARG_T> marginal_ptr)
    {
      using KM_T = typename Q_MARG_T::KeyMeta_t;
      // assert the inserted marginal type corresponds to existing marginal type in the container
      static_assert(std::is_same_v<KM_T,KEYMETA_T> || (std::is_same_v<KM_T,KEYMETA_Ts> || ...)  );

      // template programming magic to get the tuple index
      constexpr std::size_t I = get_correct_tuple_idx_by_marg<KM_T>();

      // Create object marginal history object
      // The first item is of the history is its current value (mean and cov)
      auto marginal_history = MarginalHistory<Q_MARG_T>(
                                                         key_id
                                                       , *marginal_ptr->mean_ptr // TODO: action : maybe it's mean_distribution type here
                                                       , marginal_ptr->get_visual_2d_covariance()
                                                       );

      // select the correct map structure, and insert/assign the marginal history
      std::get<I>(this->marginal_history_tuple).insert_or_assign(key_id, marginal_history ); // TODO: manage failure
    }
    
    /**
     * @brief Push the moments (mean & cov) of the a marginal whose identifier key already exists 
     *
     * @tparam Q_MARG_T marginal type
     * @param key_id identifier key of the marginal
     * @param marginal_ptr point to the marginal value
     */
    template <typename Q_MARG_T>
    void push_marginal_history(const std::string & key_id, const std::shared_ptr<Q_MARG_T> marginal_ptr)
    {
      using KM_T = typename Q_MARG_T::KeyMeta_t;
      // check that we are inserting an existing type
      static_assert(std::is_same_v<KM_T,KEYMETA_T> || (std::is_same_v<KM_T,KEYMETA_Ts> || ...)  );

      // template programming magic to get the tuple index
      constexpr std::size_t I = get_correct_tuple_idx_by_marg<KM_T>();

      // get marginal history ref
      auto marginal_history_it = std::get<I>(this->marginal_history_tuple).find(key_id);
      // TODO: assert (run time) that the marginal key exists in the map
      // TODO: assert(marginal_history_it != std::get<I>(this->marginal_history_tuple).end() );
      // push new data
      marginal_history_it->second.iterative_means.push_back( *(marginal_ptr->mean_ptr) ); // TODO: ACTION: might be mean_distribution here
      marginal_history_it->second.iterative_covariances.push_back( marginal_ptr->get_visual_2d_covariance() );
    }


    /**
     * @brief static function to find the correct index of a marginal meta type in a tuple of various marginals
     *
     * @tparam KM_T keymeta type
     * @return index in tuple
     */
    template <typename KM_T, std::size_t I = 0>
    static constexpr std::size_t get_correct_tuple_idx_by_marg()
    {
      static_assert(I < kNbMarginals);
      if constexpr (std::is_same_v<typename std::tuple_element_t<I, marginals_histories_t>::
                                       mapped_type::Marginal_t::KeyMeta_t, KM_T>)
      { return I; }
      else
      {
        return get_correct_tuple_idx_by_marg<KM_T,I + 1>();
      }
    }

  };

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

    using marginals_containers_t
        = std::tuple<std::unordered_map<std::string, std::shared_ptr<BaseMarginal<KEYMETA_T>>>,
                     std::unordered_map<std::string, std::shared_ptr<BaseMarginal<KEYMETA_Ts>>>...>;

    static constexpr const std::size_t kNbMarginals { std::tuple_size_v<marginals_containers_t>};

    /**
     * @brief find a marginal ptr in this container.
     *        WARNING: not used yet, be sure to test
     *
     * @tparam Q_KEYMETA_T type of the queried marginal
     * @param key_id key identifier of the marginal (e.g. "x0")
     * @return optional marginal shared pointer
     */
    template <typename Q_KEYMETA_T>
    std::optional<std::shared_ptr<BaseMarginal<Q_KEYMETA_T>>>
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

    /**
     * @brief find, by id, a marginal 's mean ptr in this container
     *
     * @tparam Q_KEYMETA_T type of the keymeta of the marginal
     * @param key_id key identifier of the amrginal (e.g. "x0")
     * @return optional mean shared pointer
     */
    template <typename Q_KEYMETA_T>
    std::optional<std::shared_ptr<typename BaseMarginal<Q_KEYMETA_T>::Mean_t>>  // TODO: ACTION: use the distribution mean type ??
        find_mean_ptr(const std::string& key_id) 
    {
      // static assert
      static_assert(std::is_same_v<Q_KEYMETA_T,
                                   KEYMETA_T> || (std::is_same_v<Q_KEYMETA_T, KEYMETA_Ts> || ...));

      // get the correct tuple element
      constexpr std::size_t I = get_correct_tuple_idx<Q_KEYMETA_T>();

      if (auto it {std::get<I>(this->data_map_tuple).find(key_id)};
          it != std::end(std::get<I>(this->data_map_tuple)))
      { return it->second->mean_ptr; } // TODO: ACTION: use the distribution mean
      else
        return std::nullopt;
    }

    /**
     * @brief insert marginal pointer in this container. The key id might
     * already exists, in this situation, the new marginal ptr value is
     * assigned
     *
     * @tparam Q_MARG_T type of the marginal
     * @param key_id key identifier of the marginal (e.g. "x0")
     * @param marg_ptr marginal pointer
     */
    template <typename Q_MARG_T>
    void insert_in_marginal_container(const std::string & key_id, std::shared_ptr<Q_MARG_T> marg_ptr)
    {
      constexpr std::size_t I = get_correct_tuple_idx_by_marg<Q_MARG_T>();
      std::get<I>(this->data_map_tuple).insert_or_assign(key_id, marg_ptr);
      // TODO: run time assertion: verify that the key_id does not exist in other marginal type (e.g. having "x0" as a pose and "x0" as something else in another part of the tuple)
    }

    // main structure
    marginals_containers_t data_map_tuple;

    protected:
    /**
     * @brief get an idx in a tuple statically using the key meta type  (recursive until the meta matches)
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
    /**
     * @brief get an idx in a tuple statically using the marginal type  (recursive until the meta matches)
     *
     * @tparam Q_MARGINAL_T
     * @tparam I
     *
     * @return
     */
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
