#pragma

#include "meta/meta_interface.h"
#include "system/config.h"
#include "utils/tuple_patterns.h"

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>


namespace sam::Marginal
{
  template <typename KEYMETA>
  class BaseMarginal : KEYMETA   // Rename as gaussian ?
  {
    public:
    using KeyMeta_t       = KEYMETA;
    using Mean_t          = typename KeyMeta_t::key_t;
    using Tangent_Space_t = typename KeyMeta_t::tangent_space_t;
    using Covariance_t    = Eigen::Matrix<double, KEYMETA::kN, KEYMETA::kN>;
    using OptCovariance_t = std::optional<Covariance_t>;
    Mean_t          mean;
    OptCovariance_t covariance;


    // ctor
    BaseMarginal(Mean_t mean, const OptCovariance_t& covariance = std::nullopt)
        : mean(mean)
        , covariance(covariance) {};
  };


  template <typename MARGINAL_T>
  struct WrapperPersistentMarginal
  {
    using Marginal_t = MARGINAL_T;
    using Mean_t     = typename Marginal_t::Mean_t;
    // using Mean_t_ptr = typename Marginal_t::Mean_t_ptr;
    using OptCovariance_t = typename Marginal_t::OptCovariance_t;

    WrapperPersistentMarginal(const std::string&             key_id,
                              const std::shared_ptr<Mean_t>& mean_ptr,
                              const OptCovariance_t&         opt_covariance = std::nullopt)
        : shared_mean(mean_ptr)
        , key_id(key_id)
        , marginal(*mean_ptr, opt_covariance)
        , marginal_histories({})
    {
    }

    const std::string key_id;

    std::vector<MARGINAL_T> marginal_histories;

    void save_and_replace(const MARGINAL_T& new_marginal)
    {
      this->marginal_histories.push_back(this->marginal);
      this->marginal       = new_marginal;
      *(this->shared_mean) = new_marginal.mean;
    }

    void save_and_add(const typename MARGINAL_T::Tangent_Space_t& tangent_increment,
                      const OptCovariance_t&                      opt_covariance = std::nullopt)
    {
      this->marginal_histories.push_back(this->marginal);
      this->marginal.mean += tangent_increment;
      this->marginal.covariance = opt_covariance;
      *(this->shared_mean)      = this->marginal.mean;
    }

    void clear_history() { this->marginal_histories.clear(); }

    // copy assignment (explicit otherwise default copy assignment fails because of const members)
    WrapperPersistentMarginal<MARGINAL_T>&
        operator=(const WrapperPersistentMarginal<MARGINAL_T>& other)
    {
      if (this == &other) return *this;

      // we dont copy-assign the key or the shared pointer obviously
      // But lets assert they are the same
      assert(this->key_id == other.key_id);
      assert(this->shared_mean == other.shared_mean);   // same address
      this->marginal_histories = other.marginal_histories;
      this->marginal           = other.marginal;   // shallow copy of marginal is ok

      return *this;
    }

    // copy ctor (ro3)
    WrapperPersistentMarginal<MARGINAL_T>(const WrapperPersistentMarginal<MARGINAL_T>& other)
        : shared_mean(other.shared_mean)
        , key_id(other.key_id)
        , marginal(other.marginal)
        , marginal_histories(other.marginal_histories)
    {
    }

    // destructor (ro3)
    ~WrapperPersistentMarginal<MARGINAL_T>() {}

    const std::shared_ptr<Mean_t> shared_mean;   // seen by other system class (wfactors...)
    MARGINAL_T                    marginal;
  };


  //------------------------------------------------------------------//
  //                       MARGINALS CONTAINER                        //
  //------------------------------------------------------------------//
  /**
   * @brief System level structure to hold the marginals
   *
   * @tparam KEYMETA_T first type of marginal (there has to be at least one type of marginal in the
   * system)
   * @tparam KEYMETA_Ts expansion on the other types of marginals
   * @param key_id marginal key identifier
   */
  template <typename KEYMETA_T, typename... KEYMETA_Ts>
  class MarginalsCollection
  {
    public:
    using type = MarginalsCollection<KEYMETA_T, KEYMETA_Ts...>;

    using Marginals_Data_t = std::tuple<
        std::unordered_map<std::string, WrapperPersistentMarginal<BaseMarginal<KEYMETA_T>>>,
        std::unordered_map<std::string, WrapperPersistentMarginal<BaseMarginal<KEYMETA_Ts>>>...>;

    static constexpr std::size_t kNbMarginals {std::tuple_size_v<Marginals_Data_t>};

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
      {
        return it->second;
      }
      else
        return std::nullopt;
    }

    // find a mean ptr, the same as above, but more practical to build composite state of optionals
    // (when guessing incomplete init point)
    template <typename Q_KEYMETA_T>
    std::optional<std::shared_ptr<typename Q_KEYMETA_T::key_t>>
        find_mean_ptr(const std::string& key_id) const
    {
      // static assert
      static_assert(std::is_same_v<Q_KEYMETA_T,
                                   KEYMETA_T> || (std::is_same_v<Q_KEYMETA_T, KEYMETA_Ts> || ...));

      // get the correct tuple element
      constexpr std::size_t TUPLE_IDX = get_correct_tuple_idx<Q_KEYMETA_T>();

      if (auto it {std::get<TUPLE_IDX>(this->data_map_tuple).find(key_id)};
          it != std::end(std::get<TUPLE_IDX>(this->data_map_tuple)))
      {
        return it->second.shared_mean;
      }
      else
        return std::nullopt;
    }

    template <typename Q_WMARG_T>
    void insert_in_marginal_container(const Q_WMARG_T& wmarg)
    {
      constexpr std::size_t TUPLE_IDX = get_correct_tuple_idx_by_wmarg<Q_WMARG_T>();
      std::get<TUPLE_IDX>(this->data_map_tuple).insert_or_assign(wmarg.key_id, wmarg);
      // TODO: run time assertion: verify that the key_id does not exist in other marginal type
      // (e.g. having "x0" as a pose and "x0" as something else in another part of the tuple)
    }


    void clear_histories()
    {
      // warning: it is quite slow to iterate a large map
      std::apply([](const auto&... map_of_wmarg) { ((clear_map_of_histories(map_of_wmarg)), ...); },
                 this->data_map_tuple);
    }

    template <typename MAP_OF_WMARG>
    static void clear_map_of_histories(const MAP_OF_WMARG& map_of_wmarg)
    {
      for (auto& [id, wmarg] : map_of_wmarg) wmarg.clear_history();
    }

    // main structure
    Marginals_Data_t data_map_tuple;

    protected:
    /**
     * @brief get an idx in a tuple statically using the key meta type  (recursive until the meta
     * matches)
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
      if constexpr (std::is_same_v<typename std::tuple_element_t<TUPLE_IDX, Marginals_Data_t>::
                                       mapped_type::Marginal_t::KeyMeta_t,
                                   Q_KEYMETA_T>)
      {
        return TUPLE_IDX;
      }
      else { return get_correct_tuple_idx<Q_KEYMETA_T, TUPLE_IDX + 1>(); }
    }
    /**
     * @brief get an idx in a tuple statically using the marginal type  (recursive until the meta
     * matches)
     *
     * @tparam Q_MARGINAL_T
     * @tparam I
     *
     * @return
     */
    template <typename Q_WMARG_T, std::size_t TUPLE_IDX = 0>
    static constexpr std::size_t get_correct_tuple_idx_by_wmarg()
    {
      static_assert(TUPLE_IDX < kNbMarginals);
      if constexpr (std::is_same_v<
                        typename std::tuple_element_t<TUPLE_IDX, Marginals_Data_t>::mapped_type,
                        Q_WMARG_T>)
      {
        return TUPLE_IDX;
      }
      else { return get_correct_tuple_idx_by_wmarg<Q_WMARG_T, TUPLE_IDX + 1>(); }
    }

    // static assert that all KEYMETA are unique  TODO:
  };


  // print all marginals in the marginal container
  template <typename TUPLE_MAP_WMARGINAL_T>
  std::string stringify_marginal_container_block(const TUPLE_MAP_WMARGINAL_T& marginals_data,
                                                 int                          tabulation = 4,
                                                 int                          precision  = 4)
  {
    std::stringstream ss;
    std::apply(
        [&](const auto&... map_of_wmarginals)
        {
          // declaring the function
          auto loop_map = [&](const auto& my_map)
          {
            for (const auto& [key_id, wmarginal] : my_map)
            {
              ss << std::setw(tabulation) << "[ " << key_id << " ] : \t"
                 << stringify_marginal_oneliner(wmarginal.marginal, precision) << '\n';
            }
          };
          (loop_map(map_of_wmarginals), ...);
        },
        marginals_data);
    return ss.str();
  }

  // print marginal, no cov
  template <typename MARGINAL_T>
  std::string stringify_marginal_oneliner(const MARGINAL_T& Xmarg, int precision = 4)
  {
    using keymeta_t = typename MARGINAL_T::KeyMeta_t;
    std::stringstream ss;
    ss << keymeta_t::stringify_key_oneliner(Xmarg.mean, precision);
    return ss.str();
  }

  // print marginal, with cov
  template <typename MARGINAL_T>
  std::string stringify_marginal_blockliner(const MARGINAL_T& Xmarg, int precision = 4)
  {
    using keymeta_t = typename MARGINAL_T::KeyMeta_t;
    std::stringstream ss;
    ss << keymeta_t::stringify_key_oneliner(Xmarg.mean, precision);
    if (Xmarg.covariance.has_value())
    {
      // TODO: embellish a little bit...
      ss << Xmarg.covariance.value() << "\n";
    }
    return ss.str();
  }


  // visual covariance
  // WARNING: it is assumed, for now, that the first 2x2 block of the cov matrix hold x,y marginal
  // covariance
  template <typename COV_T>
  std::tuple<std::pair<double, double>, double>
      get_visual_2d_covariance(const COV_T& covariance_matrix)
  {
    Eigen::SelfAdjointEigenSolver<COV_T> es(covariance_matrix);
    std::pair<double, double>            sigma;
    sigma.first  = sqrt(es.eigenvalues()[0]);
    sigma.second = sqrt(es.eigenvalues()[1]);
    auto   R     = es.eigenvectors();
    double rot   = std::atan2(R(1, 0), R(0, 0));
    return {sigma, rot};
  }

  //------------------------------------------------------------------//
  //                     METAPROGRAMING UTILITIES                     //
  //------------------------------------------------------------------//
  // specialization: if tuple of marginals is given, then extract whats inside the tuple and
  // fallback to the struct above
  template <typename KEYMETA_T, typename... KEYMETA_Ts>
  class MarginalsCollection<std::tuple<KEYMETA_T, KEYMETA_Ts...>>
      : public MarginalsCollection<KEYMETA_T, KEYMETA_Ts...>   // WOW !!
  {
  };
  template <typename KEYMETA_T>
  class MarginalsCollection<std::tuple<KEYMETA_T>> : public MarginalsCollection<KEYMETA_T>   // WOW !!
  {
  };
}   // namespace sam::Marginal
