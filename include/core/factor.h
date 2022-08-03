#ifndef FACTOR_H_
#define FACTOR_H_

#include "utils/tuple_patterns.h"
#include "utils/utils.h"

#include <array>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <optional>
#include <string_view>
#include <tuple>
#include <utility>

namespace details_sam::Conduct{

  /**
   * @brief KeyContextualConduct  -- The class describes how a key (associated with a key meta)
   * behaves in a certain context (in a factor). It stores dimension informations name and most
   * importantly Jacobian matrices to be used when differentiating the factor wrt the decision keys.
   * As a factor can have several keys, therefore, the KeyContextualConduct class only manages a
   * subset of the columns of the factor 's process matrix (normed Jacobian).
   *
   * @tparam DerivedKCC Derived user class. Static polymorhism pattern.
   * @tparam KEY_META Meta of the key (dimensions, type, components name of the key)
   * @tparam MEASURE_META Meta of the measure z (dimensions, type, components name)
   * @tparam ContextRole -- (text) role of the key in the context
   */
  template <typename DerivedKCC,
            typename KEY_META,
            typename MEASURE_META,
            const char* ContextRole>
  struct KeyContextualConduct : KEY_META
  {
    using KeyMeta_t = KEY_META;
    using MeasureMeta_t = MEASURE_META;
    using Key_t     = typename KeyMeta_t::key_t;
    using Measure_t = typename MeasureMeta_t::measure_t;
    static constexpr const char*       kRole {ContextRole};
    static constexpr const std::size_t kM {MeasureMeta_t::kM};
    static constexpr const std::size_t kN {KeyMeta_t::kN};
    static constexpr const bool        kLinear {false};
    // non static but const
    const std::string key_id;
    // non static, not const
    using key_process_matrix_t = Eigen::Matrix<double, kM, kN>;
    using measure_cov_t        = Eigen::Matrix<double, kM, kM>;
    using tangent_space_vect_t = Eigen::Matrix<double, kN, 1>;
    // measure_vect_t   b;
    const measure_cov_t& rho;

    // euclidian space is a trivial manifold (same as its tangent space)
    static constexpr bool kIsTrivialManifold = std::is_same_v<Key_t, tangent_space_vect_t>;

      // WARNING: SRP: remove
    const std::shared_ptr<Key_t> key_mean_view = nullptr;

    /**
     * @brief compute Aik, the normed jacobian matrix, a.k.a. the partial derivative of the factor
     * criterion premultiply by the root of the measure precision Aik = \rho * \partial r_i(X) /
     * \partial X_k Index i refers to the usual iterator on factor, k to the kth key of factor \phi_i
     *
     * @param Xk linearization point
     * @return the process matrix associated with this key. Note that it only a subset of the columns
     * of the full process matrix of factor \phi_i as there may be other keys k in the factor with
     * their own sub process-matrix Aik.
     */
    key_process_matrix_t compute_Aik_at(const Key_t & Xk) const
    {
      return static_cast<const DerivedKCC*>(this)->compute_Aik_at_impl(Xk);
    }

    /**
     * @brief compute Aik, the normed jacobian matrix, a.k.a. the partial derivative of the factor
     * criterion premultiply by the root of the measure precision Aik = \rho * \partial r_i(X) /
     * \partial X_k Index i refers to the usual iterator on factor, k to the kth key of factor \phi_i
     * @return the process matrix associated with this key. Note that it only a subset of the columns
     * of the full process matrix of factor \phi_i as there may be other keys k in the factor with
     * their own sub process-matrix Aik.
     */
    key_process_matrix_t compute_Aik_at_current_lin_point() const
    {
      // WARNING: SRP: remove
      // copy the current linearization point
      if (key_mean_view == nullptr) throw std::runtime_error("nullptr to linearization point Xk");
      const auto Xk = *key_mean_view;
      return this->compute_Aik_at(Xk);
    }

    /**
     * @brief Constructor , no pointer to the value of the key is required. This one is therefore by
     * fully linear SLAM problems.
     *
     * @param key_id identifier of the key
     * @param rho root of the inverse of the measure covariance matrix:   \rho = chol( \Sigma_i )^T
     */
    KeyContextualConduct(const std::string& key_id, const measure_cov_t& rho)
        : key_id(key_id)
        , rho(rho)
    {
      // necessary (but not sufficient) condition for this ctor: the context model must be linear.
      // sufficient condition would be that the wider system be linear (enforceable at higher level)
    }

    /**
     * @brief Constructor. In nonlinear systems, it required to provide a ptr to a value of the key:
     * typically this value is the linearization point. Note that a Key can be linear in the context
     * of one factor, but nonlinear with respect to another. This is why a slam system that has at
     * least one type of nonlinear factor alters the potential linearity behavior of other factors in
     * the same system. To summarize, even if this key behaves linearly in this context, the nonlinear
     * aspects may still be enforced by the user
     *
     * @param key_id identifier of the key
     * @param rho root of the inverse of the measure covariance matrix: \rho = chol( \Sigma_i ) ^T
     * @param init_point_view shared pointer to a value of the key
     */
    KeyContextualConduct(const std::string&     key_id,
                         const measure_cov_t&   rho,
                         std::shared_ptr<Key_t> init_point_view)
        : key_id(key_id)
        , rho(rho)
        , key_mean_view(init_point_view)
      // WARNING: SRP: remove
    {
    }
  };

  //------------------------------------------------------------------//
  //          Linear augmentation of Key Contextual Conduct           //
  //------------------------------------------------------------------//
  /**
   * @brief KeyContextualConduct  -- The class describes how a key (associated with a key meta)
   * behaves in a certain context (in a linear factor). It stores dimension informations name and most
   * importantly process matrices to be used when assessing the measure expectation wrt the decision keys.
   * As a factor can have several keys, therefore, the LinearKeyContextualConduct class only manages a
   * subset of the columns of the factor 's process matrix.
   *
   * @tparam DerivedLinearKCC Derived user class. Static polymorhism pattern.
   * @tparam KEY_META Meta of the key (dimensions, type, components name of the key)
   * @tparam DimMes dimension of the measure
   * @tparam ContextRole -- (text) role of the key in the context
   */
  template <
            typename KEY_META,
            typename MEASURE_META,
            const char* ContextRole,
            const auto * Hik_MATRIX_PTR
            >
  struct LinearKeyContextualConduct 
  : KeyContextualConduct
      <
        LinearKeyContextualConduct<KEY_META,MEASURE_META,ContextRole,Hik_MATRIX_PTR>
        ,KEY_META
        ,MEASURE_META
        ,ContextRole
      >
  {
    using base_kcc_t = KeyContextualConduct
      <
        LinearKeyContextualConduct<KEY_META,MEASURE_META,ContextRole,Hik_MATRIX_PTR>
        ,KEY_META
        ,MEASURE_META
        ,ContextRole
      >;
    using Key_t = typename base_kcc_t::Key_t;

    using key_process_matrix_t = typename base_kcc_t::key_process_matrix_t;
    using measure_cov_t = typename base_kcc_t::measure_cov_t;

    static_assert(std::is_same_v< std::remove_pointer_t<decltype(Hik_MATRIX_PTR)>,const key_process_matrix_t > );

     // NOTE: couldn't make it (inline) static, for some reason it would always be full of zeros
     // NOTE: So it is only const, i.e. one object by instance but, for small matrix, the additional
     // NOTE: memory cost shouldn't be too bad, plus it is probably better for cache locality
    const key_process_matrix_t Hik  {*Hik_MATRIX_PTR}; // by copy at construction time

    // this is a linear context for this KCC template
    static constexpr const bool        kLinear {true};

    const key_process_matrix_t Aik = this->rho*this->Hik;

    LinearKeyContextualConduct(const std::string& key_id, const measure_cov_t& rho)
      : base_kcc_t(key_id,rho)
    {
    }

    LinearKeyContextualConduct
      (
        const std::string& key_id
        ,const measure_cov_t& rho
        ,std::shared_ptr<Key_t> init_point_view
      )
      : base_kcc_t(key_id,rho,init_point_view)
    {
    }
  };
}

namespace sam::Factor
{
  //------------------------------------------------------------------//
  //                    Base Factor class template                    //
  //------------------------------------------------------------------//
  /**
   * @brief Class template that holds:
   * - a gaussian pdf of a measurement Z given its set of -hidden- variables \mathcal X.
   * - the model that links the measurement it to its set of hidden variables \cal X := { X_k }, also
   * called keys.
   *
   * A factor is a representation of the pdf likelihood p(Z = z | \cal X) .
   * It is assumed that p( Z = z | \mathcal X ) \propto \exp^ { - r(X;Z)^T r(X;Z) }
   * This block is class template because:
   * - the meta type, the number of keys varies factor to factor
   * - the measurement meta type varies factor to factor
   * - the model r(X;Z) varies factor to factor, which in turns implies that the Jacobian derivatives
   *   of this function wrt X do vary too.
   *
   * The API exposes the computation of Ai and bi to be used by modern SLAM systems and the negative
   * log likelyhood norm at a given point.
   *
   *   A factor implementation must 'statically' inherits from this template or its refined
   * derivatives
   *
   * @tparam DerivedFactor derived class that implements all of or part of the API specific to a
   * factor type
   * @tparam FactorLabel labelization of the factor type
   * @tparam MEASURE_META meta data of the measurement (dimensions, components name ...)
   * @tparam KCCs Contextual Conduct of a Key inside this factor type. See KeyContextualConduct
   * class template.
   * @param factor_id identifier (string) of the factor (e.g. 'f0')
   * @param z measurement Z
   * @param z_cov measurement covariance
   * @param keys_id identifiers of the keys (e.g. "x0" , "x1")
   * @param tup_init_points_ptr init point of each key (for NL solvers)
   */
  template <typename DerivedFactor,
            const char* FactorLabel,
            typename MEASURE_META,    // FIX: URGENT: no need MEASURE_META: can be access via Kcc (but s_assert its the same for all kcc)
            typename... KCCs>  // FIX: CHORE: perhaps make it clear that you need always at least 1 KCC
  class BaseFactor
  {
    public:
    friend DerivedFactor;
    using MeasureMeta_t = MEASURE_META;
    using measure_t      = typename MeasureMeta_t::measure_t;
    static constexpr const char* kFactorLabel {FactorLabel};
    static constexpr size_t      kN      = (KCCs::kN + ...);
    static constexpr size_t      kM      = MeasureMeta_t::kM;
    static constexpr size_t      kNbKeys = sizeof...(KCCs);
    using criterion_t                    = Eigen::Vector<double, kM>;
    using measure_cov_t                  = Eigen::Matrix<double, kM, kM>;
    using factor_process_matrix_t        = Eigen::Matrix<double, kM, kN>;
    using state_vector_t                 = Eigen::Matrix<double, kN, 1>;   // { dXk , ... }
    using composite_state_ptr_t
        = std::tuple<std::shared_ptr<typename KCCs::Key_t>...>;   // {*Xk ...}
    using composite_of_opt_state_ptr_t
        = std::tuple<std::optional<std::shared_ptr<typename KCCs::Key_t>>...>;
    //  NOTE: Xk same type as dXk in euclidian factors
    using matrix_Ai_t    = Eigen::Matrix<double, kM, kN>;
    using matrices_Aik_t = std::tuple<typename KCCs::key_process_matrix_t...>;
    using KeysSet_t      = std::tuple<KCCs...>;

    static constexpr const char*                 kMeasureName {MeasureMeta_t::kMeasureName};
    static constexpr auto kMeasureComponentsName = MeasureMeta_t::components;
    const std::string                            factor_id;   // fill at ctor
    const measure_t                              z;           // fill at ctor
    const measure_cov_t                          z_cov;       // fill at ctor
    const measure_cov_t                          rho;         // fill at ctor
    const std::array<std::string, kNbKeys>       keys_id;     // fill at ctor
    KeysSet_t                                    keys_set;    // fill at ctor, mutable

    static constexpr bool isLinear = (KCCs::kLinear && ...);

    static_assert( (std::is_same_v<MeasureMeta_t,typename KCCs::MeasureMeta_t> && ...) );

    // FIX: URGENT: have a default constructor ...?

    /**
     * @brief Constructor
     *
     * @param factor_id
     * @param factor_id identifier (string) of the factor (e.g. 'f0')
     * @param z measurement Z
     * @param z_cov measurement covariance
     * @param keys_id array identifiers of the keys (e.g. "x0" , "x1"). Order must respect the set
     * KCCs order
     * @param tup_init_points_ptr init point of each key (for NL solvers)
     */
    BaseFactor(const std::string&                      factor_id,
               const measure_t&                        z,
               const measure_cov_t&                    z_cov,
               const std::array<std::string, kNbKeys>& keys_id,
               const composite_state_ptr_t&            tup_init_points_ptr)
        : z(z)
        , z_cov(z_cov)
        , factor_id(factor_id)
        , rho(Eigen::LLT<measure_cov_t>(z_cov.inverse()).matrixU())
        , keys_id(keys_id)
        , keys_set(sam_tuples::reduce_array_variadically(
              keys_id, 
              []<std::size_t... I>(const auto& my_keys_id,
                                   const auto& rho,
                                   const auto& tup_init_points_ptr,
                                   std::index_sequence<I...>)
                  ->KeysSet_t {
                    // return std::make_tuple(KCCs(my_keys_id[I], rho)...); // original
                    // might be possible to use perfect forwarding, by declaring an empty tuple
                    // and next line expanding tuple_cat with an intermediary function that has
                    // perfect forwarding ( TODO:)
                    //  TODO: CHORE: use std::apply here
                    //  FIX: URGENT: main point of difficulty when breaking downs the <,KCCs...> into <,KCC,...KCCs>
                    return {KCCs(my_keys_id[I], rho, std::get<I>(tup_init_points_ptr))...};
                  },
              rho,
              tup_init_points_ptr))
        , keyIdToTupleIdx(map_keyid(keys_id))
    {
      // TODO: throw if cov is not a POS matrix (consistency check enabled ?)
    }

    std::map<std::string, size_t> keyIdToTupleIdx;   // fill at ctor

    /**
     * @brief static method initial point guesstimator. At factor creation, an initial value for some
     * key points is missing (e.g. observation of a new landmark) For every combination of
     * missing/available key points, this method **tries to** guess an initial point for the missing
     * ones by using the factor model. If a combination can't fill a missing init point(e.g. first
     * bearing-only observation of a new 2D landmark ), a null option value  is returned.
     *
     * Since init points for all keys are necessary for the factor construction, failure to guess one
     * or more missing initial key point prevents the factor instantiation. It is the responsibility
     * of system designer to come up with adequate recovery approaches at higher level (e.g. waiting
     * for several bearing-only observations to define triangulation scheme).
     *
     * If all initial values are available anyway, this method does nothing
     *
     * @param x_init_ptr_optional_tup tuple containing, for each key in the factor type, the optional
     * pointers to values of initial key points value. If no initial value is available in inputs,
     * std::nullopt must be passed.
     * @param z measurement M
     * @return optional of either:
     *            - nullopt if the guesstimator has failed for one or more points.
     *            - a tuple of completed pointers to the values
     */
    static std::optional<composite_state_ptr_t>
        guess_init_key_points(composite_of_opt_state_ptr_t x_init_ptr_optional_tup,
                              const criterion_t&           z)
    {
      return DerivedFactor::guess_init_key_points_impl(x_init_ptr_optional_tup, z);
    }

    static composite_state_ptr_t make_composite(typename KCCs::Key_t... keys)
    {
      return {std::make_shared<typename KCCs::Key_t>(keys)...};
    }

    /**
     * @brief method that returns an array made of the keys id of this factor instance
     * The order correspond to the one given at construction time.
     *
     * @return array made of the keys id (array of string) of this factor instance
     */
    std::array<std::string, kNbKeys> get_array_keys_id() const
    {
      return sam_tuples::reduce_array_variadically(
          this->keys_set,
          [this]<std::size_t... J>(const auto& keyset, std::index_sequence<J...>) {
            return std::array<std::string, kNbKeys> {std::get<J>(keyset).key_id...};
          });
    }


    /**
     * @brief method template that computes matrices Ai and bi of this factor instance
     *  bi:= -  r( \bar X )  where \bar X is the linearization point
     *  Ai:=  \frac{ \partial r(X) } {\partial X} at \bar X .
     *
     *  The Ai matrix is actually given in a set of Aik (one Aik per key)
     *
     *  The method varies on linear factors whether or not the SLAM system is linear
     *
     * @return nested tuple of {bi , tuple (Aiks...) }
     */
    // template <bool isSystFullyLinear>
    std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_at(const composite_state_ptr_t & X) const
    {
      return static_cast<const DerivedFactor*>(this)->compute_Ai_bi_at_impl(X);
    }

      // WARNING: SRP: remove
    std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_at_current_lin_point() const
    {
      auto X = get_key_points();
      return this->compute_Ai_bi_at(X);
    }

    /**
     * @brief compute the value r(X) at X
     *
     * @param X tuple of pointers to Xk values (one Xk per key)
     * @return r(X) value (eigen vector)
     */
    criterion_t compute_r_of_x_at(const composite_state_ptr_t& X) const
    {
      return static_cast<const DerivedFactor*>(this)->compute_r_of_x_at_impl(X);
    }

    /**
     * @brief compute r(\bar X) where \bar X is the curent linearization point
     *
     * @return r(\bar X) value (eigen vector)
     */
    criterion_t compute_r_of_x_at_current_lin_point() const
      // WARNING: SRP: remove
    {
      // build back the tup of stored lin point
      auto lin_point_tup = this->get_key_points();
      return compute_r_of_x_at(lin_point_tup);
    }

    /**
     * @brief returns square root of the negative log likelihood at X.
     *  - log ( p(Z|X) ) = r(X;Z)^T r(X;Z) = || r(X;Z) ||^2_2 = norm ^2
     *
     * @param X tuple of pointers to Xk values (one pointer to value Xk per key)
     * @return scalar L2 norm
     */
    double factor_norm_at(const composite_state_ptr_t& X) const
    {
      return compute_r_of_x_at(X).norm();
    }

    /**
     * @brief returns square root of the negative log likelihood at the current linearization point
     * \bar X.
     *  - log ( p(Z|X) ) = r(X;Z)^T r(X;Z) = || r(X;Z) ||^2_2 = norm ^2
     *
     * @return scalar L2 norm
     */
    double factor_norm_at_current_lin_point() const
    {
      // WARNING: SRP: remove
      return compute_r_of_x_at_current_lin_point().norm();
    }

    protected:
    /**
     * @brief  constructor helper that creates a map that associated a key id to its index in the
     * input array of keys
     *
     * E.g.  { "x0", "l2" , "x3" }  produces { {"x0",0}, {"l2", 1}, {"x3", 2} }
     *
     * @param keys_id array of key identifiers (array of string)
     * @return std::map association key id (string) to index (int) in the input array
     */
    std::map<std::string, std::size_t>
        map_keyid(const std::array<std::string, kNbKeys>& keys_id) const
    {
      std::map<std::string, std::size_t> result;
      for (int i = 0; i < keys_id.size(); i++) result[keys_id[i]] = i;
      return result;
    }

    /**
     * @brief method that gets the tuple of pointers to the active
     * linearization points
     *
     * @return tuple of key values \bar Xk (one for each key in the factor)
     */
    composite_state_ptr_t get_key_points() const
    {
      // WARNING: SRP: remove
      // REFACTOR: why not make it a const member, at worst it would mean that the 
      // class holds 2 times the same shared_ptr for each key (one in keys_set.key_mean_view
      // , the other in that new tuple member)
      // const composite_state_ptr_t lin_points_view; 
      composite_state_ptr_t current_lin_points_ptr;
      std::apply([this, &current_lin_points_ptr](const auto& ... kcc)
          {
            current_lin_points_ptr = { kcc.key_mean_view ...};
          }
          , this->keys_set
          );
      return current_lin_points_ptr;
    }
  };


  //------------------------------------------------------------------//
  //                 Euclidian Factor class template                  //
  //                     r(X) := rho ( h(X) - z )                     //
  //        z and h(X) are euclidian vectors, keys X might not        //
  //                                be                                //
  //------------------------------------------------------------------//
  /**
   * @brief Class template that holds:
   * - a gaussian pdf of a measurement vector Z given its set of -hidden- variables \mathcal X.
   * - the model that links the measurement it to its set of hidden variables \cal X := { X_k }, also
   * called keys.
   *
   * The Euclidian Factor template 'statically' derives from the Base Factor. It cover the cases where
   * r(X) is of the form: r(X) = \rho ( h(X) - z ) I.e. the measure z and h(X) are both euclidian
   * vectors The keys X might still not be expressed in euclidian space though (e.g. landmark
   * cartesian observation). Look at LinearEuclidianFactor for a class template that supports
   * euclidian keys.
   *
   * A factor is a representation of the pdf likelihood p(Z = z | \cal X) .
   * It is assumed that p( Z = z | \mathcal X ) \propto \exp^ { - r(X;Z)^T r(X;Z) }
   * This block is class template because:
   * - the meta type, the number of keys varies factor to factor
   * - the measurement meta type varies factor to factor
   * - the model r(X;Z) varies factor to factor, which in turns implies that the Jacobian derivatives
   *   of this function wrt X do vary too.
   *
   * The API exposes the computation of Ai and bi to be used by modern SLAM systems and the negative
   * log likelyhood norm at a given point.
   *
   *
   *   A factor implementation must 'statically' inherits from this template or its refined
   * derivatives
   *
   * @tparam DerivedEuclidianFactor derived class that implements all of or part of the API specific
   * to a factor type
   * @tparam FactorLabel labelization of the factor type
   * @tparam MEASURE_META meta data of the measurement (dimensions, components name ...)
   * @tparam KCCs Contextual Conduct of a Key inside this factor type. See KeyContextualConduct
   * class template.
   * @param factor_id identifier (string) of the factor (e.g. 'f0')
   * @param z measurement Z
   * @param z_cov measurement covariance
   * @param keys_id identifiers of the keys (e.g. "x0" , "x1")
   * @param tup_init_points_ptr init point of each key (for NL solvers)
   */
  template <typename DerivedEuclidianFactor,
            const char* FactorLabel,
            typename MEASURE_META,
            typename... KCCs>
  class EuclidianFactor
      : public BaseFactor<
            EuclidianFactor<DerivedEuclidianFactor, FactorLabel, MEASURE_META, KCCs...>,
            FactorLabel,
            MEASURE_META,
            KCCs...>
  {
    friend DerivedEuclidianFactor;

    public:
    using BaseFactor_t = BaseFactor<
        EuclidianFactor<DerivedEuclidianFactor, FactorLabel, MEASURE_META, KCCs...>,
        FactorLabel,
        MEASURE_META,
        KCCs...>;
    // declares friendlies so that they get to protected impl methods of this class
    friend DerivedEuclidianFactor;
    friend BaseFactor_t;
    // passing some type definitions for convenience
    using criterion_t                  = typename BaseFactor_t::criterion_t;
    using measure_t                    = typename BaseFactor_t::measure_t;
    using measure_cov_t                = typename BaseFactor_t::measure_cov_t;
    using matrices_Aik_t               = typename BaseFactor_t::matrices_Aik_t;
    using composite_state_ptr_t        = typename BaseFactor_t::composite_state_ptr_t;
    using composite_of_opt_state_ptr_t = typename BaseFactor_t::composite_of_opt_state_ptr_t;
    using KeysSet_t = typename BaseFactor_t::KeysSet_t;

    // check that the measure is euclidian (i.e. same as the criterion_t)
    static_assert(std::is_same_v<measure_t, criterion_t>);

    // rosie is a precious name for rho*z
    const criterion_t rosie = this->rho * this->z;

    /**
     * @brief Constructor
     *
     * @param factor_id
     * @param factor_id identifier (string) of the factor (e.g. 'f0')
     * @param z measurement Z
     * @param z_cov measurement covariance
     * @param keys_id array identifiers of the keys (e.g. "x0" , "x1"). Order must respect the set
     * KCCs order
     * @param tup_init_points_ptr init point of each key (for NL solvers)
     */
    EuclidianFactor(const std::string&                                    factor_id,
                    const measure_t&                                      z,
                    const measure_cov_t&                                  z_cov,
                    const std::array<std::string, BaseFactor_t::kNbKeys>& keys_id,
                    const composite_state_ptr_t&                          tup_init_points_ptr)
        : BaseFactor_t(factor_id, z, z_cov, keys_id, tup_init_points_ptr)
    {
    }

    /**
     * @brief static method initial point guesstimator. At factor creation, an initial value for some
     * key points is missing (e.g. observation of a new landmark) For every combination of
     * missing/available key points, this method **tries to** guess an initial point for the missing
     * ones by using the factor model. If a combination can't fill a missing init point(e.g. first
     * bearing-only observation of a new 2D landmark ), a null option value  is returned.
     *
     * Since init points for all keys are necessary for the factor construction, failure to guess one
     * or more missing initial key point prevents the factor instantiation. It is the responsibility
     * of system designer to come up with adequate recovery approaches at higher level (e.g. waiting
     * for several bearing-only observations to define triangulation scheme).
     *
     * If all initial values are available anyway, this method does nothing
     *
     * @param x_init_ptr_optional_tup tuple containing, for each key in the factor type, the optional
     * pointers to values of initial key points value. If no initial value is available in inputs,
     * std::nullopt must be passed.
     * @param z measurement M
     * @return optional of either:
     *            - nullopt if the guesstimator has failed for one or more points.
     *            - a tuple of completed pointers to the values
     */
    static std::optional<composite_state_ptr_t>
        guess_init_key_points_impl(composite_of_opt_state_ptr_t x_init_ptr_optional_tup,
                                   const criterion_t&           z)
    {
      return DerivedEuclidianFactor::guess_init_key_points_impl(x_init_ptr_optional_tup, z);
    }

    protected:
    /**
     * @brief method template that computes matrices Ai and bi of this factor instance
     *  bi:= -  r( \bar X )  where \bar X is the linearization point
     *  Ai:=  \frac{ \partial r(X) } {\partial X} at \bar X .
     *
     *  The Ai matrix is actually given in a set of Aik (one Aik per key)
     *
     *  The method varies on linear factors whether or not the SLAM system is linear
     *
     * @return nested tuple of {bi , tuple (Aiks...) }
     */
    std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_at_impl(const composite_state_ptr_t & X) const
    {
      // if 
      // if constexpr (
      //     // HACK: branch to the derived (bad pattern)
      //           std::is_member_function_pointer_v< decltype(&DerivedEuclidianFactor::compute_Ai_bi_at_impl)>
      //             )
      // {
      //   return  static_cast<const DerivedEuclidianFactor*>(this)->compute_Ai_bi_at_impl(X); 
      // }
      // else
      // {
      criterion_t bi = - this->compute_r_of_x_at(X);
      // matrices_Aik_t all_Aik ;
      // double apply pattern to zip two tuples
      auto all_Aik = std::apply([this,&X](const auto& ... kcc)
          {
            return std::apply([&](const auto & ... Xkptr)
                {
                  return std::make_tuple(kcc.compute_Aik_at(*Xkptr) ...);
                },X);
          }
          ,this->keys_set
          );

      return { bi, all_Aik };
      // }
    }


    /**
     * @brief compute the value r(X) at X
     *
     * @param X tuple of pointers to Xk values (one Xk per key)
     * @return r(X) value (eigen vector)
     */
    criterion_t compute_r_of_x_at_impl(const composite_state_ptr_t& X) const
    {
      return this->rho * this->compute_h_of_x(X) - this->rosie;
    }

    /**
     * @brief compute the value h(X) at X
     *
     * @param X X tuple of pointers to Xk values (on Xk per key)
     * @return h(X) value (eigen vector)
     */
    criterion_t compute_h_of_x(const composite_state_ptr_t& X) const
    {
      return static_cast<const DerivedEuclidianFactor*>(this)->compute_h_of_x_impl(X);
    }
  };


  //------------------------------------------------------------------//
  //                     Linear  Euclidian Factor                     //
  //            z, h(X), and the keys X are all euclidian             //
  //                        h(X) = H.X is linear                      //
  //------------------------------------------------------------------//
  /**
   * @brief Class template that holds:
   * - a gaussian pdf of a measurement vector Z given its set of -hidden- variables \mathcal X.
   * - the model that links the measurement it to its set of hidden variables \cal X := { X_k }, also
   * called keys.
   *
   * The Linear Factor template 'statically' derives from the Base Factor. It cover the cases where
   * r(X) is of the form: r(X) = \rho ( HX - z ) I.e. the measure z and h(X)=HX are both euclidian
   * vectors Additionnaly, the keys Xk of X are also euclidian vectors (no tricky manifolds such as
   * SE(n) or SO(n), just \mathbb R^n )
   *
   * A factor is a representation of the pdf likelihood p(Z = z | \cal X) .
   * It is assumed that p( Z = z | \mathcal X ) \propto \exp^ { - r(X;Z)^T r(X;Z) }
   * This block is class template because:
   * - the meta type, the number of keys varies factor to factor
   * - the measurement meta type varies factor to factor
   * - the model r(X;Z) varies factor to factor, which in turns implies that the Jacobian derivatives
   *   of this function wrt X do vary too.
   *
   * The API exposes the computation of Ai and bi to be used by modern SLAM systems and the negative
   * log likelihood norm at a given point.
   *
   *   A factor implementation must 'statically' inherits from this template or its refined
   * derivatives
   *
   * @tparam DerivedEuclidianFactor derived class that implements all of or part of the API specific
   * to a factor type
   * @tparam FactorLabel labelization of the factor type
   * @tparam MEASURE_META meta data of the measurement (dimensions, components name ...)
   * @tparam KCCs Contextual Conduct of a Key inside this factor type. See KeyContextualConduct
   * class template.
   * @param factor_id identifier (string) of the factor (e.g. 'f0')
   * @param z measurement Z
   * @param z_cov measurement covariance
   * @param keys_id identifiers of the keys (e.g. "x0" , "x1")
   * @param tup_init_points_ptr init point of each key (for NL solvers)
   */
  template <typename DerivedLinearEuclidianFactor,
            const char* FactorLabel,
            typename MEASURE_META,
            typename... LinearKCCs>
  class LinearEuclidianFactor   // the measure is euclidian and the keys are expressed in Euclidian
                                 // space too
      : public BaseFactor<LinearEuclidianFactor<DerivedLinearEuclidianFactor,
                                                      FactorLabel,
                                                      MEASURE_META,
                                                      LinearKCCs...>,
                               FactorLabel,
                               MEASURE_META,
                               LinearKCCs...>
  {
    public:
    using BaseFactor_t
        = BaseFactor<LinearEuclidianFactor<DerivedLinearEuclidianFactor,
                                                 FactorLabel,
                                                 MEASURE_META,
                                                 LinearKCCs...>,
                          FactorLabel,
                          MEASURE_META,
                          LinearKCCs...>;
    // declares friendlies so that they get to protected impl methods of this class
    friend DerivedLinearEuclidianFactor;
    friend BaseFactor_t;
    // passing some type definitions for convenience
    using criterion_t                  = typename BaseFactor_t::criterion_t;
    using measure_t                    = typename BaseFactor_t::measure_t;
    using measure_cov_t                = typename BaseFactor_t::measure_cov_t;
    using matrices_Aik_t               = typename BaseFactor_t::matrices_Aik_t;
    using composite_state_ptr_t        = typename BaseFactor_t::composite_state_ptr_t;
    using composite_of_opt_state_ptr_t = typename BaseFactor_t::composite_of_opt_state_ptr_t;
    using KeysSet_t = typename BaseFactor_t::KeysSet_t;
    // all key conduct are trivial manifold
    static_assert((LinearKCCs::kIsTrivialManifold && ...));
    // measurement generative model wrt X is linear
    static_assert((LinearKCCs::kLinear && ...));

    // rosie is a precious name for rho*z
    const criterion_t rosie = this->rho * this->z;

    /**
     * @brief Constructor
     *
     * @param factor_id
     * @param factor_id identifier (string) of the factor (e.g. 'f0')
     * @param z measurement Z
     * @param z_cov measurement covariance
     * @param keys_id array identifiers of the keys (e.g. "x0" , "x1"). Order must respect the set
     * KCCs order
     * @param tup_init_points_ptr init point of each key (for NL solvers)
     */
    LinearEuclidianFactor(const std::string&                                             factor_id,
                           const measure_t&                                               z,
                           const measure_cov_t&                                           z_cov,
                           const std::array<std::string, BaseFactor_t::kNbKeys>& keys_id,
                           const composite_state_ptr_t& tup_init_points_ptr)
        : BaseFactor_t(factor_id, z, z_cov, keys_id, tup_init_points_ptr)
    {
    }

    /**
     * @brief static method initial point guesstimator. At factor creation, an initial value for some
     * key points is missing (e.g. observation of a new landmark) For every combination of
     * missing/available key points, this method **tries to** guess an initial point for the missing
     * ones by using the factor model. If a combination can't fill a missing init point(e.g. first
     * bearing-only observation of a new 2D landmark ), a null option value  is returned.
     *
     * Since init points for all keys are necessary for the factor construction, failure to guess one
     * or more missing initial key point prevents the factor instantiation. It is the responsibility
     * of system designer to come up with adequate recovery approaches at higher level (e.g. waiting
     * for several bearing-only observations to define triangulation scheme).
     *
     * If all initial values are available anyway, this method does nothing
     *
     * @param x_init_ptr_optional_tup tuple containing, for each key in the factor type, the optional
     * pointers to values of initial key points value. If no initial value is available in inputs,
     * std::nullopt must be passed.
     * @param z measurement M
     * @return optional of either:
     *            - nullopt if the guesstimator has failed for one or more points.
     *            - a tuple of completed pointers to the values
     */
    // TODO: For linear factors, this method could be written generically  (difficulty: **)
    static std::optional<composite_state_ptr_t>
        guess_init_key_points_impl(const composite_of_opt_state_ptr_t& x_init_ptr_optional_tup,
                                   const criterion_t&                  z)
    {
      return DerivedLinearEuclidianFactor::guess_init_key_points_impl(x_init_ptr_optional_tup, z);
    }

    /**
     * @brief compute Ai and bi in the case that the factor is linear AND the
     * system is linear (all other factors in the system are linear).
     * bi := rho* z .
     * This yields a different result for b, because , in linear systems, the 
     * Ai bi are in this context:    Ai X - bi .
     * While in NL system its   :      Ai dX - bi.
     * Hence the bi s are not the same
     * NOTE: this method does not appear in base factor
     *
     * @return nested tuple of {bi, tuple (Aiks ...)}
     */
    std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_linear() const
    {
      matrices_Aik_t all_Aik
        =
        std::apply(
            [this,&all_Aik](const auto& ... kcc)
            {
              return std::make_tuple( kcc.Aik ... );
            }
            ,this->keys_set
        );
      criterion_t bi = this->rosie;
      return { bi , all_Aik };
    }

    /**
     * @brief compute Ai and bi of this linear factor inside a NL system:  Ai . dX - bi.
     * This implentation is valid for all linear factors.
     *
     * @param X composite state of linearization points
     * @return tuple of { bi, tuple of Aiks } 
     */
    std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_at_impl(const composite_state_ptr_t & X) const
    {
      matrices_Aik_t all_Aik
        =
        std::apply(
            [this,&all_Aik](const auto& ... kcc)
            {
              return std::make_tuple( kcc.Aik ... );
            }
            ,this->keys_set
        );
      criterion_t bi = - this->compute_r_of_x_at(X);
      return { bi , all_Aik };
    }

    protected:
    /**
     * @brief compute the value h(X) at X for linear. This is a generic method for linear systems
     * as h(X) = HX = H_{k=1} * X_{k=1} + H_{k=2} * X_{k=2} + ...
     *
     * Nonlinear factors classes have to implement their own compute_h_of_x_impl methods
     *
     * @param X tuple of pointers to Xk values (on Xk per key)
     * @return h(X) value (eigen vector)
     */
    criterion_t compute_h_of_x(const composite_state_ptr_t& X) const
    {
      // h(X) = sum_k( Hik*Xk)
      // Nested tuples is a hacky way to *zip* those 2 tuples
      // We compute a tuple of { H_k*X_k , ... }
      auto HkXks = std::apply([this,&X](const auto & ... kcc)
      {
        return 
        std::apply([&](const auto & ... Xkptr)
            {
              return std::make_tuple(kcc.Hik* *Xkptr ...);
            }
            ,X);
      },this->keys_set);
      // We then sum up the elements of that tuple and return
      return std::apply([&](const auto & ...HkXk )
          {
            return (HkXk + ...) ;
          }
          ,HkXks);
    }

    /**
     * @brief compute the value r(X) at X. For LinearEuclidianFactor, the method 
     * is computed opportunely faster since each Aik=rho*Hik is already saved at constructor time
     * r(X) = AX - b
     *
     * @param X tuple of pointers to Xk values (one Xk per key)
     * @return r(X) value (eigen vector)
     */
    criterion_t compute_r_of_x_at_impl(const composite_state_ptr_t& X) const
    {
      // use double std::apply to zip-multiply tuples X and kcc.Aik 
      // then sum the elements of resulting tuple
      // then substract by b
      auto tuple_Aik_times_Xk =  std::apply(
          [&X](const auto & ... kcc)
          {
            return std::apply(
                [&](const auto & ... Xptr)
                {
                  return std::make_tuple(kcc.Aik* *Xptr  ...); 
                }
                , X);
          }
          ,this->keys_set);
      // r(X) =  - b  + Sum(Aik*Xk)
      return - this->rosie + std::apply([](const auto ... AikXk)
          {
            return (AikXk + ...);
          }, tuple_Aik_times_Xk);
    }

  };

}


//------------------------------------------------------------------//
//            Factor History & Factors History Container            //
//------------------------------------------------------------------//
template <typename FT>
struct FactorHistory
{
  using Factor_t                                       = FT;
  using measure_t                                      = typename FT::measure_t;
  const std::string                          factor_id = "NaS";
  static constexpr const char*               kFactorLabel {FT::kFactorLabel};
  static constexpr const char*               kMeasureName {FT::kMeasureName};
  const std::array<std::string, FT::kNbKeys> vars_id = {};
  std::vector<double>                        norms;
  const measure_t                            z;

  // ctor
  FactorHistory(const std::string&                          factor_id,
                const std::array<std::string, FT::kNbKeys>& vars_id,
                const measure_t&                            z)
      : factor_id(factor_id)
      , vars_id(vars_id)
      , z(z)
  {
  }

  // using measure_t = Factor_t::measure_t;
  // const measure_t z;

  void push_norm_value(double factor_norm) { norms.push_back(factor_norm); }
};

template <typename FT, typename... FTs>
struct FactorsHistoriesContainer
{
  using factors_histories_t = std::tuple<std::unordered_map<std::string, FactorHistory<FT>>,
                                         std::unordered_map<std::string, FactorHistory<FTs>>...>;

  // the data : maps of
  // Its wrapped in a tuple because the
  factors_histories_t factors_histories_container;

  static constexpr const std::size_t kNbFactorTypes {std::tuple_size_v<factors_histories_t>};


  template <typename Q_FT>
  void insert_new_factor_history(const std::string& factor_id,
                                 const Q_FT&        factor,
                                 double             factor_norm)
  {
    // static_assert(std::is_same_v<FT,Q_FT> || (std::is_same_v<FTs,Q_FT> || ...)  );
    // constexpr std::size_t I = get_correct_tuple_idx_of_factor_type<Q_FT>();
    //
    // auto [it, hasBeenPlaced] = std::get<I>(this->factors_histories_container)
    //   .try_emplace(factor_id, factor_id, factor.get_array_keys_id(),factor.z);
    // assert(hasBeenPlaced); // TODO: consider this a consistency check
    auto it = insert_new_factor_history<Q_FT>(factor_id, factor);

    // push the norm
    it->second.push_norm_value(factor_norm);
  }

  // overloard where
  // TODO: no overload have the one with 3 args call the one with 2 args
  template <typename Q_FT>
  auto insert_new_factor_history(const std::string& factor_id, const Q_FT& factor)
  {
    static_assert(std::is_same_v<FT, Q_FT> || (std::is_same_v<FTs, Q_FT> || ...));
    constexpr std::size_t I = get_correct_tuple_idx_of_factor_type<Q_FT>();

    auto [it, hasBeenPlaced]
        = std::get<I>(this->factors_histories_container)
              .try_emplace(factor_id, factor_id, factor.get_array_keys_id(), factor.z);
    assert(hasBeenPlaced);   // TODO: consider this a consistency check
    return it;
  }

  // convenience overload
  template <typename Q_FT>
  void insert_new_factor_history(const Q_FT& factor)
  {
    this->insert_new_factor_history(factor.factor_id, factor);
  }


  template <typename Q_FT>
  void push_data_in_factor_history(const std::string& factor_id, double factor_norm)
  {
    static_assert(std::is_same_v<FT, Q_FT> || (std::is_same_v<FTs, Q_FT> || ...));
    constexpr std::size_t I = get_correct_tuple_idx_of_factor_type<Q_FT>();
    // get marginal history ref
    auto factor_history_it = std::get<I>(this->factors_histories_container).find(factor_id);
    // TODO: assert(marginal_history_it != std::get<I>(this->marginal_history_tuple).end() );
    // push new norm
    factor_history_it->second.push_norm_value(factor_norm);
  }


  template <typename Q_FT, std::size_t I = 0>
  static constexpr std::size_t get_correct_tuple_idx_of_factor_type()
  {
    static_assert(I < kNbFactorTypes);
    if constexpr (std::is_same_v<
                      typename std::tuple_element_t<I, factors_histories_t>::mapped_type::Factor_t,
                      Q_FT>)
    {
      return I;
    }
    else { return get_correct_tuple_idx_of_factor_type<Q_FT, I + 1>(); }
  }
};


//------------------------------------------------------------------//
//                      Helper print functions                      //
//------------------------------------------------------------------//
// TODO:  refactor those method in a class template associated with
//        a factor class tempalte

/**
 * @brief Traverse a tuple and print
 *
 */
template <typename TUP, size_t I = 0>
constexpr void traverse_tup()
{
  if constexpr (I == std::tuple_size<TUP>::value) { return; }
  else
  {
    using KT = std::tuple_element_t<I, TUP>;
    std::cout << "\t\t+ Key Nature: " << KT::kKeyName << ".  Role: " << KT::kRole << '\n';

    // recursive call
    traverse_tup<TUP, I + 1>();
  }
}

// runtime traverse of the KeySet tuple
template <typename TUP, size_t I = 0>
void traverse_tup(const TUP& tup)
{
  if constexpr (I == std::tuple_size<TUP>::value) { return; }
  else
  {
    using KT = std::tuple_element_t<I, TUP>;
    std::cout << "\t\t+ Key Nature: " << KT::kKeyName << ".  Role: " << KT::kRole
              << ". Id: " << std::get<I>(tup).key_id << '\n';
    // std::cout << "\t\t\t A:\n" << std::get<I>(tup).A << '\n';

    // recursive call
    traverse_tup<TUP, I + 1>(tup);
  }
}

template <typename KC>
void print_KeyContextConduct(const KC& kcc)
{
  std::cout << "\t\t+ Key Name: " << KC::kKeyName << ".  Role: " << KC::kRole
            << ". Id: " << kcc.key_id << '\n';
}


// print static information of a factor label
template <typename FT>
constexpr void factor_print()
{
  std::cout << FT::kFactorLabel << '\n';
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n' << "\tKeys (in order):\n";

  // traverse statically the tuple of keys data
  traverse_tup<typename FT::KeysSet_t>();

  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << " { ";
  for (const auto& comp : FT::kMeasureComponentsName) std::cout << comp << " ";
  std::cout << "}\n";

  std::cout << "\t----- \n";
}

// This is the non static version
template <typename FT>
void factor_print(const FT& fact)
{
  Eigen::IOFormat
      CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "  ", ";");

  std::cout << FT::kFactorLabel << " - id : " << fact.factor_id << '\n';
  std::cout << "\tM: " << FT::kM << " ,  N: " << FT::kN << '\n' << "\tKeys (in order):\n";

  // traverse_tup(fact.keys_set);
  // for_each_in_tuple(fact.keys_set, [](const auto& kcc){
  //   // using KT = ;
  //   std::cout << "\t\t+ Key Nature: " << decltype(kcc)::kKeyName << ".  Role: " <<
  //   decltype(kcc)::kRole
  //             << ". Id: " << kcc.key_id << '\n';
  // });
  // for_each_in_tuple(fact.keys_set, &printtupelem);
  std::apply([](auto... kcc) { ((print_KeyContextConduct(kcc)), ...); }, fact.keys_set);


  std::cout << "\t Measure: " << FT::kMeasureName;
  std::cout << "\n\t\tz: { ";
  for (int i = 0; i < FT::kMeasureComponentsName.size(); i++)
  {
    std::cout << FT::kMeasureComponentsName[i] << ": " // this could use the constexpr get_component version
              << FT::MeasureMeta_t::get_component(FT::kMeasureComponentsName[i], fact.z)
              << "  ";
  }
  std::cout << "}\n\t\t Cov: [ " << fact.z_cov.format(CommaInitFmt) << " ] \n";


  std::cout << "\t----- \n";
}

// TODO: URGENT: print a composite_state_ptr_t (hint: cppman tuple)
//
// TODO: URGENT: print tup_bi_Aiks

#endif
