#pragma once

// #include "utils/tuple_patterns.h"
// #include "utils/utils.h"

#include <array>
#include <memory>
#include <Eigen/Dense>
#include <map>
#include <optional>
#include <tuple>
#include <utility>

// for helpers (stringify)
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string_view>


// WARNING: SRP
// include a factor/utils.h with all the print function templates

namespace details_sam::Conduct{

  /**
   * @brief KeyContextualConduct  -- The class describes how a key (associated with a key meta)
   * behaves in a certain context (in a factor, with a measure type). 
   * It stores dimension informations name and most
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
    const std::string key_id; // the  only non-static element

    using key_process_matrix_t = Eigen::Matrix<double, kM, kN>;
    using measure_cov_t        = Eigen::Matrix<double, kM, kM>;
    using tangent_space_vect_t = Eigen::Matrix<double, kN, 1>;

    // euclidian space is a trivial manifold (same as its tangent space)
    static constexpr bool kIsTrivialManifold = std::is_same_v<Key_t, tangent_space_vect_t>;

    /**
     * @brief compute Hik, the jacobian matrix, a.k.a. the partial derivative of the factor
     * criterion premultiply by the root of the measure precision Hik = \partial r_i(X) /
     * \partial X_k Index i refers to the usual iterator on factor, k to the kth key of factor \phi_i
     *
     * @param Xk linearization point
     * @return the process matrix associated with this key. Note that it only a subset of the columns
     * of the full process matrix of factor \phi_i as there may be other keys k in the factor with
     * their own sub process-matrix Hik.
     */
    key_process_matrix_t compute_Hik_at(const Key_t & Xk) const
    {
      return static_cast<const DerivedKCC*>(this)->compute_Hik_at_impl(Xk);
    }

    /**
     * @brief Constructor
     *
     * @param key_id identifier of the key
     */
    KeyContextualConduct(const std::string& key_id)//, const measure_cov_t& rho)
        : key_id(key_id)
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

    // one for each instance (not static, couldn't make it static), better for cache locality
    const key_process_matrix_t Hik  {*Hik_MATRIX_PTR}; // by copy at construction time, non-static

    // this is a linear context for this KCC template
    static constexpr const bool        kLinear {true};

    // inherite ctor
    LinearKeyContextualConduct(const std::string& key_id)
      : base_kcc_t(key_id)
    {
    }

    key_process_matrix_t compute_Hik_at_impl(const Key_t & Xk) const
    {
      // this is not supposed to be called
      return this->Hik;
    }

  };
}

namespace sam::Factor
{
  // TODO: urgent: move CompositeState in separate header for clarity (as well with KCCs actually)
  // composite state
  template <typename ...KCCs>
  struct CompositeStatePtr
  {
    using type = std::tuple<std::shared_ptr<typename KCCs::Key_t>...>;
    static constexpr auto size = sizeof...(KCCs);
    using kccs_type = std::tuple<KCCs...>;
  };
  template <typename ...KCCs>
  using CompositeStatePtr_t = typename CompositeStatePtr<KCCs...>::type;

  // template <typename FT>
  // struct CompositeStatePtr : CompositeStatePtr<typename FT::KeysSet_t>{};

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
            typename KCC,
            typename... KCCs>
  class BaseFactor
  {
    public:
    friend DerivedFactor;
    using MeasureMeta_t = typename KCC::MeasureMeta_t;
    using measure_t      = typename MeasureMeta_t::measure_t; // FIX: URGENT: rename Measure_t
    static constexpr const char* kFactorLabel {FactorLabel};
    // *** the syntax:  N + (Ns + ...) has no fallback when Ns pack is empty
    static constexpr size_t      kN      = (KCC::kN  + ... + KCCs::kN );  // ***
    static constexpr size_t      kM      = MeasureMeta_t::kM;
    static constexpr size_t      kNbKeys = 1 + sizeof...(KCCs);
    using criterion_t                    = Eigen::Vector<double, kM>;
    using measure_cov_t                  = Eigen::Matrix<double, kM, kM>;
    using factor_process_matrix_t        = Eigen::Matrix<double, kM, kN>;
    using state_vector_t                 = Eigen::Matrix<double, kN, 1>;   // { dXk , ... }
    using composite_state_ptr_t = CompositeStatePtr_t<KCC,KCCs...>;
        // = std::tuple<std::shared_ptr<typename KCC::Key_t>, std::shared_ptr<typename KCCs::Key_t>...>;   // {*Xk ...}
    using composite_of_opt_state_ptr_t
        = std::tuple<std::optional<std::shared_ptr<typename KCC::Key_t>>,std::optional<std::shared_ptr<typename KCCs::Key_t>>...>;
    //  NOTE: Xk same type as dXk in euclidian factors
    using matrix_Ai_t    = Eigen::Matrix<double, kM, kN>;
    using matrices_Aik_t = std::tuple<typename KCC::key_process_matrix_t, typename KCCs::key_process_matrix_t...>;
    using KeysSet_t      = std::tuple<KCC,KCCs...>;
    using KeyMetas_t     = std::tuple<typename KCC::KeyMeta_t, typename KCCs::KeyMeta_t...>; // not unique

    static constexpr const char*                 kMeasureName {MeasureMeta_t::kMeasureName};
    static constexpr auto kMeasureComponentsName = MeasureMeta_t::components;
    const std::string                            factor_id;   // fill at ctor
    const measure_t                              z;           // fill at ctor
    const measure_cov_t                          z_cov;       // fill at ctor
    const measure_cov_t                          rho;         // fill at ctor
    // const std::array<std::string, kNbKeys>       keys_id;     // fill at ctor
    KeysSet_t                                    keys_set;    // fill at ctor, mutable NOTE: perhaps shouldnt be mutable

    static constexpr bool isLinear = KCC::kLinear && (KCCs::kLinear && ...);

    // check that all measure_t are the same in key contextual conduct
    static_assert( (std::is_same_v<typename KCC::MeasureMeta_t, typename KCCs::MeasureMeta_t> && ...) );

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
               const std::array<std::string, kNbKeys>& keys_id)
      // , const composite_state_ptr_t&            tup_init_points_ptr)
        : z(z)
        , z_cov(z_cov)
        , factor_id(factor_id)
        , rho(z_cov.inverse().llt().matrixL().transpose())  // cov^-1 =: LL^T
        // , keys_id(keys_id)
        , keys_set(construct_keys_set<KCC,KCCs...>(keys_id))
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
                              const measure_t&           z)
    {
      return DerivedFactor::guess_init_key_points_impl(x_init_ptr_optional_tup, z);
    }

    static composite_state_ptr_t make_composite(typename KCC::Key_t key, typename KCCs::Key_t... keys)
    {
      return std::make_tuple(std::make_shared<typename KCC::Key_t>(key), std::make_shared<typename KCCs::Key_t>(keys) ...);
    }

    /**
     * @brief method that returns an array made of the keys id of this factor instance
     * The order correspond to the one given at construction time.
     *
     * @return array made of the keys id (array of string) of this factor instance
     */
    std::array<std::string, kNbKeys> get_array_keys_id() const
    {
      return 
        std::apply([](const auto & ...key_set)
                    {
                      return std::array<std::string, kNbKeys>{key_set.key_id ... }; 
                    }
                    ,this->keys_set
                  );
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

    //   // WARNING: SRP: remove
    // std::tuple<criterion_t, matrices_Aik_t> compute_Ai_bi_at_current_lin_point() const
    // {
    //   auto X = get_key_points();
    //   return this->compute_Ai_bi_at(X);
    // }

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

    // /**
    //  * @brief compute r(\bar X) where \bar X is the curent linearization point
    //  *
    //  * @return r(\bar X) value (eigen vector)
    //  */
    // criterion_t compute_r_of_x_at_current_lin_point() const
    //   // WARNING: SRP: remove
    // {
    //   // build back the tup of stored lin point
    //   auto lin_point_tup = this->get_key_points();
    //   return compute_r_of_x_at(lin_point_tup);
    // }

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

    // /**
    //  * @brief returns square root of the negative log likelihood at the current linearization point
    //  * \bar X.
    //  *  - log ( p(Z|X) ) = r(X;Z)^T r(X;Z) = || r(X;Z) ||^2_2 = norm ^2
    //  *
    //  * @return scalar L2 norm
    //  */
    // double factor_norm_at_current_lin_point() const
    // {
    //   // WARNING: SRP: remove
    //   return compute_r_of_x_at_current_lin_point().norm();
    // }

    private:
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

    // /**
    //  * @brief method that gets the tuple of pointers to the active
    //  * linearization points
    //  *
    //  * @return tuple of key values \bar Xk (one for each key in the factor)
    //  */
    // composite_state_ptr_t get_key_points() const
    // {
    //   // WARNING: SRP: remove
    //   // REFACTOR: why not make it a const member, at worst it would mean that the 
    //   // class holds 2 times the same shared_ptr for each key (one in keys_set.key_mean_view
    //   // , the other in that new tuple member)
    //   // const composite_state_ptr_t lin_points_view; 
    //   composite_state_ptr_t current_lin_points_ptr;
    //   std::apply([this, &current_lin_points_ptr](const auto& ... kcc)
    //       {
    //         current_lin_points_ptr = { kcc.key_mean_view ...};
    //       }
    //       , this->keys_set
    //       );
    //   return current_lin_points_ptr;
    // }
    
    /**
     * @brief construct keys set (tuple of key contextual conducts objects)
     *
     * @tparam allKCCs types of all key contextual conducts (KCC and KCCs into one expansion)
     *   HACK: cheap trick so that my pack allKCCs has same size as keys_id & init_points, allowing valid expansion in std::apply
     * @param keys_id keys_id
     * @param rho square root matrix (upper triangular) of the measurement covariance
     * @param init_points composite state of initial points
     * @return keys set of contextual conduct (tuple)
     */
    template <typename ...allKCCs>
    KeysSet_t construct_keys_set( const std::array<std::string, kNbKeys> & keys_id)
    {
      return 
        std::apply
        (
          [&](const auto & ... key_id)
          {
            return std::make_tuple( allKCCs(key_id)... );
          //   std::apply([&](const auto & ...key_id)
          //             {
          //                 // std::tuple<KCC> key_set0 = { KCC(key_id, rho, init_point) };     
          //                 // auto kss =std::make_tuple( KCCs(key_id,rho,init_point) ... );           
          //                return std::make_tuple( allKCCs(key_id) ... );           
          //             }
          //             ,keys_id
          //           ); 
          }
          , keys_id  
        );
      // return keys_set;
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
            typename KCC,
            typename... KCCs>
  class EuclidianFactor
      : public BaseFactor<
            EuclidianFactor<DerivedEuclidianFactor, FactorLabel, KCC, KCCs...>,
            FactorLabel,
            KCC,
            KCCs...>
  {
    friend DerivedEuclidianFactor;

    public:
    using BaseFactor_t = BaseFactor<
        EuclidianFactor<DerivedEuclidianFactor, FactorLabel, KCC, KCCs...>,
        FactorLabel,
        KCC,
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
                    const std::array<std::string, BaseFactor_t::kNbKeys>& keys_id)
                    // const composite_state_ptr_t&                          tup_init_points_ptr)
        : BaseFactor_t(factor_id, z, z_cov, keys_id)
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
                                   const measure_t&           z)
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
      criterion_t bi = - this->compute_r_of_x_at(X);
      // matrices_Aik_t all_Aik ;
      // double apply pattern to zip two tuples
      matrices_Aik_t all_Aik = std::apply([this,&X](const auto& ... kcc)
          {
            return 
              std::apply([&](const auto & ... Xkptr)
                  {
                    return std::make_tuple(this->rho*kcc.compute_Hik_at(*Xkptr) ...);
                  },X);
          }
          ,this->keys_set
          );

      return { bi, all_Aik };
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
            typename LinearKCC,
            typename... LinearKCCs>
  class LinearEuclidianFactor   // the measure is euclidian and the keys are expressed in Euclidian
                                 // space too
      : public BaseFactor<LinearEuclidianFactor<DerivedLinearEuclidianFactor,
                                                      FactorLabel,
                                                      LinearKCC,
                                                      LinearKCCs...>,
                               FactorLabel,
                               LinearKCC,
                               LinearKCCs...>
  {
    public:

    using BaseFactor_t
        = BaseFactor<LinearEuclidianFactor<DerivedLinearEuclidianFactor,
                                                 FactorLabel,
                                                 LinearKCC,
                                                 LinearKCCs...>,
                          FactorLabel,
                          LinearKCC,
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
    const criterion_t rosie;// = this->rho * this->z;

    const matrices_Aik_t all_Aik;

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
                           const std::array<std::string, BaseFactor_t::kNbKeys>& keys_id)
                           // const composite_state_ptr_t& tup_init_points_ptr)
        : BaseFactor_t(factor_id, z, z_cov, keys_id)
          , all_Aik(
              std::apply(
                [this](const auto& ...kcc)
                {
                  // HACK: it seems I must cast otherwise I get Eigen::Product< T1, T2> whichh doesn't cast automatically (somehow) into Eigen< T3> where T3 is the matrix type of T1*T2
                  // return std::make_tuple( std::decay_t<std::remove_const_t<decltype(kcc.Hik)>>(this->rho * kcc.Hik) ... );
                  return std::make_tuple( typename std::decay_t<std::remove_const_t<decltype(kcc)>>::key_process_matrix_t(this->rho * kcc.Hik) ... );
                }
                ,this->keys_set)
              )
            , rosie(this->rho*this->z)
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
                                   const measure_t&                  z)
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
      criterion_t bi = this->rosie;
      return { bi , this->all_Aik };
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
      criterion_t bi = - this->compute_r_of_x_at(X);
      return { bi , this->all_Aik };
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
              // here we need Hik unfortunately
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
      return this->rho * this-> compute_h_of_x(X) - this->rosie;    
    }

  };

  //------------------------------------------------------------------//
  //                      Helper print functions                      //
  //------------------------------------------------------------------//
// helper to print
template <typename... KCCs>
struct stringify_composite_state_blockliner
{
  static std::string str(const CompositeStatePtr_t<KCCs...>&             X,
                         const std::array<std::string, sizeof...(KCCs)>& keys_id,
                         int                                             tabulation = 4,
                         int                                             precision  = 4)
  {
    std::stringstream ss;
    // zip tupple pattern
    std::apply(
        [&](auto&&... key_id)
        {
          std::apply(
              [&](auto&&... Xk_ptr)
              {
                ((ss << std::setw(tabulation) << "[ " << key_id << " ] : \t"
                     << KCCs::KeyMeta_t::stringify_key_oneliner(*Xk_ptr, precision) << '\n'),
                 ...);
              },
              X);
        },
        keys_id);
    return ss.str();
  }
};

template <typename... T_KCCs>
struct stringify_composite_state_blockliner<std::tuple<T_KCCs...>>
    : stringify_composite_state_blockliner<T_KCCs...>
{
};


// static version
template <typename KC>
std::string stringify_keyconduct_oneliner()
{
  std::stringstream ss;
  ss << KC::kRole << " ( " << KC::kKeyName << " ) \n";
  return ss.str();
}

template <typename KC>
std::string stringify_keyconduct_oneliner(const KC& kcc)
{
  std::stringstream ss;

  ss << kcc.key_id << " <- " << KC::kRole << " ( " << KC::kKeyName << " ) \n";
  return ss.str();
}


// print static information of a factor label (cant be constexpr)
template <typename FT>
std::string stringify_factor_blockliner(int tab = 4, int precision = 4)
{
  std::stringstream ss;
  ss << std::setw(tab) << " " << FT::kFactorLabel << ". M: " << FT::kM << ", N: " << FT::kN << '\n';
  ss << std::setw(tab + 2) << " "
     << "Keys :\n";
  // FIX: find a way to extract the KCC static info
  // std::apply([&](auto&... kcc)
  //             {
  //               ((ss << std::setw(tab+4) << "+ " << stringify_keyconduct_oneliner(kcc) ), ...);
  //             }
  //           , fact.keys_set);

  ss << std::setw(tab + 2) << " "
     << "Measurement: " << FT::kMeasureName << '\n';
  ss << std::setw(tab + 4) << " "
     << "z   : " << FT::MeasureMeta_t::stringify_measure_oneliner() << '\n';

  return ss.str();
}

// pretty print factor
template <typename FT>
std::string stringify_factor_blockliner(const FT& fact, int tab = 4, int precision = 4)
{
  std::stringstream ss;

  Eigen::IOFormat
      CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "  ", ";");

  ss << std::setw(tab) << " " << fact.factor_id << " : " << FT::kFactorLabel << ". M: " << FT::kM
     << ", N: " << FT::kN << '\n';
  ss << std::setw(tab + 2) << " "
     << "Keys :\n";
  std::apply(
      [&](auto&... kcc)
      {
        ((ss << std::setw(tab + 4) << "+"
             << " " << stringify_keyconduct_oneliner(kcc)),
         ...);
      },
      fact.keys_set);

  ss << '\n'
     << std::setw(tab + 2) << " "
     << "Measurement: " << FT::kMeasureName << '\n';
  ss << std::setw(tab + 4) << " "
     << "z   : " << FT::MeasureMeta_t::stringify_measure_oneliner(fact.z) << '\n';
  ss << std::setw(tab + 4) << " "
     << "cov : [ " << fact.z_cov.format(CommaInitFmt) << " ] \n";

  return ss.str();
}
}
