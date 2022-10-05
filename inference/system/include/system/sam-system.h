#pragma once

#include "marginal/marginal.h"
#include "solver/solver.h"
#include "system/MatrixConverter.hpp"
#include "system/PersistentFactor.h"
#include "system/SystemConverter.hpp"
#include "system/GraphConverter.hpp"
#include "system/GraphConverter.h"
#include "system/HybridConverter.h"
#include "system/system_jsonify.h"
#include "utils/config.h"
#include "utils/tuple_patterns.h"
#include "utils/utils.h"

#include <execution>
#include <functional>
#include <iomanip>
#include <numeric>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
// boost (graph system only)
#include <amd.h>

// namespace bmi = ::boost::multi_index;


namespace sam::Inference
{

  struct SystemHeader
  {
    std::string agent_id;
    std::string system_label;
    // std::string solver_name;
    std::size_t nbSequence = 0;

    SystemHeader(const std::string& agent_id, const std::string& system_label)
        : agent_id(agent_id)
        , system_label(system_label)
    {
    }
  };






  //------------------------------------------------------------------//
  //                           Base System                            //
  //------------------------------------------------------------------//
  template <typename DerivedSystem, typename OPTIM_STATS_T, typename OPTIM_OPTIONS_T, typename FACTOR_T, typename... FACTORS_Ts>
  class BaseSystem
  {
    public:
    // keymetae (latin plural <3) remove duplicates
    using KeyMetae_t = typename sam_tuples::tuple_filter_duplicate<
        sam_tuples::tuple_cat_t<typename FACTOR_T::KeyMetas_t,
                                typename FACTORS_Ts::KeyMetas_t...>>::type;

    // declare marginal container type of those keymetas
    using Vectors_Marginals_t = Marginal::MarginalsVectorCollection<KeyMetae_t>;
    using Map_Marginals_t = typename ::sam::Marginal::MarginalsCollection<KeyMetae_t>::type;
    using type = BaseSystem<DerivedSystem,OPTIM_STATS_T,OPTIM_OPTIONS_T, FACTOR_T, FACTORS_Ts...>;
    using SystemHeader = SystemHeader;
    // using OptimOptions = OptimOptions<SOLVER_T>;
    // using OptimStats   = OptimStats<SOLVER_T>;
    // using SolverStats  = typename SOLVER_T::Stats_t;

    // friend DerivedSystem;

    // TODO: remove potential duplicates in the factor types (or alternatively trigger a static_assert fail)

    static constexpr const bool isSystFullyLinear
        = FACTOR_T::isLinear && (FACTORS_Ts::isLinear && ...);

    static constexpr std::size_t kNbFactorTypes = 1 + sizeof...(FACTORS_Ts);
    static constexpr std::size_t kNbKeyTypes    = std::tuple_size_v<KeyMetae_t>;

    // // for later
    // using Wrapped_Factors_t = std::tuple<
    //   bmi::multi_index_container< ::sam::Factor::WrapperPersistentFactor<FACTOR_T, isSystFullyLinear>, 
    //                               bmi::indexed_by<
    //                                         bmi::random_access<
    //                                                           // bmi::identity< ::sam::Factor::WrapperPersistentFactor<FACTOR_T, isSystFullyLinear> >
    //                                                           >
    //                                             >
    //                           >
    //                           // >;
    //                           ,
    //   bmi::multi_index_container< ::sam::Factor::WrapperPersistentFactor<FACTORS_Ts, isSystFullyLinear>, 
    //                               bmi::indexed_by<
    //                                         bmi::random_access<
    //                                                           // bmi::identity< ::sam::Factor::WrapperPersistentFactor<FACTORS_Ts, isSystFullyLinear> >
    //                                                           >
    //                                                 >
    //                           >...
    //                           >;

    using Wrapped_Factors_t = std::tuple<
        std::vector<::sam::Factor::WrapperPersistentFactor<FACTOR_T, isSystFullyLinear>>,
        std::vector<::sam::Factor::WrapperPersistentFactor<FACTORS_Ts, isSystFullyLinear>>...>;

    SystemHeader header;

    // // WARNING: change struct to boost undirected graph
    // std::unordered_map<std::string, typename SystemConverter::KeyDispatchInfos> keys_affectation
    //     = {}; // FIX: keep it just for (sparse?) matrix based system

    using DispatchContainer_t = typename SystemConverter::DispatchContainer_t;

    DispatchContainer_t keys_affectation;
        // = {}; // FIX: keep it just for (sparse?) matrix based system


    /**
     * @brief constructor
     *
     * @param agent id
     */
    BaseSystem(const std::string& agent_id, const std::string& system_label = "inference system")
        : header(agent_id, system_label)
    {
    }

    OPTIM_STATS_T sam_optimise(const OPTIM_OPTIONS_T& options = OPTIM_OPTIONS_T() )
    {
      return static_cast<DerivedSystem*>(this)->sam_optimise_specialized(options);
    }

    void remove_factor(const std::string& factor_id)
    {
      // nothing happend if factor_id doesnt exist
      std::apply(
          [&](auto&... vec_of_wfactor)
          {
            // erase-remove idiom wrapped in expression expansion
            (
                // TODO: assert that number of eliminated element is 1 at most (returned value of
                // erase_if)
                std::erase_if(vec_of_wfactor,
                              [&](auto& wf) { return wf.factor.factor_id == factor_id; }),
                ...);
          },
          this->all_factors_tuple_);
      // TODO: remove smthing in GraphModel ?

      // TODO: the valid of any cache data should be questioned after removing a factor
    }

    void remove_key(const std::string& key_id)
    {
      // remove associated factors?
      // several landmines here: what happen if graph becomes unconnected etc...
      // TODO: the valid of any cache data should be questioned after removing a factor
    }

    void sum_out(const std::string& key_id)
    {
      // not the same as remove_key !
      // -> a marginalisation should occur first

      // TODO: the valid of any cache data should be questioned after removing a factor
    }

    auto get_all_factors() const { return this->all_factors_tuple_; }

    auto get_keys_affectation() const { return this->keys_affectation; } // FIX: keep it just for (sparse?) matrix based system

    // TODO: urgent get_joint-marginal etc...
    // joint_marginal<marginals_t> get_joint_marginal()
    // get_key_marginal()
    // get_full_joint(){}
    // get_full_joint_except( ... )

    // get all marginals
    auto get_marginals() const { return this->all_vectors_marginals_.vectors_of_marginals; }

    virtual void visitor_compute_keys_affectation()
    {
    }


    Map_Marginals_t get_marginals_as_map() const
    {
      // motivation: don't assume users utilisation of the key marginals collection
      //             e.g. for json it is better to have a vector
      //                  for most other purposes better to have a map
      Map_Marginals_t map_of_wmarginals;
      std::apply(
          [&](const auto&... vwm)
          {
            ((std::for_each(vwm.begin(),
                            vwm.end(),
                            [&](const auto& wm)
                            {
                              map_of_wmarginals.template insert_in_marginal_container<
                                  typename std::remove_cvref_t<decltype(vwm)>::value_type>(wm);
                            })),
             ...);
          },
          this->all_vectors_marginals_.vectors_of_marginals);
      return map_of_wmarginals;
    }

    // Marginals_t all_marginals_;
    Vectors_Marginals_t all_vectors_marginals_;


    // there's at least one factor, the rest are expanded
    Wrapped_Factors_t all_factors_tuple_;

    /**
     * @brief New factor registration
     *
     * @tparam FT
     * @param factor_id
     * @param mes_vect
     * @param measure_cov
     * @param keys_id
     */
    template <typename FT>
    void register_new_factor(const std::string&                          factor_id,
                             const typename FT::measure_t&               mes_vect,
                             const typename FT::measure_cov_t&           measure_cov,
                             const std::array<std::string, FT::kNbKeys>& keys_id,
                             const bool recompute_key_affectation = true)
    {
      static_assert(std::is_same_v<FT, FACTOR_T> || (std::is_same_v<FT, FACTORS_Ts> || ...),
                    "This type of factor doesnt exist ");
      std::string scope_name = "register factor " + factor_id;
      PROFILE_SCOPE(scope_name.c_str());

      // TODO: consistent management failure (throw ? return value false ?

      // look up factor tuple if factor_id already stored
      bool factor_id_already_exists = std::apply(
          [&factor_id](const auto&... vof)
          {
            return (std::ranges::any_of(vof,
                                        [&factor_id](const auto& wf)
                                        { return wf.factor.factor_id == factor_id; })
                    || ...);
          },
          this->all_factors_tuple_);

      if (factor_id_already_exists)
        throw std::runtime_error("factor " + factor_id + " already exists");

      // apply to emplace this factor in the correct container
      std::apply(
          [&, this](auto&... vect_of_wfactors)
          {
            ((this->emplace_factor_in<FT>(factor_id,
                                          mes_vect,
                                          measure_cov,
                                          keys_id,
                                          vect_of_wfactors)),
             ...);
          },
          this->all_factors_tuple_);
      // recompute keys_affectation
      // this is wasteful IF many factors are registered at once (batch slam)
      // because then it would just be easier to do it at the end
      if (recompute_key_affectation) // FIX: keep it just for (sparse?) matrix based system. Use a boost::graph for graph
      {
          this->keys_affectation = SystemConverter::compute_keys_affectation(
              this->all_factors_tuple_,
              this->all_vectors_marginals_.vectors_of_marginals);
          this->keys_affectation_unsync_ = false;
      }
      else this->keys_affectation_unsync_ = true;
    }


    // bundle
    template<typename FT>
    void register_factors_in_bundle(
         const std::vector<std::string                         > &                          v_factor_id,
         const std::vector<typename FT::measure_t              > &               v_measure,
         const std::vector<typename FT::measure_cov_t          > &           v_measure_cov,
         const std::vector<std::array<std::string, FT::kNbKeys>> & v_keys_id,
         const bool recompute_key_affectation = true)
    {
      PROFILE_FUNCTION();
      auto it_fid = v_factor_id.begin();
      auto it_measure = v_measure.begin();
      auto it_measure_cov = v_measure_cov.begin();
      auto it_keys_id = v_keys_id.begin();

      while (it_fid != v_factor_id.end())
      {
        this->register_new_factor<FT>(*it_fid,*it_measure,*it_measure_cov,*it_keys_id,false);
        it_fid ++;
        it_measure ++;
        it_measure_cov ++;
        it_keys_id ++;
      }
      if (recompute_key_affectation) // FIX: keep it just for (sparse?) matrix based system. Use a boost::graph for graph
      {
          this->keys_affectation = SystemConverter::compute_keys_affectation(
              this->all_factors_tuple_,
              this->all_vectors_marginals_.vectors_of_marginals);
          this->visitor_compute_keys_affectation();
          this->keys_affectation_unsync_ = false;
      }
      else this->keys_affectation_unsync_ = true;
    }

    protected:

    bool keys_affectation_unsync_ = false;

    template <typename FT, typename VECT_OF_WFT>
    void emplace_factor_in(const std::string&                          factor_id,
                           const typename FT::measure_t&               mes_vect,
                           const typename FT::measure_cov_t&           measure_cov,
                           const std::array<std::string, FT::kNbKeys>& keys_id,
                           VECT_OF_WFT&                                vector_of_wrapped_factors)
    {
      using WFT = typename VECT_OF_WFT::value_type;
      // only run if compatible type
      if constexpr (std::is_same_v<FT, typename WFT::Factor_t>)
      {
        // look up our container to see if we have existing means for each key
        typename FT::KeysSet_t                    KccSet = FT::construct_keys_set(keys_id);
        typename FT::composite_of_opt_state_ptr_t tuple_of_opt_means_ptr = std::apply(
            [this](auto... kcc)   // copy, no big deal
            {
              auto lambda = [this](const auto& akcc)   // -> std::optional<>
              {
                using keymeta_t = typename std::remove_cvref_t<decltype(akcc)>::KeyMeta_t;
                // do the key exist ?
                auto [it, marginal_found] = this->all_vectors_marginals_.template find_if<keymeta_t>(akcc.key_id);
                if (marginal_found)
                {
                  return std::optional<std::shared_ptr<typename keymeta_t::key_t>>(it->shared_mean);
                }
                else return std::optional<std::shared_ptr<typename keymeta_t::key_t>>(std::nullopt);
              };
              return std::make_tuple(lambda(kcc)...);
            },
            KccSet);
        // It is often the case that the above tuple contains at least 1 std::nullopt.

        // Attempt to guess the full init point for this factor by using the measurement if
        // necessary. If we don't have enough data to fill in the blank, then
        // `opt_tuple_of_init_point = std::nullopt`
        std::optional<typename WFT::composite_state_ptr_t> opt_tuple_of_init_point_ptr
            = FT::guess_init_key_points(tuple_of_opt_means_ptr, mes_vect);
        // NOTE: this is were we heap allocate for the mean of new keys (or so far unknown key)

        // if all keys are guessed or were already complete, fill the container of
        // the means for keys that were missing
        if (opt_tuple_of_init_point_ptr.has_value())
        {
          // hack: triple zip tupple pattern
          std::apply(
              [&, this](const auto&... opt_mean_ptr)
              {
                std::apply(
                    [&, this](const auto&... guessed_mean_ptr)
                    {
                      std::apply(
                          [&, this](auto... kcc)   // copy, no big deal though
                          {
                            auto lambda = [&, this](const auto& _opt_mean_ptr,
                                                    const auto& _guessed_mean_ptr,
                                                    auto        _kcc)
                            {
                              // if opt_mean_ptr has no value
                              //  then create a wrapped marginal with the guessed mean pointer
                              if (!_opt_mean_ptr.has_value())
                              {
                                using marginal_t = ::sam::Marginal::BaseMarginal<
                                    typename decltype(_kcc)::KeyMeta_t>;
                                using wrapped_marginal_t
                                    = ::sam::Marginal::WrapperPersistentMarginal<marginal_t>;
                                auto wrapped_marginal
                                    = wrapped_marginal_t(_kcc.key_id, _guessed_mean_ptr);
                                // TODO: intermediary step before updating the marginal: infer a
                                // covariance (difficulty ***)
                                this->all_vectors_marginals_.template push_back<wrapped_marginal_t>(
                                    wrapped_marginal);
                              }
                            };
                            ((lambda(opt_mean_ptr, guessed_mean_ptr, kcc)), ...);
                          },
                          KccSet);
                    },
                    opt_tuple_of_init_point_ptr.value());
              },
              tuple_of_opt_means_ptr);
        }
        else
        {
          // TODO: FEATURE: emplace factor in a staging container if unable to guess all factors
          // init points
          throw std::runtime_error("Unable to determine all init points for this factor");
        }
        // emplace back in the structure
        vector_of_wrapped_factors.emplace_back(factor_id,
                                               mes_vect,
                                               measure_cov,
                                               keys_id,
                                               opt_tuple_of_init_point_ptr.value());
      }
    }
  }; // end BaseSystem<>



  // FIX: move to matrix system
  template <typename SOLVER_T>
  struct OptimOptions
  {
    // max number of iterations (no effect if linear system)
    std::size_t max_iterations = 3;
    // allow the solver to cache informations (mind the memory impact)
    // to enable incremental inference
    bool cache_incremental = false;
    // compute the covariance (mind the time complexity)
    bool compute_covariance = true;
    // cache the full joint covariance (mind the memory impact, but allow for rapid joint covariance
    // queries)
    bool cache_covariance = this->compute_covariance
                            && false;   // cache_covariance (moot if no computation of covariance)
    // TODO: thresholding strategy (by key type?? -> leads to template)
    // solver options
    typename SOLVER_T::Options_t solver;
    // ctor
    OptimOptions() : solver(this->compute_covariance) {}

    // ctor
    OptimOptions(bool compute_covariance)
        : compute_covariance(compute_covariance)
        , solver(this->compute_covariance)
    {
    }
  };


  // FIX: move to matrix system
  template <typename SOLVER_T>
  struct OptimStats
  {
    // optimization success (or not)
    bool success;
    // forwards the options inputs (the rest is more output-ish)
    OptimOptions<SOLVER_T> optim_options;
    // the stats specifics to the solver
    typename SOLVER_T::Stats_t solver;
    // number of scalar nonzeros in the jacobian matrix
    std::size_t nbNZ_jacobian_scalar;
    // ratio of nonzeros in the jacobian matrix
    std::size_t ratioNZ_jacobian_scalar;
    // number of scalar nonzeros in the hessian matrix
    std::size_t nbNZ_hessian_scalar;
    // ratio of nonzeros in the hessian matrix
    std::size_t ratioNZ_hessian_scalar;
    // size M,N of the jacobian matrix
    std::size_t M, N;
    // size of the semantic jacobian matrix
    std::size_t M_semantic, N_semantic;

    // report message
    std::string report_str;
    // negative log likelihood of the product of factors
    double NLog_value_before;
    double NLog_value_after;
  };

  //------------------------------------------------------------------//
  //                        Matrix Base System                        //
  //------------------------------------------------------------------//
  template <typename DerivedMatrixSystem, typename SOLVER_T, typename FACTOR_T, typename... FACTORS_Ts>
  class MatrixBaseSystem : public BaseSystem<MatrixBaseSystem<DerivedMatrixSystem,SOLVER_T,FACTOR_T,FACTORS_Ts...>
                                            ,OptimStats<SOLVER_T>
                                            ,OptimOptions<SOLVER_T>
                                            ,FACTOR_T
                                            ,FACTORS_Ts...>
  {
    public:
      using Base_System_t = typename BaseSystem<MatrixBaseSystem<DerivedMatrixSystem,SOLVER_T,FACTOR_T,FACTORS_Ts...>
                                            ,OptimStats<SOLVER_T>
                                            ,OptimOptions<SOLVER_T>
                                            ,FACTOR_T
                                            ,FACTORS_Ts...>::type;

      using type = MatrixBaseSystem<DerivedMatrixSystem, SOLVER_T, FACTOR_T, FACTORS_Ts...>;
      using KeyMetae_t = typename Base_System_t::KeyMetae_t;
      using Vectors_Marginals_t = typename Base_System_t::Vectors_Marginals_t;
      using Map_Marginals_t = typename Base_System_t::Map_Marginals_t;
      using Wrapped_Factors_t = typename Base_System_t::Wrapped_Factors_t;
      using SystemHeader = typename Base_System_t::SystemHeader;
      using OptimOptions = OptimOptions<SOLVER_T>;
      using OptimStats   = OptimStats<SOLVER_T>;
      using SolverStats  = typename SOLVER_T::Stats_t;
      // using OptimOptions = typename Base_System_t::OptimOptions;
      // using OptimStats   = typename Base_System_t::OptimStats;
      // using SolverStats  = typename Base_System_t::SolverStats;

      static constexpr const bool isSystFullyLinear = Base_System_t::isSystFullyLinear;
      static constexpr std::size_t kNbFactorTypes   = Base_System_t::kNbFactorTypes;
      static constexpr std::size_t kNbKeyTypes      = Base_System_t::kNbKeyTypes;

    /**
     * @brief constructor
     *
     * @param agent id
     */
    MatrixBaseSystem(const std::string& agent_id, const std::string& system_label = "matrix inference system")
        : Base_System_t(agent_id,system_label)
    {
    }

    OptimStats sam_optimise_specialized(const OptimOptions& options)
    {
      return static_cast<DerivedMatrixSystem*>(this)->sam_optimise_matrix_specialized(options);
    }

  }; // end MatrixBaseSystem<..>






  //------------------------------------------------------------------//
  //                          Dense  System                           //
  //------------------------------------------------------------------//
  template <typename SOLVER_T,
            typename FACTOR_T,
            typename... FACTORS_Ts>   // I need at least one type of factor
  class DenseSystem : public MatrixBaseSystem<DenseSystem<SOLVER_T,FACTOR_T,FACTORS_Ts...>
                                        ,SOLVER_T
                                        ,FACTOR_T
                                        ,FACTORS_Ts...>
  {
    public:
      using Matrix_Base_System_t = typename MatrixBaseSystem<DenseSystem<SOLVER_T,FACTOR_T,FACTORS_Ts...>
                                        ,SOLVER_T
                                        ,FACTOR_T
                                        ,FACTORS_Ts...>::type;
      using type = DenseSystem<SOLVER_T, FACTOR_T, FACTORS_Ts...>;
      using KeyMetae_t = typename Matrix_Base_System_t::KeyMetae_t;
      using Vectors_Marginals_t = typename Matrix_Base_System_t::Vectors_Marginals_t;
      using Map_Marginals_t = typename Matrix_Base_System_t::Map_Marginals_t;
      using Wrapped_Factors_t = typename Matrix_Base_System_t::Wrapped_Factors_t;
      using SystemHeader = typename Matrix_Base_System_t::SystemHeader;
      using OptimOptions = typename Matrix_Base_System_t::OptimOptions;
      using OptimStats   = typename Matrix_Base_System_t::OptimStats;
      using SolverStats  = typename Matrix_Base_System_t::SolverStats;

      static constexpr const bool isSystFullyLinear = Matrix_Base_System_t::isSystFullyLinear;
      static constexpr std::size_t kNbFactorTypes   = Matrix_Base_System_t::kNbFactorTypes;
      static constexpr std::size_t kNbKeyTypes      = Matrix_Base_System_t::kNbKeyTypes;

    /**
     * @brief constructor
     *
     * @param agent id
     */
    DenseSystem(const std::string& agent_id, const std::string& system_label = "dense matrix inference system")
        : Matrix_Base_System_t(agent_id,system_label)
    {
    }

    OptimStats sam_optimise_matrix_specialized(const OptimOptions& options = OptimOptions())
    {
      // scoped timer
      PROFILE_FUNCTION();
      size_t M = SystemConverter::Scalar::M(this->all_factors_tuple_);
      size_t N = SystemConverter::Scalar::N(this->all_vectors_marginals_.vectors_of_marginals);
      size_t semantic_M = SystemConverter::Semantic::M(this->all_factors_tuple_);
      size_t semantic_N = SystemConverter::Semantic::N(this->all_vectors_marginals_.vectors_of_marginals);

      OptimStats optim_stats;
      if (M == 0) return optim_stats;   // set success bool to false

      auto natural_scalar_M_offsets = SystemConverter::Scalar::FactorTypeIndexesOffset(this->all_factors_tuple_);
      auto natural_scalar_N_offsets = SystemConverter::Scalar::MarginalTypeIndexesOffset( this->all_vectors_marginals_.vectors_of_marginals);
      auto natural_semantic_M_offsets = SystemConverter::Semantic::FactorTypeIndexesOffset(this->all_factors_tuple_);
      // auto semantic_N_type_idx_offsets =
      // MatrixConverter::Semantic::MarginalTypeIndexesOffset(this->all_vectors_marginals_.vectors_of_marginals);

      if (this->keys_affectation_unsync_)
      {
          this->keys_affectation = SystemConverter::compute_keys_affectation(
              this->all_factors_tuple_,
              this->all_vectors_marginals_.vectors_of_marginals);
          this->visitor_compute_keys_affectation();
          this->keys_affectation_unsync_ = false;
      }

      //------------------------------------------------------------------//
      //                            LOOP START                            //
      //------------------------------------------------------------------//
      // loop of the iterations
      int max_iterations;
      if constexpr (isSystFullyLinear)
        max_iterations = 1;
      else
        max_iterations = options.max_iterations;

      int nIter = 0;
      while (nIter < max_iterations)
      {
        // scoped timer
        std::string timer_name = "iter" + std::to_string(nIter);
        PROFILE_SCOPE(timer_name.c_str());

        auto [b, A] = MatrixConverter::Dense::compute_b_A(
            this->all_factors_tuple_,
            this->all_vectors_marginals_.vectors_of_marginals,
            this->keys_affectation,
            M,
            N,
            natural_scalar_M_offsets);

        // maximum a posteriori, may represent a \hat X or \delta \hat X (NL)
        Eigen::VectorXd                                 MaP;
        typename SOLVER_T::Stats_t                      solver_stats;
        std::shared_ptr<std::optional<Eigen::MatrixXd>> optional_covariance_ptr;
        std::tie(MaP, optional_covariance_ptr, solver_stats)
            = SOLVER_T::solve(A, b, options.solver);

#if ENABLE_DEBUG_TRACE
        {
          PROFILE_SCOPE("print console");
          std::cout << "#### Iteration : " << nIter << '\n';
          std::cout << "#### Syst: A(" << A.rows() << "," << A.cols() << ") computed :\n";
          // only display if matrix not too big
          if (A.rows() < 22 && nIter == 0) std::cout << Eigen::MatrixXd(A) << "\n\n";
          // std::cout << "#### Syst: R computed :\n" << Eigen::MatrixXd(A) <<
          // "\n\n";
          std::cout << "#### Syst: b computed :\n";
          if (b.rows() < 22 && nIter == 0) std::cout << b << "\n";
          std::cout << "#### Syst: MAP computed :\n" << MaP << '\n';
          // std::cout << "#### Syst: Covariance Sigma("<< SigmaCovariance.rows() <<","<<
          // SigmaCovariance.cols() <<") computed : \n" ; if (SigmaCovariance.rows()<15) std::cout
          // << SigmaCovariance << '\n';
        }
#endif

        //------------------------------------------------------------------//
        //                  POST SOLVER LOOP ON MARGINALS                   //
        //------------------------------------------------------------------//
        std::apply(
            [this, &MaP, &options, optional_covariance_ptr, &nIter, &natural_scalar_N_offsets](
                auto&... vect_of_wmarginals)
            {
              std::apply(
                  [&, this](const auto&... N_type_start_idx)
                  {
                    std::string scope_name = "save marginal updates";
                    PROFILE_SCOPE(scope_name.c_str());
                    // define the function
                    auto lambda_update_map_of_wmarginals
                        = [&, this](auto& vector_of_wrapped_marginals, std::size_t KeyTypeStartIdx)
                    {
                      using wrapped_marginal_t = typename std::remove_cvref_t<
                          decltype(vector_of_wrapped_marginals)>::value_type;
                      using marginal_t         = typename wrapped_marginal_t::Marginal_t;
                      using tangent_space_t    = typename marginal_t::Tangent_Space_t;
                      using keymeta_t          = typename marginal_t::KeyMeta_t;
                      constexpr std::size_t kN = marginal_t::KeyMeta_t::kN;
                      // looping over the marginal collection and updating them with the MAP result
                      // std::for_each (
                      //     // std::execution::par   // on M3500, sequential is still slightly
                      //     faster than par_unseq (1.4 ms vs 1.625 ms) ,
                      //     map_of_wrapped_marginals.begin() , map_of_wrapped_marginals.end() ,
                      //     [&,this]( auto & kvpair)
                      std::size_t idx_marg = 0;
                      for (auto it_marg = vector_of_wrapped_marginals.begin();
                           it_marg != vector_of_wrapped_marginals.end();
                           it_marg++)
                      {
                        // std::string key_id = it_marg->first;
                        std::string         key_id           = it_marg->key_id;
                        wrapped_marginal_t& wrapped_marginal = *it_marg;
                        // get the subvector from the Maximum A Posteriori vector
                        auto sysidx = KeyTypeStartIdx + idx_marg * kN;
                        idx_marg++;
                        auto MaP_subvector = MaP.block<kN, 1>(sysidx, 0);

                        // clear previous history at first iteration
                        if (nIter == 0) wrapped_marginal.clear_history();

                        // covariance, if option is set
                        std::optional<typename marginal_t::Covariance_t> optional_subcovariance;
                        if (options.compute_covariance)
                        {
                          if (optional_covariance_ptr->has_value())
                          {
                            optional_subcovariance
                                = optional_covariance_ptr->value().block<kN, kN>(sysidx, sysidx);
                          }
                        }
                        else
                          optional_subcovariance = std::nullopt;

                        // writes the new mean (or increment in NL systems) and the new covariance
                        // in the marginal
                        if constexpr (isSystFullyLinear)
                        {
                          // replace the mean by the maximum a posterior subvector, and save
                          // previous marginal in history std::cout <<
                          // optional_subcovariance.value() << '\n'; std::cout <<
                          // ::sam::Marginal::stringify_marginal_blockliner(marginal_t(MaP_subvector,optional_subcovariance));
                          wrapped_marginal.save_and_replace(
                              marginal_t(MaP_subvector, optional_subcovariance));
                          // std::cout << "post save_and_replace \n" <<
                          // ::sam::Marginal::stringify_marginal_blockliner(wrapped_marginal.marginal);
                        }
                        else
                        {
                          // the Max a Posteriori is in the tangent space (R^kN technically, hat
                          // operator must be used to be in the tangent space formally)
                          wrapped_marginal.save_and_add(tangent_space_t(MaP_subvector),
                                                        optional_subcovariance);
                        }
                      }
                    };
                    (lambda_update_map_of_wmarginals(vect_of_wmarginals, N_type_start_idx), ...);
                  },
                  natural_scalar_N_offsets);
            }
            ,
            this->all_vectors_marginals_.vectors_of_marginals);

        //------------------------------------------------------------------//
        //                   Post Solver loop on factors                    //
        //------------------------------------------------------------------//
        // std::atomic<double> accumulated_syst_squared_norm  (0);
        double accumulated_syst_squared_norm(0);
        std::apply(
            [&accumulated_syst_squared_norm](auto&... vec_of_wfactors)
            {
              std::string title = "loop factor and update data";
              PROFILE_SCOPE(title.c_str());
              // on M3500, sequential policy is ~3.5 times faster (0.37 ms vs 1.25ms)
              // probably because of the lock !
              (std::for_each(   //  std::execution::par_unseq,  // linker failure if tbb not found
                                //  at cmake level
                   vec_of_wfactors.begin(),
                   vec_of_wfactors.end(),
                   [&accumulated_syst_squared_norm](auto& wfactor)
                   {
                     // push the former norm
                     wfactor.norm_history.push_back(wfactor.get_current_point_data().norm);
                     // enforce new linearisation point on data (Ai)
                     auto new_data_at_lin_point = wfactor.compute_persistent_data();
                     wfactor.set_persistent_data(new_data_at_lin_point);
                     accumulated_syst_squared_norm
                         += new_data_at_lin_point.norm;   // fetch_add for atomic
                   }),
               ...);
            },
            this->all_factors_tuple_);

        // NOTE: save accumulated squared norm in OptimStats

        nIter++;
      }

      // update sequence number
      this->header.nbSequence++;

      return optim_stats;
    }

  }; // end DenseSystem











  //------------------------------------------------------------------//
  //                          Sparse System                           //
  //------------------------------------------------------------------//
  template <typename SOLVER_T,
            typename FACTOR_T,
            typename... FACTORS_Ts>   // I need at least one type of factor
  class SparseSystem : public MatrixBaseSystem<SparseSystem<SOLVER_T,FACTOR_T,FACTORS_Ts...>
                                        ,SOLVER_T
                                        ,FACTOR_T
                                        ,FACTORS_Ts...>
  {
    public:
      using Matrix_Base_System_t = typename MatrixBaseSystem<SparseSystem<SOLVER_T,FACTOR_T,FACTORS_Ts...>
                                        ,SOLVER_T
                                        ,FACTOR_T
                                        ,FACTORS_Ts...>::type;
      using type = SparseSystem<SOLVER_T, FACTOR_T, FACTORS_Ts...>;
      using KeyMetae_t = typename Matrix_Base_System_t::KeyMetae_t;
      using Vectors_Marginals_t = typename Matrix_Base_System_t::Vectors_Marginals_t;
      using Map_Marginals_t = typename Matrix_Base_System_t::Map_Marginals_t;
      using Wrapped_Factors_t = typename Matrix_Base_System_t::Wrapped_Factors_t;
      using SystemHeader = typename Matrix_Base_System_t::SystemHeader;
      using OptimOptions = typename Matrix_Base_System_t::OptimOptions;
      using OptimStats   = typename Matrix_Base_System_t::OptimStats;
      using SolverStats  = typename Matrix_Base_System_t::SolverStats;

      static constexpr const bool isSystFullyLinear = Matrix_Base_System_t::isSystFullyLinear;
      static constexpr std::size_t kNbFactorTypes   = Matrix_Base_System_t::kNbFactorTypes;
      static constexpr std::size_t kNbKeyTypes      = Matrix_Base_System_t::kNbKeyTypes;
    /**
     * @brief constructor
     *
     * @param agent id
     */
    SparseSystem(const std::string& agent_id, const std::string& system_label = "sparse matrix inference system")
        : Matrix_Base_System_t(agent_id,system_label)
    {
    }

    /**
     * @brief optimisation method
     */
    OptimStats sam_optimise_matrix_specialized(const OptimOptions& options = OptimOptions())
    {
      // scoped timer
      PROFILE_FUNCTION();

      size_t M = SystemConverter::Scalar::M(this->all_factors_tuple_);
      size_t N = SystemConverter::Scalar::N(this->all_vectors_marginals_.vectors_of_marginals);
      size_t semantic_M = SystemConverter::Semantic::M(this->all_factors_tuple_);
      size_t semantic_N
          = SystemConverter::Semantic::N(this->all_vectors_marginals_.vectors_of_marginals);
      size_t nnz_jacobian = MatrixConverter::Scalar::JacobianNNZ(this->all_factors_tuple_);
      size_t nnz_semantic_jacobian
          = MatrixConverter::Semantic::JacobianNNZ(this->all_factors_tuple_);

      OptimStats optim_stats;
      if (M == 0) return optim_stats;   // set success bool to false

      auto natural_scalar_M_offsets
          = SystemConverter::Scalar::FactorTypeIndexesOffset(this->all_factors_tuple_);
      auto natural_scalar_N_offsets = SystemConverter::Scalar::MarginalTypeIndexesOffset(
          this->all_vectors_marginals_.vectors_of_marginals);
      auto natural_semantic_M_offsets
          = SystemConverter::Semantic::FactorTypeIndexesOffset(this->all_factors_tuple_);
      // auto semantic_N_type_idx_offsets =
      // MatrixConverter::Semantic::MarginalTypeIndexesOffset(this->all_vectors_marginals_.vectors_of_marginals);

      if (this->keys_affectation_unsync_)
      {
          this->keys_affectation = SystemConverter::compute_keys_affectation(
              this->all_factors_tuple_,
              this->all_vectors_marginals_.vectors_of_marginals);
          this->visitor_compute_keys_affectation();
          this->keys_affectation_unsync_ = false;
      }

      Eigen::SparseMatrix<int> semantic_A
          = MatrixConverter::Sparse::Semantic::spyJacobian(this->all_factors_tuple_,
                                                           this->keys_affectation,
                                                           semantic_M,
                                                           semantic_N,
                                                           nnz_semantic_jacobian,
                                                           natural_semantic_M_offsets);

      Eigen::SparseMatrix<int> semantic_H = MatrixConverter::Sparse::Semantic::spyHessian(this->keys_affectation,nnz_semantic_jacobian);


      //------------------------------------------------------------------//
      //                            LOOP START                            //
      //------------------------------------------------------------------//
      // loop of the iterations
      int max_iterations;
      if constexpr (isSystFullyLinear)
        max_iterations = 1;
      else
        max_iterations = options.max_iterations;

      int nIter = 0;
      while (nIter < max_iterations)
      {
        // scoped timer
        std::string timer_name = "iter" + std::to_string(nIter);
        PROFILE_SCOPE(timer_name.c_str());

        auto [b, A] = MatrixConverter::Sparse::compute_b_A(
            this->all_factors_tuple_,
            this->all_vectors_marginals_.vectors_of_marginals,
            this->keys_affectation,
            M,
            N,
            nnz_jacobian,
            natural_scalar_M_offsets);  // WARNING: system split

        // maximum a posteriori, may represent a \hat X or \delta \hat X (NL)
        Eigen::VectorXd                                 MaP;
        typename SOLVER_T::Stats_t                      solver_stats;
        std::shared_ptr<std::optional<Eigen::MatrixXd>> optional_covariance_ptr;
        std::tie(MaP, optional_covariance_ptr, solver_stats)
            = SOLVER_T::solve(A, b, options.solver);

#if ENABLE_DEBUG_TRACE
        {
          PROFILE_SCOPE("print console");
          std::cout << "#### Iteration : " << nIter << '\n';
          std::cout << "#### Syst: A(" << A.rows() << "," << A.cols() << ") computed :\n";
          // only display if matrix not too big
          if (A.rows() < 22 && nIter == 0) std::cout << Eigen::MatrixXd(A) << "\n\n";
          // std::cout << "#### Syst: R computed :\n" << Eigen::MatrixXd(A) <<
          // "\n\n";
          std::cout << "#### Syst: b computed :\n";
          if (b.rows() < 22 && nIter == 0) std::cout << b << "\n";
          std::cout << "#### Syst: MAP computed :\n" << MaP << '\n';
          // std::cout << "#### Syst: Covariance Sigma("<< SigmaCovariance.rows() <<","<<
          // SigmaCovariance.cols() <<") computed : \n" ; if (SigmaCovariance.rows()<15) std::cout
          // << SigmaCovariance << '\n';
        }
#endif

        //------------------------------------------------------------------//
        //                  POST SOLVER LOOP ON MARGINALS                   //
        //------------------------------------------------------------------//
        std::apply(
            [this, &MaP, &options, optional_covariance_ptr, &nIter, &natural_scalar_N_offsets](
                auto&... vect_of_wmarginals)
            {
              std::apply(
                  [&, this](const auto&... N_type_start_idx)
                  {
                    std::string scope_name = "save marginal updates";
                    PROFILE_SCOPE(scope_name.c_str());
                    // define the function
                    auto lambda_update_map_of_wmarginals
                        = [&, this](auto& vector_of_wrapped_marginals, std::size_t KeyTypeStartIdx)
                    {
                      using wrapped_marginal_t = typename std::remove_cvref_t<
                          decltype(vector_of_wrapped_marginals)>::value_type;
                      using marginal_t         = typename wrapped_marginal_t::Marginal_t;
                      using tangent_space_t    = typename marginal_t::Tangent_Space_t;
                      using keymeta_t          = typename marginal_t::KeyMeta_t;
                      constexpr std::size_t kN = marginal_t::KeyMeta_t::kN;
                      // looping over the marginal collection and updating them with the MAP result
                      // std::for_each (
                      //     // std::execution::par   // on M3500, sequential is still slightly
                      //     faster than par_unseq (1.4 ms vs 1.625 ms) ,
                      //     map_of_wrapped_marginals.begin() , map_of_wrapped_marginals.end() ,
                      //     [&,this]( auto & kvpair)
                      std::size_t idx_marg = 0;
                      for (auto it_marg = vector_of_wrapped_marginals.begin();
                           it_marg != vector_of_wrapped_marginals.end();
                           it_marg++)
                      {
                        // std::string key_id = it_marg->first;
                        std::string         key_id           = it_marg->key_id;
                        wrapped_marginal_t& wrapped_marginal = *it_marg;
                        // get the subvector from the Maximum A Posteriori vector
                        auto sysidx = KeyTypeStartIdx + idx_marg * kN;
                        idx_marg++;
                        auto MaP_subvector = MaP.block<kN, 1>(sysidx, 0);

                        // clear previous history at first iteration
                        if (nIter == 0) wrapped_marginal.clear_history();

                        // covariance, if option is set
                        std::optional<typename marginal_t::Covariance_t> optional_subcovariance;
                        if (options.compute_covariance)
                        {
                          if (optional_covariance_ptr->has_value())
                          {
                            optional_subcovariance
                                = optional_covariance_ptr->value().block<kN, kN>(sysidx, sysidx);
                          }
                        }
                        else
                          optional_subcovariance = std::nullopt;

                        // writes the new mean (or increment in NL systems) and the new covariance
                        // in the marginal
                        if constexpr (isSystFullyLinear)
                        {
                          // replace the mean by the maximum a posterior subvector, and save
                          // previous marginal in history std::cout <<
                          // optional_subcovariance.value() << '\n'; std::cout <<
                          // ::sam::Marginal::stringify_marginal_blockliner(marginal_t(MaP_subvector,optional_subcovariance));
                          wrapped_marginal.save_and_replace(
                              marginal_t(MaP_subvector, optional_subcovariance));
                          // std::cout << "post save_and_replace \n" <<
                          // ::sam::Marginal::stringify_marginal_blockliner(wrapped_marginal.marginal);
                        }
                        else
                        {
                          // the Max a Posteriori is in the tangent space (R^kN technically, hat
                          // operator must be used to be in the tangent space formally)
                          wrapped_marginal.save_and_add(tangent_space_t(MaP_subvector),
                                                        optional_subcovariance);
                        }
                      }
                      // ); // for each
                    };
                    (lambda_update_map_of_wmarginals(vect_of_wmarginals, N_type_start_idx), ...);
                  },
                  natural_scalar_N_offsets);
            }
            // , this->all_marginals_.data_map_tuple);
            ,
            this->all_vectors_marginals_.vectors_of_marginals);

        //------------------------------------------------------------------//
        //                   Post Solver loop on factors                    //
        //------------------------------------------------------------------//
        // std::atomic<double> accumulated_syst_squared_norm  (0);
        double accumulated_syst_squared_norm(0);
        std::apply(
            [&accumulated_syst_squared_norm](auto&... vec_of_wfactors)
            {
              std::string title = "loop factor and update data";
              PROFILE_SCOPE(title.c_str());
              // on M3500, sequential policy is ~3.5 times faster (0.37 ms vs 1.25ms)
              // probably because of the lock !
              (std::for_each(   //  std::execution::par_unseq,  // linker failure if tbb not found
                                //  at cmake level
                   vec_of_wfactors.begin(),
                   vec_of_wfactors.end(),
                   [&accumulated_syst_squared_norm](auto& wfactor)
                   {
                     // push the former norm
                     wfactor.norm_history.push_back(wfactor.get_current_point_data().norm);
                     // enforce new linearisation point on data (Ai)
                     auto new_data_at_lin_point = wfactor.compute_persistent_data();
                     wfactor.set_persistent_data(new_data_at_lin_point);
                     accumulated_syst_squared_norm
                         += new_data_at_lin_point.norm;   // fetch_add for atomic
                   }),
               ...);
            },
            this->all_factors_tuple_);

        // NOTE: save accumulated squared norm in OptimStats

        nIter++;
      }

      // update sequence number
      this->header.nbSequence++;

      return optim_stats;
    }

  }; // end SparseSystem

  //------------------------------------------------------------------//
  //                           Graph System                           //
  //------------------------------------------------------------------//
  template <typename OPTIM_STATS_T, // WARNING: arguments subject to breaking change
            typename OPTIM_OPTIONS_T,
            typename FACTOR_T,
            typename... FACTORS_Ts>   // I need at least one type of factor
  class GraphSystem : public BaseSystem<GraphSystem<OPTIM_STATS_T,OPTIM_OPTIONS_T,FACTOR_T,FACTORS_Ts...>
                                        ,OPTIM_STATS_T
                                        ,OPTIM_OPTIONS_T
                                        ,FACTOR_T
                                        ,FACTORS_Ts...>
  {
    
    public:
      using Base_System_t = typename BaseSystem<GraphSystem<OPTIM_STATS_T,OPTIM_OPTIONS_T,FACTOR_T,FACTORS_Ts...>
                                        ,OPTIM_STATS_T
                                        ,OPTIM_OPTIONS_T
                                        ,FACTOR_T
                                        ,FACTORS_Ts...>::type;
      using type = GraphSystem<OPTIM_STATS_T,OPTIM_OPTIONS_T, FACTOR_T, FACTORS_Ts...>;
      using KeyMetae_t = typename Base_System_t::KeyMetae_t;
      using Vectors_Marginals_t = typename Base_System_t::Vectors_Marginals_t;
      using Map_Marginals_t = typename Base_System_t::Map_Marginals_t;
      using Wrapped_Factors_t = typename Base_System_t::Wrapped_Factors_t;
      using SystemHeader = typename Base_System_t::SystemHeader;
      using OptimOptions = OPTIM_OPTIONS_T;
      using OptimStats   = OPTIM_STATS_T;
      // using SolverStats  = typename Base_System_t::SolverStats;

      static constexpr const bool isSystFullyLinear = Base_System_t::isSystFullyLinear;
      static constexpr std::size_t kNbFactorTypes   = Base_System_t::kNbFactorTypes;
      static constexpr std::size_t kNbKeyTypes      = Base_System_t::kNbKeyTypes;
    /**
     * @brief constructor
     *
     * @param agent id
     */
    GraphSystem(const std::string& agent_id, const std::string& system_label = "inference system")
        : Base_System_t(agent_id,system_label)
    {
    }

    // MRF
    using UndirectedGraph_t = GraphConverter::UndirectedGraph_t;

    UndirectedGraph_t MRF;

    void visitor_compute_keys_affectation() override
    {
      this->MRF = GraphConverter::build_undirected_graph(this->all_factors_tuple_, this->all_vectors_marginals_, this->keys_affectation);
    }

    /**
     * @brief optimisation method
     */
    OptimStats sam_optimise_specialized(const OptimOptions& options = OptimOptions())
    {
      // scoped timer
      PROFILE_FUNCTION();

      size_t M = SystemConverter::Scalar::M(this->all_factors_tuple_);
      size_t N = SystemConverter::Scalar::N(this->all_vectors_marginals_.vectors_of_marginals);
      size_t semantic_M = SystemConverter::Semantic::M(this->all_factors_tuple_);
      size_t semantic_N
          = SystemConverter::Semantic::N(this->all_vectors_marginals_.vectors_of_marginals);
      size_t nnz_jacobian = MatrixConverter::Scalar::JacobianNNZ(this->all_factors_tuple_);
      size_t nnz_semantic_jacobian
          = MatrixConverter::Semantic::JacobianNNZ(this->all_factors_tuple_);

      OptimStats optim_stats;
      if (M == 0) return optim_stats;   // set success bool to false

      auto natural_scalar_M_offsets
          = SystemConverter::Scalar::FactorTypeIndexesOffset(this->all_factors_tuple_);
      auto natural_scalar_N_offsets = SystemConverter::Scalar::MarginalTypeIndexesOffset(
          this->all_vectors_marginals_.vectors_of_marginals);
      auto natural_semantic_M_offsets
          = SystemConverter::Semantic::FactorTypeIndexesOffset(this->all_factors_tuple_);
      // auto semantic_N_type_idx_offsets =
      // MatrixConverter::Semantic::MarginalTypeIndexesOffset(this->all_vectors_marginals_.vectors_of_marginals);

      if (this->keys_affectation_unsync_)
      {
          this->keys_affectation = SystemConverter::compute_keys_affectation(
              this->all_factors_tuple_,
              this->all_vectors_marginals_.vectors_of_marginals);
          this->visitor_compute_keys_affectation();
          this->keys_affectation_unsync_ = false;
      }

      Eigen::SparseMatrix<int> semantic_A
          = MatrixConverter::Sparse::Semantic::spyJacobian(this->all_factors_tuple_,
                                                           this->keys_affectation,
                                                           semantic_M,
                                                           semantic_N,
                                                           nnz_semantic_jacobian,
                                                           natural_semantic_M_offsets);

      Eigen::SparseMatrix<int> semantic_H = MatrixConverter::Sparse::Semantic::spyHessian(this->keys_affectation);

      // 1. AMD ( semantic_H )
      // 2. fillin_edges vector
      // 3. (from 1 to 2 in parallel)  boost graph G = MRF of factors (or use boost-graphetize  keys_affects ?? )
      // 4. (once 2&3) G' = G_cpy + add fill edges
      // 5. {C,S_c} = MCS (G') 
      // 6. c_r = getRoot(S_c)
      // 7. fwd message passing

      // 1. AMD
      std::vector<int> PermutationVector = HybridConverter::amd_order_permutation(semantic_N,semantic_H.outerIndexPtr(), semantic_H.innerIndexPtr());
      // PermutationVector.reserve(semantic_N);
      // {
      //   PROFILE_SCOPE("amd ordering");
      //   amd_order(semantic_N, semantic_H.outerIndexPtr(), semantic_H.innerIndexPtr(), &PermutationVector[0], (double*)NULL,(double*)NULL);
      // }
      // for (int k = 0; k < semantic_N; ++k) printf("P [%d] = %d\n", k, PermutationVector[k]);

      // 2. fillin_edges vector
      std::vector<std::pair<std::string, std::string>> fillin_edges 
        = HybridConverter::infer_fillinedges(PermutationVector, this->keys_affectation);
      // std::cout << fillin_edges.size() <<" fill in edges (Hybrid method) !\n";
      // for (auto & [e1,e2] : fillin_edges)
      // {
      //   std::cout << "\t [ " << e1 << " <-> " << e2 << " ]\n";
      // }

      // 2bis. fillin_edges in boost graph
      // performance: (m3500) hybrid method takes 37 ms (slow) while
      //              - graph<hash_setS,vecS,[...],listS> takes 5.647 ms
      //              - graph<vecS,vecS,[...], listS> takes 3.250 ms
      //              - graph<vecS,vecS,[...], vecS> takes 3.235 ms 
      //              - graph<vecS,hash_setS,[...], listS> takes
      auto cover_graph = GraphConverter::infer_fillinedges(PermutationVector, this->MRF);
      // std::cout << fillin_edges_bis.size() <<" fill in edges (graph method) !\n";
      // for (auto & [e1,e2] : fillin_edges_bis)
      // {
      //   std::cout << "\t [ " << e1 << " <-> " << e2 << " ]\n";
      // }
      
      // 3. MCS

      //------------------------------------------------------------------//
      //                            LOOP START                            //
      //------------------------------------------------------------------//
      // loop of the iterations
      int max_iterations;
      if constexpr (isSystFullyLinear)
        max_iterations = 1;
      else
        max_iterations = options.max_iterations;

      int nIter = 0;
      while (nIter < max_iterations)
      {
        // TODO:
        nIter++;
      }

      // update sequence number
      this->header.nbSequence++;

      return optim_stats;
    }

  };
  


};   // namespace sam::Inference
