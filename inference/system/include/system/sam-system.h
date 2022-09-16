#pragma once

#include "MatrixConverter.hpp"
#include "marginal/marginal.h"
#include "utils/config.h"
#include "utils/tuple_patterns.h"
#include "utils/utils.h"
#include "system/SystemConverter.hpp"
#include "system/MatrixConverter.hpp"
#include "solver/solver.h"

#include <functional>
#include <execution>
#include <numeric>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include <iomanip>

#include "system/system_jsonify.h"
#include "system/PersistentFactor.h"

namespace sam::Inference
{

  struct SystemHeader
  {
    std::string agent_id;
    std::string system_label;
    // std::string solver_name;
    std::size_t nbSequence = 0;
    
    SystemHeader(const std::string & agent_id, const std::string & system_label)
      : agent_id(agent_id), system_label(system_label)
    {}
  };

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
    // cache the full joint covariance (mind the memory impact, but allow for rapid joint covariance queries)
    bool cache_covariance = this->compute_covariance && false; // cache_covariance (moot if no computation of covariance)
    // TODO: thresholding strategy (by key type?? -> leads to template)
    // solver options
    typename SOLVER_T::Options_t solver;
    // ctor
    OptimOptions()
      : solver(this->compute_covariance)
    {}
      
    // ctor
    OptimOptions(bool compute_covariance)
      :
      compute_covariance(compute_covariance)
      ,solver(this->compute_covariance)
    {}
  };


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
    //ratio of nonzeros in the jacobian matrix
    std::size_t ratioNZ_jacobian_scalar;
    // number of scalar nonzeros in the hessian matrix
    std::size_t nbNZ_hessian_scalar;
    //ratio of nonzeros in the hessian matrix
    std::size_t ratioNZ_hessian_scalar;
    // size M,N of the jacobian matrix
    std::size_t M,N;
    // size of the semantic jacobian matrix
    std::size_t M_semantic,N_semantic;

    // report message
    std::string report_str;
    // negative log likelihood of the product of factors
    double NLog_value_before;
    double NLog_value_after;
  };

  template <typename SOLVER_T,
            typename FACTOR_T,
            typename... FACTORS_Ts>   // I need at least one type of factor
  class System
  {
    public:
    // keymetae (latin plural <3) remove duplicates
    using KeyMetae_t = typename sam_tuples::tuple_filter_duplicate
                                    <
                                      sam_tuples::tuple_cat_t
                                      <typename FACTOR_T::KeyMetas_t, typename FACTORS_Ts::KeyMetas_t ... >
                                    >::type;

    // declare marginal container type of those keymetas
    using MapMarginals_t = typename ::sam::Marginal::MarginalsCollection<KeyMetae_t>::type ;

    using type = System<SOLVER_T,FACTOR_T,FACTORS_Ts...>;

    using SystemHeader = SystemHeader;
    using OptimOptions = OptimOptions<SOLVER_T>;
    using OptimStats = OptimStats<SOLVER_T>;
    using SolverStats = typename SOLVER_T::Stats_t;

    static constexpr const bool isSystFullyLinear = FACTOR_T::isLinear && ( FACTORS_Ts::isLinear && ... );

    using Wrapped_Factor_t = 
      std::tuple<
        std::vector<::sam::Factor::WrapperPersistentFactor<FACTOR_T,isSystFullyLinear>>
        , std::vector<::sam::Factor::WrapperPersistentFactor<FACTORS_Ts,isSystFullyLinear>>
        ...
        >;

    SystemHeader header;

    std::unordered_map<std::string,typename SystemConverter::KeyDispatchInfos> keys_affectation = {};

    static constexpr std::size_t kNbFactorTypes = 1 + sizeof...(FACTORS_Ts);
    static constexpr std::size_t kNbKeyTypes = std::tuple_size_v<KeyMetae_t>;

      /**
      * @brief constructor
      *
      * @param agent id
      */
    System(const std::string & agent_id, const std::string & system_label = "inference system")
      :
      header(agent_id, system_label)
    { 
    }

    /**
    * @brief optimisation method
    */
    OptimStats sam_optimise(const OptimOptions & options = OptimOptions())
    {
      // scoped timer
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      size_t M = SystemConverter::Scalar::M(this->all_factors_tuple_);
      size_t N = SystemConverter::Scalar::N(this->all_vectors_marginals_.vectors_of_marginals);
      size_t semantic_M = SystemConverter::Semantic::M(this->all_factors_tuple_);
      size_t semantic_N = SystemConverter::Semantic::N(this->all_vectors_marginals_.vectors_of_marginals);
      size_t nnz_jacobian = MatrixConverter::Scalar::JacobianNNZ(this->all_factors_tuple_);
      // size_t nnz_hessian = MatrixConverter::Scalar::HessianNNZ(this->all_factors_tuple_);        // a bit long
      size_t nnz_semantic_jacobian = MatrixConverter::Semantic::JacobianNNZ(this->all_factors_tuple_);
      // size_t nnz_semantic_hessian  = MatrixConverter::Semantic::HessianNNZ(this->all_factors_tuple_); // a bit long

      
      auto natural_scalar_M_offsets = SystemConverter::Scalar::FactorTypeIndexesOffset(this->all_factors_tuple_);
      auto natural_scalar_N_offsets = SystemConverter::Scalar::MarginalTypeIndexesOffset(this->all_vectors_marginals_.vectors_of_marginals);
      auto natural_semantic_M_offsets = SystemConverter::Semantic::FactorTypeIndexesOffset(this->all_factors_tuple_);
      // auto semantic_N_type_idx_offsets = MatrixConverter::Semantic::MarginalTypeIndexesOffset(this->all_vectors_marginals_.vectors_of_marginals);

      Eigen::SparseMatrix<int> semantic_A = MatrixConverter::Sparse::Semantic::spyJacobian(
                                                this->all_factors_tuple_ 
                                              , this->keys_affectation
                                              , semantic_M
                                              , semantic_N
                                              , nnz_semantic_jacobian
                                              , natural_semantic_M_offsets);

      OptimStats optim_stats;

      // NOTE: SolverStats might have ratio rnnz/N*N

      if (M == 0) return optim_stats; // set success bool to false

#if ENABLE_DEBUG_TRACE
        std::cout << "### Syst: Starting an optimisation \n";
        std::cout << "### Syst: size " << M << " * " << N << '\n';
        std::cout << "### Semantic A: \n" ;
#endif

      //------------------------------------------------------------------//
      //                            LOOP START                            //
      //------------------------------------------------------------------//
      // loop of the iterations
      int max_iterations;
      if constexpr ( isSystFullyLinear )  max_iterations = 1;
      else max_iterations =  options.max_iterations;

      int nIter = 0;
      while(nIter < max_iterations)
      {
        // scoped timer
        std::string timer_name = "iter" + std::to_string(nIter);
        PROFILE_SCOPE(timer_name.c_str(),sam_utils::JSONLogger::Instance());

        auto [b,A] = MatrixConverter::Sparse::compute_b_A(
                    this->all_factors_tuple_
                    , this->all_vectors_marginals_.vectors_of_marginals
                    , this->keys_affectation
                    , M,N,nnz_jacobian,natural_scalar_M_offsets);

       // maximum a posteriori, may represent a \hat X or \delta \hat X (NL)
        Eigen::VectorXd MaP;
        typename SOLVER_T::Stats_t solver_stats;
        std::shared_ptr<std::optional<Eigen::MatrixXd>> optional_covariance_ptr;
        std::tie(MaP,optional_covariance_ptr,solver_stats) = SOLVER_T::solve(A,b,options.solver); 

#if ENABLE_DEBUG_TRACE
      {
        PROFILE_SCOPE("print console",sam_utils::JSONLogger::Instance());
        std::cout << "#### Iteration : " << nIter << '\n';
        std::cout << "#### Syst: A("<< A.rows() <<","<< A.cols() <<") computed :\n";
        // only display if matrix not too big
        if ( A.rows() < 22 && nIter ==0) std::cout << Eigen::MatrixXd(A) << "\n\n";
        // std::cout << "#### Syst: R computed :\n" << Eigen::MatrixXd(A) <<
        // "\n\n";
        std::cout << "#### Syst: b computed :\n";
        if ( b.rows() < 22 && nIter ==0) std::cout << b << "\n";
        std::cout << "#### Syst: MAP computed :\n" << MaP << '\n';
        // std::cout << "#### Syst: Covariance Sigma("<< SigmaCovariance.rows() <<","<< SigmaCovariance.cols() <<") computed : \n" ;
        // if (SigmaCovariance.rows()<15) std::cout << SigmaCovariance << '\n';
      }
#endif

        //------------------------------------------------------------------//
        //                  POST SOLVER LOOP ON MARGINALS                   //
        //------------------------------------------------------------------//
        std::apply(
            [this, &MaP,&options,optional_covariance_ptr,&nIter,&natural_scalar_N_offsets](auto & ... vect_of_wmarginals)
            {
              std::apply(
                  [&,this](const auto & ... N_type_start_idx)
                  {
                      std::string scope_name = "save marginal updates";
                      PROFILE_SCOPE( scope_name.c_str() ,sam_utils::JSONLogger::Instance());
                      // define the function
                      auto lambda_update_map_of_wmarginals = [&,this](auto & vector_of_wrapped_marginals, std::size_t KeyTypeStartIdx)
                      {
                          using wrapped_marginal_t = typename std::remove_cvref_t<decltype(vector_of_wrapped_marginals)>::value_type;
                          using marginal_t = typename wrapped_marginal_t::Marginal_t;
                          using tangent_space_t = typename marginal_t::Tangent_Space_t;
                          using keymeta_t = typename marginal_t::KeyMeta_t;
                          constexpr std::size_t kN = marginal_t::KeyMeta_t::kN;
                          // looping over the marginal collection and updating them with the MAP result
                          // std::for_each (
                          //     // std::execution::par   // on M3500, sequential is still slightly faster than par_unseq (1.4 ms vs 1.625 ms)
                          //     , map_of_wrapped_marginals.begin()
                          //     , map_of_wrapped_marginals.end() 
                          //     , [&,this]( auto & kvpair)
                          std::size_t idx_marg = 0;
                          for(auto it_marg =vector_of_wrapped_marginals.begin(); it_marg!=vector_of_wrapped_marginals.end(); it_marg++) // WARNING: marginal refactor: vector will speed up a bit (not much though)
                          {
                            // std::string key_id = it_marg->first;
                            std::string key_id = it_marg->key_id;
                            wrapped_marginal_t & wrapped_marginal = *it_marg;
                            // get the subvector from the Maximum A Posteriori vector
                            auto sysidx = KeyTypeStartIdx + idx_marg * kN ;
                            idx_marg++;
                            auto MaP_subvector = MaP.block<kN,1>(sysidx, 0);

                            // clear previous history at first iteration
                            if (nIter ==0)  wrapped_marginal.clear_history();
                           
                            // covariance, if option is set
                            std::optional<typename marginal_t::Covariance_t>  optional_subcovariance;
                            if (options.compute_covariance)
                            {
                              if (optional_covariance_ptr->has_value())
                              {
                                optional_subcovariance = optional_covariance_ptr->value().block<kN,kN>(sysidx,sysidx);
                              }
                            }
                            else optional_subcovariance = std::nullopt;

                            // writes the new mean (or increment in NL systems) and the new covariance in the marginal
                            if constexpr (isSystFullyLinear) 
                            {
                              // replace the mean by the maximum a posterior subvector, and save previous marginal in history
                              // std::cout << optional_subcovariance.value() << '\n';
                              // std::cout << ::sam::Marginal::stringify_marginal_blockliner(marginal_t(MaP_subvector,optional_subcovariance));
                              wrapped_marginal.save_and_replace( 
                                  marginal_t(MaP_subvector, optional_subcovariance) 
                                  );
                              // std::cout << "post save_and_replace \n" << ::sam::Marginal::stringify_marginal_blockliner(wrapped_marginal.marginal);
                            }
                            else
                            {
                              // the Max a Posteriori is in the tangent space (R^kN technically, hat operator must be used
                              // to be in the tangent space formally)
                              wrapped_marginal.save_and_add( tangent_space_t(MaP_subvector), optional_subcovariance );
                            }
                          }
                          // ); // for each
                      };
                    ( lambda_update_map_of_wmarginals(vect_of_wmarginals, N_type_start_idx), ...);
                  }
                  , natural_scalar_N_offsets);
            }
            // , this->all_marginals_.data_map_tuple);
            , this->all_vectors_marginals_.vectors_of_marginals);

        //------------------------------------------------------------------//
        //                   Post Solver loop on factors                    //
        //------------------------------------------------------------------//
        // std::atomic<double> accumulated_syst_squared_norm  (0);
        double accumulated_syst_squared_norm  (0);
        std::apply(
            [&accumulated_syst_squared_norm](auto & ...vec_of_wfactors)
            {
              std::string title = "loop factor and update data";
              PROFILE_SCOPE(title.c_str(), sam_utils::JSONLogger::Instance());
              // on M3500, sequential policy is ~3.5 times faster (0.37 ms vs 1.25ms)
              // probably because of the lock !
              (
               std::for_each(//  std::execution::par_unseq,  // linker failure if tbb not found at cmake level
                 vec_of_wfactors.begin(),vec_of_wfactors.end(),
                  [&accumulated_syst_squared_norm](auto & wfactor)
                  {
                    // push the former norm
                    wfactor.norm_history.push_back(wfactor.get_current_point_data().norm);
                    // enforce new linearisation point on data (Ai)
                    auto new_data_at_lin_point = wfactor.compute_persistent_data();
                    wfactor.set_persistent_data(new_data_at_lin_point);
                    accumulated_syst_squared_norm += new_data_at_lin_point.norm; // fetch_add for atomic
                  })
               ,...
              );

            }
            , this->all_factors_tuple_
        );

        // NOTE: save accumulated squared norm in OptimStats

        nIter++;
      }

      // update sequence number
      this->header.nbSequence++;

      return optim_stats;
    }

    void remove_factor(const std::string & factor_id)
    {
      // nothing happend if factor_id doesnt exist
      std::apply(
          [&](auto& ... vec_of_wfactor)
          {
            // erase-remove idiom wrapped in expression expansion 
          ( 
           // TODO: assert that number of eliminated element is 1 at most (returned value of erase_if)
           std::erase_if(
             vec_of_wfactor, 
             [&](auto & wf){return wf.factor.factor_id == factor_id;}
             )
            , ...);
          },this->all_factors_tuple_);
      // TODO: remove smthing in GraphModel ?

      // TODO: the valid of any cache data should be questioned after removing a factor
    }

    void remove_key(const std::string & key_id)
    {
      // remove associated factors?
      // several mines here: what happen if graph becomes unconnected etc...
      // TODO: the valid of any cache data should be questioned after removing a factor
    }

    void sum_out(const std::string & key_id)
    {
      // not the same as remove_key !
      // -> a marginalisation should occur first
      
      // TODO: the valid of any cache data should be questioned after removing a factor
    }

    auto get_all_factors() const
    {
      return this->all_factors_tuple_;
    }

    auto get_keys_affectation() const
    {
      return this->keys_affectation;
    }
    
    // TODO: urgent get_joint-marginal etc... 
    // joint_marginal<marginals_t> get_joint_marginal()
    // get_key_marginal() // WARNING: marginal refactor: linear cost
    // get_full_joint(){}
    // get_full_joint_except( ... )

    // get all marginals
    auto get_marginals() const
    {
      // WARNING: after marginal refactor: return the vector of marginals. If you want the map of marginals, use get_marginals_as_map()
      // return all_marginals_.data_map_tuple;
      return this->all_vectors_marginals_.vectors_of_marginals;
    }

    
    MapMarginals_t get_marginals_as_map() const
    {
      // motivation: don't assume users utilisation of the key marginals collection
      //             e.g. for json it is better to have a vector
      //                  for most other purposes better to have a map
      MapMarginals_t map_of_wmarginals;
      std::apply([&](const auto & ...vwm)
          {
            (
              (std::for_each(
                vwm.begin(),vwm.end(),
                [&](const auto & wm)
                {
                     map_of_wmarginals
                    .template 
                    insert_in_marginal_container
                    <typename std::remove_cvref_t<decltype(vwm)>::value_type>
                    (wm);
                }
                )
             ),...);
          }
          ,this->all_vectors_marginals_.vectors_of_marginals);
      return map_of_wmarginals;
    }

    // Marginals_t all_marginals_;
    using Vectors_Marginals_t = Marginal::MarginalsVectorCollection<KeyMetae_t>;
    Vectors_Marginals_t all_vectors_marginals_;
    

    // there's at least one factor, the rest are expanded
    Wrapped_Factor_t all_factors_tuple_;

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
                             const typename FT::measure_t&          mes_vect,
                             const typename FT::measure_cov_t&           measure_cov,
                             const std::array<std::string, FT::kNbKeys>& keys_id)
    {
      static_assert(std::is_same_v<FT, FACTOR_T> || (std::is_same_v<FT, FACTORS_Ts> || ...),
                    "This type of factor doesnt exist ");
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      // TODO: consistent management failure (throw ? return value false ?

      // look up factor tuple if factor_id already stored
      bool factor_id_already_exists = std::apply([&factor_id](const auto & ...vof)
          {
            return 
            ( std::ranges::any_of(vof,[&factor_id](const auto & wf)
                                   {
                                      return wf.factor.factor_id == factor_id;
                                   }) || ... );
          },this->all_factors_tuple_);

      if (factor_id_already_exists)
        throw std::runtime_error("factor "+ factor_id+ " already exists");

      // apply to emplace this factor in the correct container
      std::apply(
          [&,this](auto & ... vect_of_wfactors)
          {
            (
             (
                this->emplace_factor_in<FT>(factor_id, mes_vect, measure_cov, keys_id,vect_of_wfactors)
              )
            , ...);
          }
          , this->all_factors_tuple_);
    }

    private:

    template <typename FT, typename VECT_OF_WFT>
    void emplace_factor_in(const std::string&                          factor_id,
                           const typename FT::measure_t&          mes_vect,
                           const typename FT::measure_cov_t&           measure_cov,
                           const std::array<std::string, FT::kNbKeys>& keys_id,
                           VECT_OF_WFT & vector_of_wrapped_factors)
    {
      using WFT = typename VECT_OF_WFT::value_type;
      // only run if compatible type
      if constexpr( std::is_same_v<FT,typename WFT::Factor_t> )
      {
          // look up our container to see if we have existing means for each key
          typename FT::KeysSet_t KccSet = FT::construct_keys_set(keys_id);
          typename FT::composite_of_opt_state_ptr_t tuple_of_opt_means_ptr
            = std::apply(
              [this](auto ... kcc) // copy, no big deal
              { 
                auto lambda = [this] (const auto & akcc)// -> std::optional<>
                {
                  using keymeta_t = typename std::remove_cvref_t<decltype(akcc)>::KeyMeta_t;
                  // // do the keys exist ?
                  // std::cout << SystemConverter::stringify_keys_affectation_blockliner(this->keys_affectation);
                  // std::cout << "size vectors marginal :" << SystemConverter::Semantic::N(this->all_vectors_marginals_.vectors_of_marginals) << '\n';
                  if (auto it {this->keys_affectation.find(akcc.key_id)}; it != this->keys_affectation.end())
                  {
                    // yes ! => get the mean !
                    return std::optional<std::shared_ptr<typename keymeta_t::key_t>>(this->all_vectors_marginals_.template find_if<keymeta_t>(akcc.key_id).shared_mean);
                  }
                  else 
                  { // no, return nullopt
                    return std::optional<std::shared_ptr<typename keymeta_t::key_t>>(std::nullopt);
                  }
                };
                return std::make_tuple( lambda(kcc)... );

                // return
                //   std::make_tuple
                //   ( 
                //       // TODO: marginal refactor: use keydispatch, then (if exists) vector.find_if to get the mean
                //      this->all_marginals_
                //        .template find_mean_ptr<typename decltype(kcc)::KeyMeta_t>(kcc.key_id) // WARNING: adverse cost when marginal refactor: perhaps transform vector of marginals in a std::map first (yes, in this method ! A slightly costlier emplace_factor method is not a big deal, this is not where bottleneck of the users API is)
                //       ... 
                //   );
                //   FIX: remove old, keep only competing 
              }, KccSet);
          // It is often the case that the above tuple contains at least 1 std::nullopt.

          // Attempt to guess the full init point for this factor by using the measurement if necessary.
          // If we don't have enough data to fill in the blank, then `opt_tuple_of_init_point = std::nullopt`
          std::optional<typename WFT::composite_state_ptr_t> opt_tuple_of_init_point_ptr
            = FT::guess_init_key_points(tuple_of_opt_means_ptr,mes_vect);
          // NOTE: this is were we heap allocate for the mean of new keys (or so far unknown key)

          // if all keys are guessed or were already complete, fill the container of 
          // the means for keys that were missing
          if (opt_tuple_of_init_point_ptr.has_value())
          {
            // HACK: triple zip tupple pattern
            std::apply(
                [&,this](const auto & ...opt_mean_ptr)
                {
                  std::apply(
                      [&,this](const auto & ...guessed_mean_ptr)
                      {
                        std::apply(
                            [&,this](auto...kcc) // copy, no big deal though
                            {
                              auto lambda = [&,this](const auto & _opt_mean_ptr, const auto &_guessed_mean_ptr, auto _kcc)
                              {
                                // if opt_mean_ptr has no value
                                //  then create a wrapped marginal with the guessed mean pointer
                                if (!_opt_mean_ptr.has_value())
                                {
                                  using marginal_t = ::sam::Marginal::BaseMarginal<typename decltype(_kcc)::KeyMeta_t>;
                                  using wrapped_marginal_t = ::sam::Marginal::WrapperPersistentMarginal<marginal_t>; 
                                  auto wrapped_marginal = wrapped_marginal_t(_kcc.key_id, _guessed_mean_ptr);
                                  // TODO: intermediary step before updating the marginal: infer a covariance (difficulty ***)
                                  // insert the marginal we just created in the system's marginal container
                                  // this->all_marginals_.
                                  //   template insert_in_marginal_container<wrapped_marginal_t> (wrapped_marginal); // WARNING: marginal refactor: push_back
                                                                                                                  // FIX: remove old, keep only competing
                                  this->all_vectors_marginals_.template push_back<wrapped_marginal_t>(wrapped_marginal);
                                }
                              };
                              ((lambda(opt_mean_ptr,guessed_mean_ptr,kcc)),...);
                            },KccSet);
                      }
                      , opt_tuple_of_init_point_ptr.value());

                }
                , tuple_of_opt_means_ptr);
          }
          else
          {
            // TODO: FEATURE: emplace factor in a staging container if unable to guess all factors init points
            throw std::runtime_error("Unable to determine all init points for this factor");
          }
          // emplace back in the structure
          vector_of_wrapped_factors.emplace_back(factor_id,mes_vect,measure_cov,keys_id,opt_tuple_of_init_point_ptr.value());
              
          // recompute keys_affectation
          // this takes longuer
          this->keys_affectation = SystemConverter::compute_keys_affectation(this->all_factors_tuple_,this->all_vectors_marginals_.vectors_of_marginals);
      }
    }

  };

};   // namespace SAM
