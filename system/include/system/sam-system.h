#pragma once

#include "marginal/marginal.h"
#include "system/config.h"
#include "utils/tuple_patterns.h"
#include "utils/utils.h"
#include "system/MatrixConverter.hpp"

#include <Eigen/Sparse>
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

  struct SolverQR
  {
    static std::tuple<Eigen::MatrixXd,double> compute_covariance(const Eigen::SparseMatrix<double> & A)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      auto At = Eigen::MatrixXd(A.transpose());
      auto H = At*A;
      return {H.inverse(),H.nonZeros()}; // inverse done through partial LU
    }

    static std::tuple<Eigen::VectorXd,double> solve(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b)
    // TODO: add a solverOpts variable: check rank or not, check success, count the nnz of R or not
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      // solver
      Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
      // MAP
      {
        PROFILE_SCOPE("QR decomposition",sam_utils::JSONLogger::Instance());
        solver.compute(A);
      }
      // rank check: not considered a consistency check
      auto CheckRankTooLow = [](auto &solver,auto & A)
        { 
          PROFILE_SCOPE("Solver Rank Check",sam_utils::JSONLogger::Instance());
          return solver.rank() < A.cols();
        };

      if ( CheckRankTooLow(solver,A) )
      {
        throw std::runtime_error("RANK DEFICIENT PROBLEM");
      }
      auto back_substitution = [](auto & solver, auto & b)
        {
          PROFILE_SCOPE("Back-Substitution",sam_utils::JSONLogger::Instance());
          Eigen::VectorXd map = solver.solve(b);
          return map;
        };
      auto map = back_substitution(solver,b);
#if ENABLE_DEBUG_TRACE
      {
        std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
        std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
        // std::cout << "### Syst solver :  nnz in square root : " << solver.matrixR().nonZeros()
        //           << " (from " << A.nonZeros() << ") in Hessian."
        //           << "\n";
        // if ( Eigen::MatrixXd(solver.matrixR()).rows() < 15 )
        // {
        //   std::cout << "### Syst solver : matrix R : \n" << Eigen::MatrixXd(solver.matrixR()) << '\n';
        // }
      }
#endif
      return {map,0}; // R nnz number set at 0 (unused)
    }

   // cache : R, covariance 
  };

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

  // template <typename SOLVER_T>
  struct OptimOptions
  {
    std::size_t max_iterations = 3;  // keep it even if syst is linear
    bool compute_covariance = true;
    bool cache_covariance = this->compute_covariance && false; // cache_covariance (moot if no computation of covariancc)
    // TODO: threshold (by key type?? -> leads to template)
    //
    // TODO: have sane defaults
  };
  // TODO: perhaps a SolverOptions structure ?? but it should have a default
  //       members could be: cache_R , ordering ect...
  //       have sane default too

  // stats that are specific to this type of solver (eg QR, chol, naive etc..)
  struct SolverStats
  {
    //  rank , report_str
    //  theoritical flops 
    //  rnnz
    // ordering_method
    // ordering
    // residual (actually different from the NLog from a constant offset)
  };

  // template <typename SOLVER_T>
  struct OptimStats
  {
    OptimOptions optim_options; // the options inputs (the rest is more output-ish)
    SolverStats solver_stats;

    std::size_t nnz_jacobian_scalar;
    std::size_t rnz_jacobian_scalar; //ratio of nonzeros
    std::size_t nnz_hessian_scalar;
    std::size_t rnz_hessian_scalar;  //ratio of nonzeros

    std::size_t M,N;
    std::size_t M_semantic,N_semantic;

    bool optim_success;
    std::string report_str;
    double NLog_value_before; // negative log likelihood = sum of the factors norm2
    double NLog_value_after;
  };



  template <typename FACTOR_T,
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
    using Marginals_t = typename ::sam::Marginal::MarginalsCollection<KeyMetae_t>::type ;

    using type = System<FACTOR_T,FACTORS_Ts...>;

    using SystemHeader = SystemHeader;
    using OptimOptions = OptimOptions;
    using OptimStats = OptimStats;
    using SolverStats = SolverStats;

    static constexpr const bool isSystFullyLinear = FACTOR_T::isLinear && ( FACTORS_Ts::isLinear && ... );

    using Wrapped_Factor_t = 
      std::tuple<
        std::vector<::sam::Factor::WrapperPersistentFactor<FACTOR_T,isSystFullyLinear>>
        , std::vector<::sam::Factor::WrapperPersistentFactor<FACTORS_Ts,isSystFullyLinear>>
        ...
        >;

    SystemHeader header;

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
    * @brief compute the covariance
    *
    * @param A the measurement matrix (M*N)
    *
    * @return 
    */
    std::tuple<Eigen::MatrixXd,double> compute_covariance(const Eigen::SparseMatrix<double> & A) const // WARNING: matrix specific
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      auto At = Eigen::MatrixXd(A.transpose());
      auto H = At*A;
      return {H.inverse(),H.nonZeros()}; // inverse done through partial LU
    }

    /**
    * @brief optimisation method
    */
    OptimStats sam_optimise(const OptimOptions & optimisation_options = OptimOptions()) // WARNING: defer to sam_optimise_impl that will defer to matrix or graphical model
    {
      // scoped timer
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      size_t M = MatrixConverter::Scalar::M(this->all_factors_tuple_);
      size_t N = MatrixConverter::Scalar::N(this->all_marginals_.data_map_tuple);
      size_t nnz_jacobian = MatrixConverter::Scalar::JacobianNNZ(this->all_factors_tuple_);
      // size_t nnz_hessian = MatrixConverter::Scalar::HessianNNZ(this->all_factors_tuple_);

      
      auto M_type_idx_offsets = MatrixConverter::Scalar::FactorTypeIndexesOffset(this->all_factors_tuple_);
      auto N_type_idx_offsets = MatrixConverter::Scalar::MarginalTypeIndexesOffset(this->all_marginals_.data_map_tuple);
      
      OptimStats optim_stats;
      // NOTE: OptStats: we can have connectivity: ratio nnz/M*N (scalar matrix A density)
      //                                       or  ratio    /N*N

      // NOTE: SolverStats might have ratio rnnz/N*N

      if (M == 0) return optim_stats; // set success bool to false

#if ENABLE_DEBUG_TRACE
        std::cout << "### Syst: Starting an optimisation \n";
        std::cout << "### Syst: size " << M << " * " << N << '\n';
#endif

      //------------------------------------------------------------------//
      //                      PRE LOOP DECLARATIONS                       //
      //------------------------------------------------------------------//
      // declare iterations counters
      int maxIter, nIter = 0;
      if constexpr (isSystFullyLinear) maxIter = 1;
      else maxIter = 3; // NOTE: start the tests with maxIter of 1
                        // NOTE: refactor: use OptimOptions
      

      //------------------------------------------------------------------//
      //                            LOOP START                            //
      //------------------------------------------------------------------//
      // loop of the iterations
      while(nIter < maxIter)
      {
        // scoped timer
        std::string timer_name = "iter" + std::to_string(nIter);
        PROFILE_SCOPE(timer_name.c_str(),sam_utils::JSONLogger::Instance());

        auto [b,A] = MatrixConverter::Sparse::compute_b_A(this->all_factors_tuple_, this->all_marginals_, M,N,nnz_jacobian,M_type_idx_offsets, N_type_idx_offsets);

        // number of nnz elements in R
        double rnnz;
       // maximum a posteriori, may represent a \hat X or \delta \hat X (NL)
        Eigen::VectorXd MaP; 
        // give A and b to the solver
        std::tie(MaP,rnnz) = SolverQR::solve(A,b); 
        // TODO: split the compute() step with the analyse pattern (can be set before the loop)
        // NOTE: tie() is used because structure binding declaration pose issues with lambda capture (fixed in c++20 apparently)

        // optionaly compute the covariance matrix
        Eigen::MatrixXd SigmaCovariance;
        double Hnnz;
        std::tie(SigmaCovariance,Hnnz) = SolverQR::compute_covariance(A);
        // NOTE: tie() is used because structure binding declaration pose issues with lambda capture (fixed in c++20 apparently)

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
        std::cout << "#### Syst: Covariance Sigma("<< SigmaCovariance.rows() <<","<< SigmaCovariance.cols() <<") computed : \n" ;
        if (SigmaCovariance.rows()<15) std::cout << SigmaCovariance << '\n';
      }
#endif

        //------------------------------------------------------------------//
        //                  POST SOLVER LOOP ON MARGINALS                   //
        //        dispatch the Maximum A Posteriori subcomponents in        //
        //                     the marginals container.                     //
        //        Push the marginal result into a temporary history         //
        //             structure that will end up in the json.              //
        //            Do the same optionally for the covariance             //
        //------------------------------------------------------------------//
        std::apply(
            [this, &MaP,&SigmaCovariance,&nIter,&N_type_idx_offsets](auto & ... map_to_wmarginals)
            {
              std::apply(
                  [&,this](const auto & ... N_type_start_idx)
                  {
                      std::string scope_name = "save marginal updates";
                      PROFILE_SCOPE( scope_name.c_str() ,sam_utils::JSONLogger::Instance());
                      // define the function
                      auto lambda_update_map_of_wmarginals = [&,this](auto & map_of_wrapped_marginals, std::size_t KeyTypeStartIdx)
                      {
                          using wrapped_marginal_t = typename std::decay_t<decltype(map_of_wrapped_marginals)>::mapped_type;
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
                          for(auto it_marg =map_of_wrapped_marginals.begin(); it_marg!=map_of_wrapped_marginals.end(); it_marg++)
                          {
                            std::string key_id = it_marg->first;
                            auto wrapped_marginal = it_marg->second;
                            // get the subvector from the Maximum A Posteriori vector
                            auto sysidx = KeyTypeStartIdx + std::distance(map_of_wrapped_marginals.begin(), it_marg)* kN ;
                            auto MaP_subvector = MaP.block<kN,1>(sysidx, 0);

                            // clear previous history at first iteration
                            if (nIter ==0)  wrapped_marginal.clear_history();
                           
                            // covariance  TODO: if covariance is asked for or not  (std::nullopt if not)
                            std::optional<typename marginal_t::Covariance_t> 
                              OptSigmaCovariance{SigmaCovariance.block<kN,kN>(sysidx,sysidx)};

                            // writes the new mean (or increment in NL systems) and the new covariance in the marginal
                            if constexpr (isSystFullyLinear) 
                            {
                              // replace the mean by the maximum a posterior subvector, and save previous marginal in history
                              wrapped_marginal.save_and_replace( 
                                  marginal_t(MaP_subvector, OptSigmaCovariance) 
                                  );
                            }
                            else
                            {
                              // the Max a Posteriori is in the tangent space (R^kN technically, hat operator must be used
                              // to be in the tangent space formally)
                              wrapped_marginal.save_and_add( tangent_space_t(MaP_subvector), OptSigmaCovariance );
                            }
                          }
                          // ); // for each
                      };
                    ( lambda_update_map_of_wmarginals(map_to_wmarginals, N_type_start_idx), ...);
                  }
                  , N_type_idx_offsets);
            }
            , this->all_marginals_.data_map_tuple);

        // std::atomic<double> accumulated_syst_squared_norm  (0);
        double accumulated_syst_squared_norm  (0);
        // push in history & update data (Ai, bi, norm) in factors
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
    }

    void remove_key(const std::string & key_id)
    {
      // remove associated factors?
      // several mines here: what happen if graph becomes unconnected etc...
    }

    void sum_out(const std::string & key_id)
    {
      // not the same as remove !
    }

    auto get_all_factors() const
    {
      return this->all_factors_tuple_;
    }
    
    // TODO: urgent get_joint-marginal etc... 
    // joint_marginal<marginals_t> get_joint_marginal()
    // {
    //
    // }
    //
    // get_full_joint(){}

    // get all marginals
    auto get_marginals() const
    {
      return all_marginals_.data_map_tuple;
    }

    Marginals_t all_marginals_;

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
                return
                  std::make_tuple
                  ( 
                     this->all_marginals_
                       .template find_mean_ptr<typename decltype(kcc)::KeyMeta_t>(kcc.key_id)
                      ... 
                  );
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
                                  this->all_marginals_.
                                    template insert_in_marginal_container<wrapped_marginal_t> (wrapped_marginal);
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
              
      }
    }

    /**
     * @brief solve the system given the big matrices A and b. Use the sparseQR
     * solver.
     *
     * @param A sparse matrix
     * @param b
     * @throw rank deficient (columnwise) matrix A
     *
     * @return
     */
    std::tuple<Eigen::VectorXd,double> solveQR(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b) // WARNING: derived matrix
    // TODO: add a solverOpts variable: check rank or not, check success, count the nnz of R or not
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      // solver
      Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
      // MAP
      {
        PROFILE_SCOPE("QR decomposition",sam_utils::JSONLogger::Instance());
        solver.compute(A);
      }
      // rank check: not considered a consistency check
      auto CheckRankTooLow = [](auto &solver,auto & A)
        { 
          PROFILE_SCOPE("Solver Rank Check",sam_utils::JSONLogger::Instance());
          return solver.rank() < A.cols();
        };

      if ( CheckRankTooLow(solver,A) )
      {
        throw std::runtime_error("RANK DEFICIENT PROBLEM");
      }
      auto back_substitution = [](auto & solver, auto & b)
        {
          PROFILE_SCOPE("Back-Substitution",sam_utils::JSONLogger::Instance());
          Eigen::VectorXd map = solver.solve(b);
          return map;
        };
      auto map = back_substitution(solver,b);
#if ENABLE_DEBUG_TRACE
      {
        PROFILE_SCOPE("print console",sam_utils::JSONLogger::Instance());
        std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
        std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
        // std::cout << "### Syst solver :  nnz in square root : " << solver.matrixR().nonZeros()
        //           << " (from " << A.nonZeros() << ") in Hessian."
        //           << "\n";
        // if ( Eigen::MatrixXd(solver.matrixR()).rows() < 15 )
        // {
        //   std::cout << "### Syst solver : matrix R : \n" << Eigen::MatrixXd(solver.matrixR()) << '\n';
        // }
      }
#endif
      // return {map,solver.matrixR().nonZeros()};
      return {map,0}; // R nnz number set at 0 (unused)
    }

  };

};   // namespace SAM
