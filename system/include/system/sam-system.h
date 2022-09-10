#pragma once

#include "system/bookkeeper.h"
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

namespace sam::System
{
  template <typename FACTOR_T,
            typename... FACTORS_Ts>   // I need at least one type of factor
  class SamSystem
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

    using system_info_t = SystemInfo;

    static constexpr const bool isSystFullyLinear = FACTOR_T::isLinear && ( FACTORS_Ts::isLinear && ... );

      /**
      * @brief constructor
      *
      * @param agent id
      */
    SamSystem(const std::string & agent_id)
      :agent_id(agent_id),bookkeeper_(Bookkeeper(agent_id))  // NOTE: BOOKKEEPER: remove
    { 
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance()); 
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

      // check if factor id exists already
      // TODO: consistent management failure (throw ? return value false ?
      // std::optional ?)
      if (this->bookkeeper_.factor_id_exists(factor_id)) // NOTE: BOOKKEEPER: just loop over factor_tuple and find (same pattern as for remove_factor). A bit slow but thats ok
        throw std::runtime_error("Factor id already exists");

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

    /**
     * @brief compute vector b and sparse matrix A of the convex system
     *
     * @param system_infos system information (dimensions, indexes, graph connectivity ...)
     */
    std::tuple< Eigen::VectorXd, Eigen::SparseMatrix<double>> compute_b_A(std::size_t M , std::size_t N, std::size_t jacobian_NNZ) const // WARNING: matrix specific
    {
      // declare A, b, and triplets for A data
      Eigen::SparseMatrix<double> A(M,N);
      Eigen::VectorXd b(M);
      std::vector<Eigen::Triplet<double>> sparseA_triplets;
      sparseA_triplets.reserve(jacobian_NNZ); // expected number of nonzeros elements
      uint64_t line_counter = 0;
      //------------------------------------------------------------------//
      //                fill triplets of A and vector of b                //
      //------------------------------------------------------------------//
      std::apply(
          [this,&sparseA_triplets,&b,&line_counter](const auto & ...vect_of_wfactors)
          {
              (
               ( this->lay_out_factors_to_sparse_triplets(vect_of_wfactors,sparseA_triplets,b,line_counter) )
               , ... );
          }
          ,this->all_factors_tuple_);

      // set A from triplets, clear the triplets
      A.setFromTriplets(sparseA_triplets.begin(), sparseA_triplets.end());

      // sparseA_triplets.clear(); // useless as the object is destroy

      return {b,A};
    }

    /**
    * @brief optimisation method
    */
    void sam_optimise() // WARNING: defer to sam_optimise_impl that will defer to matrix or graphical model
    {
      if (this->bookkeeper_.getSystemInfos().number_of_factors == 0) return; // NOTE: BOOKKEEPER: just compute M, and if M=0 return

      // scoped timer
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      // get some dimension constants of the system
      SystemInfo system_infos = this->bookkeeper_.getSystemInfos();
      // uint nnz2 = system_infos.nnz;
      // int        MM            = system_infos.aggr_dim_mes;
      // int        NN          = system_infos.aggr_dim_keys;

      // std::size_t nnz = std::apply([](const auto & ...vect_of_wf)
      //     { 
      //       return  ((std::remove_cvref_t<decltype(vect_of_wf)>::value_type::Factor_t::factor_process_matrix_t::SizeAtCompileTime  
      //                   * vect_of_wf.size()) + ...); 
      //     },this->all_factors_tuple_);

      size_t M = MatrixConverter::Scalar::M(this->all_factors_tuple_);
      size_t N = MatrixConverter::Scalar::N(this->all_marginals_.data_map_tuple);
      size_t nnz_jacobian = MatrixConverter::Scalar::JacobianNNZ(this->all_factors_tuple_);
      // size_t nnz_hessian = MatrixConverter::Scalar::HessianNNZ(this->all_factors_tuple_);
      
      auto indexes_offset_M = MatrixConverter::Scalar::FactorTypeIndexesOffset(this->all_factors_tuple_);
      auto indexes_offset_N = MatrixConverter::Scalar::MarginalTypeIndexesOffset(this->all_marginals_.data_map_tuple);
      

      // NOTE: OptStats: we can have connectivity: ratio nnz/M*N (scalar matrix A density)
      //                                       or  ratio    /N*N

      // NOTE: SolverStats might have ratio

      // reset vector of quadratic error TODO: maybe do it at end of function
      this->bookkeeper_.clear_quadratic_errors(); // NOTE: BOOKKEEPER: remove

#if ENABLE_DEBUG_TRACE
      {
        PROFILE_SCOPE("print console",sam_utils::JSONLogger::Instance());
        std::cout << "### Syst: Starting an optimisation \n";
        std::cout << "### Syst: size " << M << " * " << N << '\n';
      }
#endif

      //------------------------------------------------------------------//
      //                      PRE LOOP DECLARATIONS                       //
      //------------------------------------------------------------------//
      // declare iterations counters
      int maxIter, nIter = 0;
      if constexpr (isSystFullyLinear) maxIter = 1;
      else maxIter = 3; // NOTE: start the tests with maxIter of 1
                        // NOTE: refactor: use OptimOpts
      

      //------------------------------------------------------------------//
      //                            LOOP START                            //
      //------------------------------------------------------------------//
      // loop of the iterations
      while(nIter < maxIter)
      {
        // scoped timer
        std::string timer_name = "iter" + std::to_string(nIter);
        PROFILE_SCOPE(timer_name.c_str(),sam_utils::JSONLogger::Instance());

        auto [b,A] = compute_b_A(M,N,nnz_jacobian); // later one more argument: lin point ??

        // number of nnz elements in R
        double rnnz;
       // maximum a posteriori, may represent a \hat X or \delta \hat X (NL)
        Eigen::VectorXd MaP; 
        // give A and b to the solver
        std::tie(MaP,rnnz) = solveQR(A,b); 
        // TODO: split the compute() step with the analyse pattern (can be set before the loop)
        // NOTE: tie() is used because structure binding declaration pose issues with lambda capture (fixed in c++20 apparently)
        this->bookkeeper_.set_syst_Rnnz(rnnz); // NOTE: BOOKKEEPER: replace, have an adhoc structure such as SolverStats or something, specific to each solver

        // optionaly compute the covariance matrix
        Eigen::MatrixXd SigmaCovariance;
        double Hnnz;
        std::tie(SigmaCovariance,Hnnz) = compute_covariance(A);
        // NOTE: tie() is used because structure binding declaration pose issues with lambda capture (fixed in c++20 apparently)
        this->bookkeeper_.set_syst_Hnnz(Hnnz); // NOTE: BOOKKEEPER: replace, have an adhoc structure such as SolverStats or something, specific to each solver

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
            [this, &MaP,&SigmaCovariance,&nIter](auto & ... map_to_wmarginals)
            {
              std::string scope_name = "save marginal updates";
              PROFILE_SCOPE( scope_name.c_str() ,sam_utils::JSONLogger::Instance());
              // define the function
              auto update_map_of_wmarginals = [&,this](auto & map_of_wrapped_marginals)
              {
                  using wrapped_marginal_t = typename std::decay_t<decltype(map_of_wrapped_marginals)>::mapped_type;
                  using marginal_t = typename wrapped_marginal_t::Marginal_t;
                  using tangent_space_t = typename marginal_t::Tangent_Space_t;
                  using keymeta_t = typename marginal_t::KeyMeta_t;
                  constexpr std::size_t kN = marginal_t::KeyMeta_t::kN;
                  // looping over the marginal collection and updating them with the MAP result
                  std::for_each (
                      std::execution::seq   // on M3500, sequential is still slightly faster than par_unseq (1.4 ms vs 1.625 ms)
                      , map_of_wrapped_marginals.begin()
                      , map_of_wrapped_marginals.end() 
                      , [&,this]( auto & kvpair)
                        {
                          std::string key_id = kvpair.first;
                          auto wrapped_marginal = kvpair.second;
                          // get the subvector from the Maximum A Posteriori vector
                          auto sysidx = this->bookkeeper_.getKeyInfos(key_id).sysidx;// NOTE: BOOKKEEPER: replace (second time this sysidx appears)
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
                  );
              };
              
              ( update_map_of_wmarginals(map_to_wmarginals), ...);

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

        // push accumulated squared norm
        this->bookkeeper_.push_back_quadratic_error(accumulated_syst_squared_norm/* .load() */); // NOTE: BOOKKEEPER: replace by an OptStats structure

        nIter++;
      }

      // clear quadratic error vector
      this->bookkeeper_.clear_quadratic_errors();
      // update sequence number
      nbSequence++;
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
      // FIX: URGENT: remove from bookkeeper
      // this->bookkeeper_.       // NOTE: BOOKKEEPER: remove smthing in GraphModel
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
    
    auto get_system_infos() const
    {
      return this->bookkeeper_.getSystemInfos();// NOTE: BOOKKEEPER: remove
    }
    
    // FIX: urgent get_joint-marginal etc... 
    // joint_marginal<marginals_t> get_joint_marginal()
    // {
    //
    // }
    //
    // get_full_joint()

    // get all marginals
    auto get_marginals() const
    {
      return all_marginals_.data_map_tuple;
    }
    using Wrapped_Factor_t = 
      std::tuple<
        std::vector<::sam::Factor::WrapperPersistentFactor<FACTOR_T,isSystFullyLinear>>
        , std::vector<::sam::Factor::WrapperPersistentFactor<FACTORS_Ts,isSystFullyLinear>>
        ...
        >;

    int nbSequence = 0;

    private:
    /**
     * @brief bookkeeper : store the infos of variables and factors, as well as
     * associative relations, total sizes, indexes , ordering
     */
    Bookkeeper bookkeeper_;// NOTE: BOOKKEEPER:

    std::string agent_id;

    Marginals_t all_marginals_;

    // there's at least one factor, the rest are expanded
    Wrapped_Factor_t all_factors_tuple_;

    /**
     * @brief how many different types of factor there are
     */
    constexpr static const size_t S_ = std::tuple_size<decltype(all_factors_tuple_)>::value;


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
          add_keys_to_bookkeeper<WFT>(keys_id, factor_id);
          // add the factor_id with its infos in the bookkeeper
          // last argument is a conversion from std::array to std::vector
          this->bookkeeper_.add_factor(factor_id, WFT::Factor_t::kN, WFT::Factor_t::kM, {keys_id.begin(), keys_id.end()});
// NOTE: BOOKKEEPER: remove (not replaced for now, because replacement will have no cache)
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
              
// Debug consistency check of everything
#if ENABLE_RUNTIME_CONSISTENCY_CHECKS
          // 1. checking if the bookkeeper is consistent with itself (systemInfo
          // vs whats on the std::maps)
          assert(this->bookkeeper_.are_dimensions_consistent());// NOTE: BOOKKEEPER: remove probably
          // 2. checking if the bookkeeper is consistent with the tuples of
          // vector holding the factors
          assert(this->is_system_consistent());
          // TODO: assert that aggregate size of factor containers correspond to the bookkeeper nb of factors
#endif

      }
    }


    template <typename WFT>
    auto compute_Ai_bi(const WFT & wrapped_factor)
    {
      if constexpr (isSystFullyLinear) 
        return wrapped_factor.factor.compute_Ai_bi_linear();
      else 
      {
        auto factor_data = wrapped_factor.get_current_point_data();
        return std::make_tuple(factor_data.bi, factor_data.Aiks);
      }
    }

// NOTE: BOOKKEEPER: remove
    template <typename WFT>
    void add_keys_to_bookkeeper(const std::array<std::string, WFT::Factor_t::kNbKeys>& keys_id,
                                const std::string&                          factor_id)
    {
      add_keys_to_bookkeeper_impl<WFT>(factor_id, keys_id, std::make_index_sequence<WFT::Factor_t::kNbKeys> {});
    }
// NOTE: BOOKKEEPER: remove
    template <typename WFT, std::size_t... INDEX_S>
    void add_keys_to_bookkeeper_impl(const std::string&                          factor_id,
                                     const std::array<std::string, WFT::Factor_t::kNbKeys>& keys_id,
                                     std::index_sequence<INDEX_S...>)
    {
      (add_in_bookkeeper_in_once(factor_id, keys_id[INDEX_S], std::tuple_element_t<INDEX_S, typename WFT::Factor_t::KeysSet_t>::kN),
       ...);
    }
// NOTE: BOOKKEEPER: remove
    void add_in_bookkeeper_in_once(const std::string& factor_id,
                     const std::string& key_id,
                     int                key_dimension)   // string_view?
    {
      // check if the key exists, if it doesn't, we will catch
      // TODO: a standard if/else might be more desirable (or std::optional)
      // TODO: would it be possible to check that the dimension and/or meta name
      // of the key is consistent ?
      try
      {
        this->bookkeeper_.getKeyInfos(key_id);// NOTE: BOOKKEEPER
      }
      catch (int e)
      {
        // add the key, the variable size is accessed via the factor Meta
        this->bookkeeper_.add_key(key_id, key_dimension);
      }
      // each key has a list of factors_id that it is connected, so add
      // this factor_id to it
      this->bookkeeper_.add_factor_id_to_key(key_id, factor_id);
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

#if ENABLE_RUNTIME_CONSISTENCY_CHECKS
    bool is_system_consistent()
    {
      // TODO:
      return true;
    }

    bool AreMatricesFilled()
    {
      // TODO: put after the end of the fill routine (check, for examples that
      // the last line_counter is coherent with A,b sizes)
      return true;
    }
#endif

    template <typename VECT_OF_WFT>
    void lay_out_factors_to_sparse_triplets
    (
     const VECT_OF_WFT & vect_of_wfactors
     , std::vector<Eigen::Triplet<double>> & sparseA_triplets_out
     , Eigen::VectorXd& b_out
     , uint64_t& line_counter_out
    ) const
    {
        using WFT = typename VECT_OF_WFT::value_type;
        using FT = typename WFT::Factor_t;
        // OPTIMIZE: parallel loop
        // OPTIMIZE: race: sparseA_triplets, b ; potential race : vector of factors (check)
        // https://stackoverflow.com/a/45773308
        for (const auto & wfactor : vect_of_wfactors)
        {
          // refactor proposal: 
          // - construct many smaller triplets in parallel (based on 0-col 0-row)
          // - rebase them in system indices (constant increment of line (all factor are the same here), col depends on bookkeeper data)
          //    in parallel also (concurrent calls to bookkeeper get sysidx)
          // - and join them later in a greater triplets for that type of factor
          // - at higher level, join all triplets of all factor types
          // It would be better because that would have more parallelism + returned values
          // methods would be static etc...
          std::string scope_name = "lay out " + wfactor.factor.factor_id + " in triplet";
          PROFILE_SCOPE( scope_name.c_str() ,sam_utils::JSONLogger::Instance());
          auto factor = wfactor.factor;
          // get Ai and bi (computations of Ai,bi not done here)
          auto matrices_Aik = wfactor.get_current_point_data().Aiks;
          auto bi = wfactor.get_current_point_data().bi;

          // declaring a triplets for matrices_Aik values to be associated with their
          // row/col indexes in view of its future integration into the system matrix A
          std::vector<Eigen::Triplet<double>> Ai_triplets; 
          Ai_triplets.reserve(FT::kN*FT::kM);
          
          // tuple of sysidx so that we know how to scatter the cols for every Aik
          auto tuple_of_sys_col_idx =
            std::apply(
                [this](const auto & ... key_id)
                {
                  return std::make_tuple( this->bookkeeper_.getKeyInfos(key_id).sysidx ... );// NOTE: BOOKKEEPER
                                                                                             // replace by something else
                }, factor.get_array_keys_id()
                );
          
          // placing those matrices in Ai_triplets
          std::apply(
              [this, &line_counter_out,&tuple_of_sys_col_idx, &Ai_triplets](const auto & ...Aik)
              {
                std::apply(
                    [&,this](auto... sys_col_idx)
                    {
                      (
                       (this->lay_out_Aik_in_triplets(Aik, sys_col_idx ,line_counter_out,Ai_triplets))
                       ,...
                      );
                    }
                    ,tuple_of_sys_col_idx);
              }
              , matrices_Aik);
           
          // put Ai and bi into sparseA_triplets and b
          // append bi into b -> WARNING: race condition on line_counter, and b, if parallel policy
          b_out.block<FT::kM,1>(line_counter_out,0) = bi;
          line_counter_out += FT::kM;
          // push Ai triplets into sparseA_triplets . WARNING: race condition on sparseA_triplets if parallel policy
          sparseA_triplets_out.insert(std::end(sparseA_triplets_out),std::begin(Ai_triplets),std::end(Ai_triplets));
        }
    }

    template <typename MAT>
    void lay_out_Aik_in_triplets(const MAT & Aik
        ,const int sys_col_idx
        ,const int line_counter
        ,std::vector<Eigen::Triplet<double>>& Ai_triplets_out) const
    {
      constexpr int M = MAT::RowsAtCompileTime;
      constexpr int Nk = MAT::ColsAtCompileTime;
      int offset_cols = sys_col_idx;
      int offset_rows = line_counter;
      auto spaghetti_Aik = Aik.reshaped(); // make it one dimension
      for (int i=0; i< Nk*M; i++)
      {
        int row = offset_rows + (i%M);
        int col = offset_cols + (i/M);
        Ai_triplets_out.emplace_back(row,col,spaghetti_Aik[i]);
      }
    }

  };

};   // namespace SAM
