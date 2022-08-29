#ifndef SAM_SYSTEM_H_
#define SAM_SYSTEM_H_

#include "core/bookkeeper.h"
#include "core/marginal.h"
#include "core/factor.h"
#include "core/config.h"
#include "utils/tuple_patterns.h"
#include "utils/utils.h"

#include <eigen3/Eigen/Sparse>
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include <iomanip>

#include "core/system_jsonify.h"

namespace sam::System
{
  template <typename FACTOR_T,
            typename... FACTORS_Ts>   // I need at least one type of factor
  class SamSystem
  {
    public:
    // marginals: infer the different types of marginals by looking into the keymeta of the factors (and filtering duplicates)
    using ___aggrkeymeta_t
        = sam_tuples::tuple_cat_t<typename FACTOR_T::KeyMetas_t, typename FACTORS_Ts::KeyMetas_t ... >;
    // remove duplicates
    using ___uniq_keymeta_set_t = typename sam_tuples::tuple_filter_duplicate<___aggrkeymeta_t>::type ;
    // declare marginal container type of those keymetas
    using marginals_t = ::sam::Marginal::MarginalsContainer<___uniq_keymeta_set_t> ;
    // declare marginal histories (over the span of iterative linearization) type
    using marginals_histories_container_t = ::sam::Marginal::MarginalsHistoriesContainer<___uniq_keymeta_set_t>;
    // declare factor histories (over the span of iterative linearization) type
    using factors_histories_t = FactorsHistoriesContainer<FACTOR_T, FACTORS_Ts ...>;

    using system_info_t = SystemInfo;

    static constexpr const bool isSystFullyLinear = FACTOR_T::isLinear && ( FACTORS_Ts::isLinear && ... );

      /**
      * @brief constructor
      *
      * @param agent id
      */
    SamSystem(const std::string & agent_id)
      :agent_id(agent_id),bookkeeper_(Bookkeeper(agent_id)) 
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
    std::tuple<Eigen::MatrixXd,double> compute_covariance(const Eigen::SparseMatrix<double> & A)
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
      if (this->bookkeeper_.factor_id_exists(factor_id))
        throw std::runtime_error("Factor id already exists");

      // emplace a factor in the correct container
      place_factor_in_container<0, FT>(factor_id, mes_vect, measure_cov, keys_id);
    }

    /**
     * @brief compute vector b and sparse matrix A of the convex system
     *
     * @param system_infos system information (dimensions, indexes, graph connectivity ...)
     */
    std::tuple< Eigen::VectorXd, Eigen::SparseMatrix<double>> compute_b_A(const SystemInfo & system_infos) const
    {
      uint nnz = system_infos.nnz;
      int        M            = system_infos.aggr_dim_mes;
      int        N            = system_infos.aggr_dim_keys;

      // declare A & b
      Eigen::SparseMatrix<double> A(M,N);
      Eigen::VectorXd b(M);

      std::vector<Eigen::Triplet<double>> sparseA_triplets;
      sparseA_triplets.reserve(nnz); // expected number of nonzeros elements
      uint64_t line_counter = 0;
      //------------------------------------------------------------------//
      //                fill triplets of A and vector of b                //
      //------------------------------------------------------------------//
      //  OPTIMIZE: the computation of {partAi...} & bi for each factor could be assumed to be already done (at factor ctor, or after the lin point is analysed)
      //  OPTIMIZE: EXPECTED PERFORMANCE GAIN : low to medium 
      sam_tuples::for_each_in_tuple(
        this->all_factors_tuple_,
        [this,&sparseA_triplets,&b,&line_counter](auto& vect_of_f, auto NIET) // NOTE: I unusable (not constexpr-able)
        {
          using factor_t =typename std::decay_t<decltype(vect_of_f)>::value_type;
          // OPTIMIZE: parallel loop
          // OPTIMIZE: race: sparseA_triplets, b ; potential race : vector of factors (check)
          // https://stackoverflow.com/a/45773308
          for(auto & factor : vect_of_f)
          {
            PROFILE_SCOPE( factor.factor_id.c_str() ,sam_utils::JSONLogger::Instance());
            // compute Ai and bi 
            // // OPTIMIZE: unecessary if this is the last iteration, low-to-medium performance hit
            auto [bi, matrices_Aik] = compute_Ai_bi<factor_t>(factor);

            // declaring a triplets for matrices_Aik values to be associated with their
            // row/col indexes in view of its future integration into the system matrix A
            std::vector<Eigen::Triplet<double>> Ai_triplets; 
            Ai_triplets.reserve(factor_t::kN*factor_t::kM);
            constexpr int mesdim = factor_t::kM;
            
            // placing those matrices in Ai_triplets
            int k = 0; // tuple idx
            sam_tuples::for_each_in_const_tuple( matrices_Aik,
            [this, &mesdim, &line_counter, &Ai_triplets, &k, &factor](auto & Aik, auto NIET)
            {
                int Nk = Aik.cols();
                auto key_id = factor.keys_id[k] ; k++; // WARNING: race condition if parallel policy
                int colIdxInSystemA = this->bookkeeper_.getKeyInfos(key_id).sysidx;
                auto spaghetti_Aik = Aik.reshaped(); // make it one dimension
                for (int i=0; i< Nk*mesdim; i++)
                {
                  int row = line_counter + (i%mesdim);
                  int col = colIdxInSystemA + i /mesdim;
                  Ai_triplets.emplace_back(row,col,spaghetti_Aik[i]);
                }
            });

             
            // put Ai and bi into sparseA_triplets and b
            // append bi into b -> WARNING: race condition on line_counter, and b, if parallel policy
            b.block<mesdim,1>(line_counter,0) = bi;
            line_counter += mesdim;
            // push Ai triplets into sparseA_triplets . WARNING: race condition on sparseA_triplets if parallel policy
            sparseA_triplets.insert(std::end(sparseA_triplets),std::begin(Ai_triplets),std::end(Ai_triplets));
          }
        }
      );

      // set A from triplets, clear the triplets
      A.setFromTriplets(sparseA_triplets.begin(), sparseA_triplets.end());
      sparseA_triplets.clear(); // doesnt alter the capacity, so the .reserve( N ) is still valid 
    }

    /**
    * @brief QR optimisation method
    *
    * @param logger the log
    */
    void sam_optimise(sam_utils::JSONLogger& logger = sam_utils::JSONLogger::Instance())
    {
      // FIX: uncontrolled failure if the system is empty (no factors)

      // scoped timer
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      // get some dimension constants of the system
      SystemInfo system_infos = this->bookkeeper_.getSystemInfos();
      uint nnz = system_infos.nnz;
      int        M            = system_infos.aggr_dim_mes;
      int        N            = system_infos.aggr_dim_keys;

      // reset vector of quadratic error TODO: maybe do it at end of function
      this->bookkeeper_.clear_quadratic_errors();

#if ENABLE_DEBUG_TRACE
      {
        PROFILE_SCOPE("print console",sam_utils::JSONLogger::Instance());
        std::cout << "### Syst: Starting an optimisation \n";
        std::cout << "### Syst: size " << M << " * " << N << '\n';
        // TODO: URGENT: print init point on NL systems
        // loop over marginals, and print current mean
      }
#endif
      // auto [b,A] = compute_A_b( tuple_of_factor_list, system_infos ); // later one more argument: lin point ??

      // declare A & b
      Eigen::SparseMatrix<double> A(M,N);
      Eigen::VectorXd b(M);

      // start compute_b_A
      std::vector<Eigen::Triplet<double>> sparseA_triplets;
      sparseA_triplets.reserve(nnz); // expected number of nonzeros elements
      uint64_t line_counter = 0;
      //------------------------------------------------------------------//
      //                fill triplets of A and vector of b                //
      //------------------------------------------------------------------//
      //  OPTIMIZE: the computation of {partAi...} & bi for each factor could be assumed to be already done (at factor ctor, or after the lin point is analysed)
      //  OPTIMIZE: EXPECTED PERFORMANCE GAIN : low to medium 
      sam_tuples::for_each_in_tuple(
        this->all_factors_tuple_,
        [this,&sparseA_triplets,&b,&line_counter](auto& vect_of_f, auto NIET) // NOTE: I unusable (not constexpr-able)
        {
          using factor_t =typename std::decay_t<decltype(vect_of_f)>::value_type;
          // OPTIMIZE: parallel loop
          // OPTIMIZE: race: sparseA_triplets, b ; potential race : vector of factors (check)
          // https://stackoverflow.com/a/45773308
          for(auto & factor : vect_of_f)
          {
            PROFILE_SCOPE( factor.factor_id.c_str() ,sam_utils::JSONLogger::Instance());
            // compute Ai and bi 
            // // OPTIMIZE: unecessary if this is the last iteration, low-to-medium performance hit
            auto [bi, matrices_Aik] = compute_Ai_bi<factor_t>(factor);

            // declaring a triplets for matrices_Aik values to be associated with their
            // row/col indexes in view of its future integration into the system matrix A
            std::vector<Eigen::Triplet<double>> Ai_triplets; 
            Ai_triplets.reserve(factor_t::kN*factor_t::kM);
            constexpr int mesdim = factor_t::kM;
            
            // placing those matrices in Ai_triplets
            int k = 0; // tuple idx
            sam_tuples::for_each_in_const_tuple( matrices_Aik,
            [this, &mesdim, &line_counter, &Ai_triplets, &k, &factor](auto & Aik, auto NIET)
            {
                int Nk = Aik.cols();
                auto key_id = factor.keys_id[k] ; k++; // WARNING: race condition if parallel policy
                int colIdxInSystemA = this->bookkeeper_.getKeyInfos(key_id).sysidx;
                auto spaghetti_Aik = Aik.reshaped(); // make it one dimension
                for (int i=0; i< Nk*mesdim; i++)
                {
                  int row = line_counter + (i%mesdim);
                  int col = colIdxInSystemA + i /mesdim;
                  Ai_triplets.emplace_back(row,col,spaghetti_Aik[i]);
                }
            });

             
            // put Ai and bi into sparseA_triplets and b
            // append bi into b -> WARNING: race condition on line_counter, and b, if parallel policy
            b.block<mesdim,1>(line_counter,0) = bi;
            line_counter += mesdim;
            // push Ai triplets into sparseA_triplets . WARNING: race condition on sparseA_triplets if parallel policy
            sparseA_triplets.insert(std::end(sparseA_triplets),std::begin(Ai_triplets),std::end(Ai_triplets));
          }
        }
      );

      // set A from triplets, clear the triplets
      A.setFromTriplets(sparseA_triplets.begin(), sparseA_triplets.end());
      sparseA_triplets.clear(); // doesnt alter the capacity, so the .reserve( N ) is still valid 
      // end compute_A_b

      // TODO: declare the eigen SPQR solver here, and analyse the pattern (because A pattern will not change in the loop)


      //------------------------------------------------------------------//
      //                      PRE LOOP DECLARATIONS                       //
      //------------------------------------------------------------------//
      // declare iterations counters
      int maxIter, nIter = 0;
      if constexpr (isSystFullyLinear) maxIter = 1;
      else maxIter = 3; // NOTE: start the tests with maxIter of 1
      // history OPTIMIZE: could be class member that would be reset here ? Expected gain almost none
      marginals_histories_container_t marginals_histories_container;
      factors_histories_t factors_histories;
      

      //------------------------------------------------------------------//
      //                            LOOP START                            //
      //------------------------------------------------------------------//
      // loop of the iterations
      while(nIter < maxIter)
      {
        // scoped timer
        std::string timer_name = "iter" + std::to_string(nIter);
        PROFILE_SCOPE(timer_name.c_str(),sam_utils::JSONLogger::Instance());

        // set A from the triplets (alrea)
        if(nIter > 0)
        {
          A.setFromTriplets(sparseA_triplets.begin(), sparseA_triplets.end());
          sparseA_triplets.clear();
        }

        // number of nnz elements in R
        double rnnz;
       // maximum a posteriori, may represent a \hat X or \delta \hat X (NL)
        Eigen::VectorXd MaP; 
        // give A and b to the solver
        std::tie(MaP,rnnz) = solveQR(A,b); // TODO: split the compute() step with the analyse pattern (can be set before the loop)
        // NOTE: tie() is used because structure binding declaration pose issues with lambda capture (fixed in c++20 apparently)
        this->bookkeeper_.set_syst_Rnnz(rnnz);

        // optionaly compute the covariance matrix
        Eigen::MatrixXd SigmaCovariance;
        double Hnnz;
        std::tie(SigmaCovariance,Hnnz) = compute_covariance(A);
        // NOTE: tie() is used because structure binding declaration pose issues with lambda capture (fixed in c++20 apparently)
        this->bookkeeper_.set_syst_Hnnz(Hnnz);
        // OPTIMIZE: easier (on memory? on cpu?) to compute each block covariance separately ? (and in parallel ?) 

#if ENABLE_DEBUG_TRACE
      {
        PROFILE_SCOPE("print console",sam_utils::JSONLogger::Instance());
        std::cout << "#### Iteration : " << nIter << '\n';
        std::cout << "#### Syst: A("<< A.rows() <<","<< A.cols() <<") computed :\n";
        // only display if matrix not too big
        if ( A.rows() < 30 && nIter ==0) std::cout << Eigen::MatrixXd(A) << "\n\n";
        // std::cout << "#### Syst: R computed :\n" << Eigen::MatrixXd(A) <<
        // "\n\n";
        std::cout << "#### Syst: b computed :\n";
        if ( b.rows() < 30 && nIter ==0) std::cout << b << "\n";
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
        static_assert( std::is_same_v<decltype(MaP),typename Eigen::VectorXd>);
        sam_tuples::for_each_in_tuple(this->all_marginals_.data_map_tuple,
        [this, &MaP, &SigmaCovariance, &nIter, &marginals_histories_container ](auto & map_to_marginal_ptr, auto margTypeIdx)
        {
          using marginal_t = typename std::decay_t<decltype(map_to_marginal_ptr)>::mapped_type::element_type;
          using tangent_space_t = typename marginal_t::Tangent_Space_t;
          using keymeta_t = typename marginal_t::KeyMeta_t;
          constexpr std::size_t kN = marginal_t::KeyMeta_t::kN;
          PROFILE_SCOPE( keymeta_t::kKeyName ,sam_utils::JSONLogger::Instance());
          // looping over the marginal collection and updating them with the MAP result
          for (auto & pair : map_to_marginal_ptr)
          {
            std::string key_id = pair.first;
            auto marginal_ptr = pair.second;
            auto sysidx = this->bookkeeper_.getKeyInfos(key_id).sysidx;
            auto marginal_MaP = MaP.block<kN,1>(sysidx, 0);
            // writes the new mean (or increment in NL systems) and the new covariance in the marginal
            if constexpr (isSystFullyLinear) 
              // replace the eman
              *(marginal_ptr->mean_ptr) =  marginal_MaP;
            else
              // increment the mean
              // BUG: URGENT: update in not the same for R^n,SE(n) and SO(n). For SE(n), it should be += manif::SE2Tangentd(marginalMaP)
              *(marginal_ptr->mean_ptr) += tangent_space_t(marginal_MaP);
                                                                       //
            marginal_ptr->covariance = SigmaCovariance.block<kN,kN>( sysidx, sysidx );
            // fill/complete the history
            if (nIter == 0)
            {
              marginals_histories_container.template insert_new_marginal<marginal_t>(key_id,marginal_ptr);
            }
            else
            {
              // push some new data in history
              marginals_histories_container.template push_marginal_history<marginal_t>(key_id,marginal_ptr);
            }
          }
        });

        //------------------------------------------------------------------//
        //                    POST SOLVER LOOP ON FACTOR                    //
        //          compute norm and accumulate norm, push norm in          //
        //           history, fill the new b vector, fill the new           //
        //                         sparseAtriplets                          //
        //------------------------------------------------------------------//
        sparseA_triplets.clear();
        int line_counter = 0;  // NOTE: the line counter makes the whole thing difficult to parallelize
        double accumulated_syst_squared_norm (0); // TODO: atomic<double> (but atomic increment is supported only since c++20)
        // for every factor list in the tuple
        sam_tuples::for_each_in_tuple(this->all_factors_tuple_,[this,&line_counter, &accumulated_syst_squared_norm,&b,&sparseA_triplets,&nIter,&factors_histories]
        (auto & factors, auto factTypeIdx)
        {
          using factor_t = typename std::remove_const_t<std::decay_t<decltype(factors)>>::value_type;
          PROFILE_SCOPE( factor_t::kFactorLabel ,sam_utils::JSONLogger::Instance());
          // for every factor in the list
          for (auto & factor : factors) // OPTIMIZE: make parallel for_each. Races: syst_norm, line_counter
          {
            PROFILE_SCOPE( factor.factor_id.c_str() ,sam_utils::JSONLogger::Instance());
            // OPTIMIZE: these steps could (independently whether or not the loop is parallel)
            // OPTIMIZE: 1st thread: norm computing and accumulation
            // OPTIMIZE: 2nd thread: Ai bi compute and incorporation on 
            // OPTIMIZE: medium gain, but improved readability
            // norm of the factor: \|h(xmap)-z\|_R (no square)
            //  at the current map
            // std::cout << "keymeanview :" << *(std::get<0>(factor.keys_set).key_mean_view) // OK: proper
            //   << '\n';
            double norm_factor = factor.factor_norm_at_current_lin_point();
#if ENABLE_DEBUG_TRACE
            std::cout << "norm factor : " << norm_factor << '\n';
#endif
            accumulated_syst_squared_norm += norm_factor*norm_factor; // WARNING: race condition if parallel policy

            constexpr int mesdim= factor_t::kM;

            // compute Ai and bi 
            // OPTIMIZE: unecessary if this is the last iteration, low-to-medium performance hit
            // if euclidian factor: bi = factor.rosie OR bi = compute_bi_nl()
            //                      for each k, Aik = kcm.compute_part_A()
            //                      So put this under  [bi, tuple(Ai1 ,Ai2 ...) ] = factor.compute_Ai_bi();
            // else if LieGroup factor:
            //                      Put it all under one function
            //                      declare elementary Jacobians Je
            //                      bi =  -rho * r(X_0) with Js computed on the fly when computing r(X_0)
            //                      for each k, Aik = rho * some_function ( Je  )
            //                              (indeed  Aik := \rho . \frac{ \partial r(X) }{ \partial X } | _X_0 which depends on the elementary Jacobians )
            //                      return tuple(bi, tuple(Ai1, Ai2 ...))
            //                     So put this under [bi , tuple (Ai1, Ai2 ...)] = factor.compute_Ai_bi();
            //
            //  the main task is to decouple the usage from the affectation in bigger matrices A & b below

            // bi is factor_t::criterion_t, matrices_Aik is tuple( Ai1, Ai2 ,... ) i.e. submatrices of row length = factor_t::kM and sum of their columns is factor_t::kN
            auto [bi, matrices_Aik] = compute_Ai_bi<factor_t>(factor);

            // declaring a triplets for matrices_Aik values to be associated with their
            // row/col indexes in view of its future integration into the system matrix A
            std::vector<Eigen::Triplet<double>> Ai_triplets; 
            Ai_triplets.reserve(factor_t::kN*factor_t::kM);
            
            // placing those matrices in Ai_triplets
            int k = 0; // tuple idx
            sam_tuples::for_each_in_const_tuple( matrices_Aik,
            [this, &mesdim, &line_counter, &Ai_triplets, &k, &factor](auto & Aik, auto NIET)
            {
                int Nk = Aik.cols();
                auto key_id = factor.keys_id[k] ; k++; // WARNING: race condition if parallel policy
                int colIdxInSystemA = this->bookkeeper_.getKeyInfos(key_id).sysidx;
                auto spaghetti_Aik = Aik.reshaped(); // make it one dimension
                for (int i=0; i< Nk*mesdim; i++)
                {
                  int row = line_counter + (i%mesdim);
                  int col = colIdxInSystemA + i /mesdim;
                  Ai_triplets.emplace_back(row,col,spaghetti_Aik[i]);
                }
            });

            // put Ai and bi into sparseA_triplets and b
            // append bi into b -> WARNING: race condition on line_counter, and b, if parallel policy
            b.block<mesdim,1>(line_counter,0) = bi;
            line_counter += mesdim;
            // push Ai triplets into sparseA_triplets . WARNING: race condition on sparseA_triplets if parallel policy
            sparseA_triplets.insert(std::end(sparseA_triplets),std::begin(Ai_triplets),std::end(Ai_triplets));
            
            // push factor norm into a history (create it if it is first iteration)
            if (nIter == 0)
              factors_histories.template insert_new_factor_history<factor_t>(factor.factor_id, factor,norm_factor);
            else
              factors_histories.template push_data_in_factor_history<factor_t>(factor.factor_id,norm_factor);
          }

        });

        // push accumulated squared norm
        this->bookkeeper_.push_back_quadratic_error(accumulated_syst_squared_norm);

        nIter++;
      }

      // TODO: remove here (2nd phase)
      this->marginals_histories_container = marginals_histories_container;
      this->factors_histories = factors_histories;

      // TODO: remove here after (firstly)

      // // declare the json graph
      // Json::Value json_graph;
      // json_graph["header"] = write_header(this->bookkeeper_.getSystemInfos());
      //
      // // write json factors list from factor history
      // // TODO: future async
      // Json::Value json_factors;
      // sam_tuples::for_each_in_const_tuple(
      // factors_histories.factors_histories_container, 
      // [this,&json_factors](const auto & map_factor_history, auto NIET)
      // {
      //   using factor_history_t = typename std::remove_const_t<std::decay_t<decltype(map_factor_history)>>::mapped_type;
      //   using factor_t = typename factor_history_t::Factor_t;
      //   
      //   for (const auto & pair : map_factor_history)
      //   {
      //     const auto & factor_h = pair.second;
      //     Json::Value json_factor;
      //     json_factor["factor_id"] = factor_h.factor_id;
      //     json_factor["type"]      = factor_history_t::kFactorLabel; // TODO: rename type to label
      //     json_factor["measure"]   = factor_history_t::kMeasureName;
      //     // vars_id
      //     Json::Value json_vars_id;
      //     for (const auto & key_id : factor_h.vars_id) json_vars_id.append(key_id);
      //     json_factor["vars_id"] = json_vars_id;
      //     json_factor["MAPerror"] = factor_h.norms.back();           // TODO: rename norm
      //     Json::Value json_norms;
      //     for (const auto & norm: factor_h.norms) json_norms.append(norm);
      //     json_factor["norms"]= json_norms;
      //     // report the measurement value, component by component
      //     Json::Value json_measure; // TODO: component name could be given statically so that the string comparaison would occur at compile time (micro opt)
      //     for (std::size_t i=0;i< factor_t::kMeasureComponentsName.size(); i++ )
      //     {
      //       json_measure[factor_t::kMeasureComponentsName[i]] 
      //         = factor_t::MeasureMeta_t::get_component(factor_t::kMeasureComponentsName[i],factor_h.z);
      //     }
      //     json_factor["measure"] = json_measure;
      //     // Append at the json list of factors
      //     json_factors.append(json_factor);
      //   }
      // });
      // 
      // // write json marginal list from marginal history
      // // TODO: future async
      // Json::Value json_marginals;
      // sam_tuples::for_each_in_const_tuple(
      // this->marginals_histories_container.marginal_history_tuple,
      // [this,&json_marginals](const auto & map_marginal_histories, auto NIET)
      // {
      //   using marginal_history_t = typename std::remove_const_t<std::decay_t<decltype(map_marginal_histories)>>::mapped_type;
      //   using marginal_t = typename marginal_history_t::Marginal_t;
      //   using keymeta_t = typename marginal_t::KeyMeta_t;
      //   for (const auto & pair : map_marginal_histories)
      //   {
      //     Json::Value json_marginal;
      //     auto marg_hist = pair.second;
      //     json_marginal["var_id"] = marg_hist.var_id;
      //     json_marginal["category"] = keymeta_t::kKeyName;
      //     Json::Value json_mean;
      //     for (std::size_t i =0; i< keymeta_t::components.size();i++)
      //     {
      //       // TODO: component name could be given statically so that the string comparaison would occur at compile time (micro opt)
      //       json_mean[keymeta_t::components[i]]  
      //         = keymeta_t::get_component(keymeta_t::components[i],marg_hist.iterative_means.back());
      //     }
      //     json_marginal["mean"] = json_mean;
      //     // iterative means
      //     Json::Value iterative_means;
      //     for (std::size_t j = 0 ; j < marg_hist.iterative_means.size(); j++)
      //     {
      //       Json::Value ith_json_mean;
      //       for (std::size_t i =0; i< keymeta_t::components.size();i++)
      //       {
      //         ith_json_mean[keymeta_t::components[i]] 
      //           = keymeta_t::get_component(keymeta_t::components[i],marg_hist.iterative_means[j]);
      //       }
      //       iterative_means.append(ith_json_mean);
      //     }
      //     json_marginal["iterative_means"] = iterative_means;
      //     // iterative covariance
      //     json_marginal["covariance"]["sigma"].append(std::get<0>(marg_hist.iterative_covariances.back())[0]);
      //     json_marginal["covariance"]["sigma"].append(std::get<0>(marg_hist.iterative_covariances.back())[1]);
      //     json_marginal["covariance"]["rot"]= std::get<1>(marg_hist.iterative_covariances.back());
      //     Json::Value iterative_covariances;
      //     for (std::size_t j=0; j< marg_hist.iterative_covariances.size();j++)
      //     {
      //       Json::Value covariance;
      //       covariance["sigma"].append(std::get<0>(marg_hist.iterative_covariances[j])[0]);
      //       covariance["sigma"].append((std::get<0>(marg_hist.iterative_covariances[j])[1]));
      //       covariance["rot"] = std::get<1>(marg_hist.iterative_covariances[j]);
      //       iterative_covariances.append(covariance);
      //     }
      //     json_marginal["iterative_covariances"] = iterative_covariances;
      //     json_marginals.append(json_marginal);
      //   }
      // });
      //
      // // attach json marginals & factors to the graph
      // json_graph["marginals"] = json_marginals;
      // json_graph["factors"] = json_factors;
      // 
      // logger.writeGraph(json_graph);

      // clear quadratic error vector
      this->bookkeeper_.clear_quadratic_errors();
    }

    // FIX: this is temporary, remove after 'persistentfactor' refactor (the *_histories_* wont exist)
    marginals_histories_container_t marginals_histories_container;
    factors_histories_t factors_histories;

    factors_histories_t get_factors_histories() const
    {
      return this->factors_histories;
    }

    // TODO: remove
    marginals_histories_container_t get_marginals_histories() const
    {
      return this->marginals_histories_container;
    }

    // TODO: remove
    auto get_all_factors() const
    {
      return this->all_factors_tuple_;
    }
    
    auto get_system_infos() const
    {
      return this->bookkeeper_.getSystemInfos();
    }

    Json::Value write_header(const SystemInfo & sysinfo) const
    {
      Json::Value json_header;
      json_header["robot_id"] = this->agent_id;
      json_header["seq"] = 0; // TODO:
      json_header["base_unit"] = 0.15;
      Json::Value quadratic_errors;
      for (auto qerr : sysinfo.quadratic_error)
        quadratic_errors.append(qerr);
      json_header["quadratic_errors"] = quadratic_errors;
      json_header["Rnnz"] = sysinfo.Rnnz;
      json_header["Hnnz"] = sysinfo.Hnnz;
      // some compile time information
      using def_t = sam::definitions::CompiledDefinitions;
      json_header["bla"] = def_t::blas;
      json_header["bla_vendor_mkl"] = def_t::bla_vendor_mkl;
      json_header["aggressively_optimised"] = def_t::optimised;
      json_header["openmp"] = def_t::openmp;
      json_header["timer"] = def_t::timer;
      json_header["json_output"] = def_t::json_output;
      json_header["runtime_checks"] = def_t::runtime_checks;
      json_header["debug_trace"] = def_t::debug_trace;
      // TODO: variable order  :  "variable_order"
      return json_header;
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

    // get all marginals
    auto get_marginals()
    {
      return all_marginals_.data_map_tuple;
    }

    private:
    /**
     * @brief bookkeeper : store the infos of variables and factors, as well as
     * associative relations, total sizes, indexes , ordering
     */
    Bookkeeper bookkeeper_;

    std::string agent_id;

    marginals_t all_marginals_;

    // there's at least one factor, the rest are expanded
    std::tuple<std::vector<FACTOR_T>, std::vector<FACTORS_Ts>...> all_factors_tuple_;

    /**
     * @brief how many different types of factor there are
     */
    constexpr static const size_t S_ = std::tuple_size<decltype(all_factors_tuple_)>::value;


    /**
     * @brief emplace back factor in the right container (recursive static)
     *
     * @tparam I
     * @tparam FT
     * @tparam Args
     * @param factor_id
     * @param keys
     * @param args
     */
    template <std::size_t TUPLE_IDX = 0, typename FT>
    void place_factor_in_container(const std::string&                          factor_id,
                                   const typename FT::measure_t&          mes_vect,
                                   const typename FT::measure_cov_t&           measure_cov,
                                   const std::array<std::string, FT::kNbKeys>& keys_id)
    {
      // beginning of static recursion (expanded at compile time)
      if constexpr (TUPLE_IDX == S_)
        return;
      else
      {
        // if this is the type we are looking for, emplace back in
        if constexpr (std::is_same_v<FT, factor_type_in_tuple_t<TUPLE_IDX>>)
        {
          add_keys_to_bookkeeper<FT>(keys_id, factor_id);
          // add the factor_id with its infos in the bookkeeper
          // last argument is a conversion from std::array to std::vector
          this->bookkeeper_.add_factor(factor_id, FT::kN, FT::kM, {keys_id.begin(), keys_id.end()});

          // recover the means (at least the ones available, some may not exist)
          // TODO: make it a function
          auto tuple_of_opt_means_ptr 
          = sam_tuples::reduce_array_variadically(
              keys_id,[this]<std::size_t...J>(const auto& keys_id, std::index_sequence<J...>)
                            -> typename FT::composite_of_opt_state_ptr_t
              {
                return 
                { 
                  this->all_marginals_
                  .template find_mean_ptr<typename std::tuple_element_t<J, typename FT::KeysSet_t>::KeyMeta_t>(keys_id[J])
                ... 
                };
              }
            );

          // It is probable that the above tuple contains std::nullopt.
          // Attempt to guess the full init point for this factor by using the measurement if necessary.
          // If we don't have enough data to fill in the blank, then `opt_tuple_of_init_point = std::nullopt`
          std::optional<typename FT::composite_state_ptr_t> opt_tuple_of_init_point_ptr
            = FT::guess_init_key_points(tuple_of_opt_means_ptr,mes_vect); // NOTE: heap allocation for the INIT POINT AS MEAN (make_shared)

          if (opt_tuple_of_init_point_ptr.has_value())
          {
            // iterations over several tuples
            sam_tuples::constexpr_for<FT::kNbKeys>(
            [&](auto idx)
            {
              constexpr auto tuple_idx = idx.value;
              // if the mean was not found ante-previously, insert it using the guesser result
              if ( !std::get<tuple_idx>(tuple_of_opt_means_ptr).has_value() )
              {
                // isolate the shared ptr to the guessed init point
                auto guessed_init_point_ptr = std::get<tuple_idx>(opt_tuple_of_init_point_ptr.value());
                // make a new marginal from the guessed init point
                using marginal_t = ::sam::Marginal::BaseMarginal<typename std::tuple_element_t<tuple_idx,typename FT::KeysSet_t>::KeyMeta_t>;
                std::shared_ptr<marginal_t> new_marginal_ptr = std::make_shared<marginal_t>(guessed_init_point_ptr); // NOTE: HEAP allocation for the full MARGINAL OF THE KEY  (this is a nuance from previous heap allocation)
                // TODO: intermediary step before updating the marginal: infer a covariance (difficulty ***)
                // insert the (shared ptr) marginal we just created in the system's marginal container
                this->all_marginals_.
                  template insert_in_marginal_container<marginal_t>
                  (keys_id[tuple_idx],new_marginal_ptr);
                // TODO: update the bookkeeper ? I think its already done ( CHECK: )
              }
            }
            );

          }
          else
          {
            // TODO: FEATURE: emplace factor in a staging container if 
            // TODO: more detail (which key.s failed etc..)
            throw std::runtime_error("Unable to determine all init points for this factor");
          }
          std::get<TUPLE_IDX>(this->all_factors_tuple_).emplace_back(factor_id,mes_vect,measure_cov,keys_id,opt_tuple_of_init_point_ptr.value());
              
// Debug consistency check of everything
#if ENABLE_RUNTIME_CONSISTENCY_CHECKS
          // 1. checking if the bookkeeper is consistent with itself (systemInfo
          // vs whats on the std::maps)
          assert(this->bookkeeper_.are_dimensions_consistent());
          // 2. checking if the bookkeeper is consistent with the tuples of
          // vector holding the factors
          assert(this->is_system_consistent());
          // TODO: assert that aggregate size of factor containers correspond to the bookkeeper nb of factors
#endif
        }
        else
        {
          // recursion :  compile time call
          place_factor_in_container<TUPLE_IDX + 1, FT>(factor_id, mes_vect, measure_cov, keys_id);
        }
      }
    }

    template <typename FT>
    auto compute_Ai_bi(const FT & factor)
    {
      if constexpr (isSystFullyLinear) 
        return factor.compute_Ai_bi_linear();
      else 
        return factor.compute_Ai_bi_at_current_lin_point();
    }


    template <typename FT>
    void add_keys_to_bookkeeper(const std::array<std::string, FT::kNbKeys>& keys_id,
                                const std::string&                          factor_id)
    {
      add_keys_to_bookkeeper_impl<FT>(factor_id, keys_id, std::make_index_sequence<FT::kNbKeys> {});
    }

    template <typename FT, std::size_t... INDEX_S>
    void add_keys_to_bookkeeper_impl(const std::string&                          factor_id,
                                     const std::array<std::string, FT::kNbKeys>& keys_id,
                                     std::index_sequence<INDEX_S...>)
    {
      (add_in_bookkeeper_in_once(factor_id, keys_id[INDEX_S], std::tuple_element_t<INDEX_S, typename FT::KeysSet_t>::kN),
       ...);
    }

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
        this->bookkeeper_.getKeyInfos(key_id);
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
    std::tuple<Eigen::VectorXd,double> solveQR(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b)
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

    /**
     * @brief Gets the factor type of the Ith tuple element. Use case: get some
     * static info about the factors such as :
     * - factor type name of 0th factor collection :
     * `factor_type_in_tuple_t<0>::kFactorCategory`
     * - some meta (dimensions) about the Ith factors collection:
     * `factor_type_in_tuple_t<I>::Meta_t::kMesDim`.
     *
     * The advantage is that it doesn't matter if the std::vector is not holding
     * any factor (or many)
     *
     * @tparam I
     */
    template <size_t TUPLE_IDX>
    using factor_type_in_tuple_t =
        typename std::tuple_element<TUPLE_IDX, decltype(all_factors_tuple_)>::type::value_type;
  };

};   // namespace SAM
#endif
