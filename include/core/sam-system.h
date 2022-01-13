#ifndef SAM_SYSTEM_H_
#define SAM_SYSTEM_H_

#include "core/bookkeeper.h"
#include "core/marginal.h"
#include "core/config.h"
// #include "factor_impl/anchor.hpp"
// #include "factor_impl/key-meta-position.h"
#include "utils/tuple_patterns.h"
#include "utils/utils.h"

#include <eigen3/Eigen/Sparse>
#include <unordered_set>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace SAM
{
  template <typename FACTOR_T,
            typename... FACTORS_Ts>   // I need at least one type of factor
  class SamSystem
  {
    public:
    // marginals: infer the different types of marginals by looking into the keymeta of the factors (and filtering duplicates)
    using ___aggrkeymeta_t
        = typename sam_tuples::cat_tuple_in_depth<typename FACTOR_T::KeysSet_t, typename FACTORS_Ts::KeysSet_t ... >::type;
    // remove duplicates
    using ___uniq_keymeta_set_t = typename sam_tuples::tuple_filter_duplicate<___aggrkeymeta_t>::type ;
    // declare marginal container type of those keymetas
    using marginals_t = MarginalsContainer<___uniq_keymeta_set_t> ;
    // declare marginal histories (over the span of iterative linearization) type
    using marginals_histories_t = MarginalsHistoriesContainer<___uniq_keymeta_set_t>;

    static constexpr const bool isSystFullyLinear = FACTOR_T::isLinear && ( FACTORS_Ts::isLinear && ... );

    SamSystem(const std::string & agent_id):agent_id(agent_id) { PROFILE_FUNCTION(sam_utils::JSONLogger::Instance()); }

    std::tuple<Eigen::MatrixXd,double> compute_covariance(const Eigen::SparseMatrix<double> & A)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      auto At = Eigen::MatrixXd(A.transpose());
      auto H = At*A;
      return {H.inverse(),H.nonZeros()}; // inverse done through partial LU
    }

    template <typename FT>
    void register_new_factor(const std::string&                          factor_id,
                             const typename FT::measure_vect_t&          mes_vect,
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

    // URGENT:
    void sam_optimize_v2()
    {
      // scoped timer
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      // get some dimension constants of the system
      SystemInfo system_infos = this->bookkeeper_.getSystemInfos();
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
        [this,&sparseA_triplets,&b,&line_counter](auto& vect_of_f, auto I) // NOTE: I unusable (not constexpr-able)
        {
          using factor_t =typename std::decay_t<decltype(vect_of_f)>::value_type;
          // OPTIMIZE: parallel loop
          // OPTIMIZE: race: sparseA_triplets, b ; potential race : vector of factors (check)
          // https://stackoverflow.com/a/45773308
          for(auto & factor : vect_of_f)
          {
            PROFILE_SCOPE( factor.factor_id.c_str() ,sam_utils::JSONLogger::Instance());
            //------------------------------------------------------------------//
            //                        refactoring notes                         //
            //------------------------------------------------------------------//
            // declare Ai,bi
            typename factor_t::measure_vect_t b_i;
            std::vector<Eigen::Triplet<double>> Ai_triplets; 
            Ai_triplets.reserve(factor_t::kN*factor_t::kM);
            if constexpr (isSystFullyLinear)
            {
              b_i = factor.rosie;
              std::apply(
                  [this, &Ai_triplets, line_counter](auto&&... keycc)
                  { (this->compute_partialA_and_fill_triplet(keycc, Ai_triplets, line_counter), ...); },
                  factor.keys_set);
            }
            else
            {
              // NOTE: this assumes lin points already been set elsewhere
              b_i = factor.compute_b_nl(); // or without mean_tup as factor is supposed to have it inside
              // logic is done internally
              std::apply(
                  [this, &Ai_triplets, line_counter](auto&&... keycc)
                  { (this->compute_partialA_and_fill_triplet(keycc, Ai_triplets, line_counter), ...); },
                  factor.keys_set);
            }

            // Append b_i into b
            b.block<factor_t::kM, 1>(line_counter, 0) = b_i;

            // Append Ai_triplets in sparseA_triplets
            sparseA_triplets.insert(std::end(sparseA_triplets),std::begin(Ai_triplets),std::end(Ai_triplets));

            // increment the line number by as many lines filled here
            line_counter += factor_t::kM;
          }
        }
      );

      // set A from triplets, clear the triplets
      A.setFromTriplets(sparseA_triplets.begin(), sparseA_triplets.end());
      sparseA_triplets.clear(); // doesnt alter the capacity, so the .reserve( N ) is still valid 

      // TODO: declare the eigen SPQR solver here, and analyse the pattern (because A pattern will not change in the loop)


      //------------------------------------------------------------------//
      //                      PRE LOOP DECLARATIONS                       //
      //------------------------------------------------------------------//
      // declare iterations counters
      int maxIter, nIter = 0;
      if constexpr (isSystFullyLinear) maxIter = 1;
      else maxIter = 3; // NOTE: start the tests with maxIter of 1
      // history OPTIMIZE: could be class member that would be reset here ? Expected gain almost none
      marginals_histories_t marginals_histories;
      

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
        if(nIter > 0) A.setFromTriplets(sparseA_triplets.begin(), sparseA_triplets.end());

        // give A and b to the solver
        auto [Xmap,qr_error, rnnz] = solveQR(A, b); // TODO: split the compute() step with the analyse pattern (can be set before the loop)
        this->bookkeeper_.set_syst_Rnnz(rnnz);
        this->bookkeeper_.set_syst_resolution_error(qr_error); // TODO: remove qr_error

        // optionaly compute the covariance
        auto [SigmaCovariance,Hnnz] = compute_covariance(A);
        this->bookkeeper_.set_syst_Hnnz(Hnnz);

        // update the Marginals here
        // update the marginal history
        sam_tuples::for_each_in_tuple(this->all_marginals_.data_map_tuple,
        [this, Xmap = std::ref(Xmap), SigmaCovariance = std::ref(SigmaCovariance),&nIter, &marginals_histories ](auto & map_to_marginal_ptr, auto margTypeIdx)
        {
          using marginal_t = typename std::decay<decltype(map_to_marginal_ptr)>::mapped_type::element_type;
          using keymeta_t = typename marginal_t::KeyMeta_t;
          constexpr std::size_t kN = marginal_t::kN;
          // looping over the marginal collection and updating them with the MAP result
          for (auto & pair : map_to_marginal_ptr)
          {
            std::string key_id = pair.first;
            auto marginal_ptr = pair.second;
            auto sysidx = this->bookkeeper_.getKeyInfos(key_id).sysidx;
            // writes the new mean and the new covariance in the marginal
            *(marginal_ptr->mean_ptr) =  Xmap.block<kN,1>(sysidx, 0); // FIX: LIN SYST -> affect new mean, NL SYST -> additive
            marginal_ptr->covariance = SigmaCovariance.block<kN,kN>( sysidx, sysidx );
            // fill/complete the history
            if (nIter == 0)
            {
              marginals_histories.template insert_new_marginal<keymeta_t>(key_id,marginal_ptr);
            }
            else
            {
              // push some new data in history
              marginals_histories.template push_marginal_history<keymeta_t>(key_id,marginal_ptr);
            }
          }
        });

        // TODO: URGENT: post process here
        // sparseA_triplets.clear(); 
        // int line_number_b = 0;  // NOTE: the line counter makes the whole thing difficult to parallelize
        // std::atomic<double> accumulated_error (0);
        // for each in the factors tuple
        //    for each factor in factors
        //      accumulated_error += factor.error
        //      add the factor.error in json
        //      // now compute Ai,bi : first bi
        //      bi = factor.rosie; // or bi = factor.compute_b_nl(); // in NL
        //      //  append bi into b -> *** increment a line number
        //      constexpr int mesdim                = factor_t::kM;
        //      b.block<mesdim, 1>(line_counter, 0) = b_i;
        //      line_counter += mesdim;
        //      // next Ai
        //      declare a triplet
        //      for each kcm of this factor
        //        partAi = kcm.compute_partAi() // use x0
        //        emplace each element of partAi into triplets  (line_counter dependency)
        //      sparseA_triplets.insert(std::end(sparseA_triplets),std::begin(Ai_triplets),std::end(Ai_triplets))

        nIter++;
      }
      // TODO: URGENT:  json : header
      
      // TODO: URGENT: publish json
      // merge marginal histories into a vector of json values
      // merge factor histories into a vector of json values

    }

    void sam_optimize()
    // TODO: add a solverOpts variable: check rank or not, check success, write
    // TODO: bookkeeper, compute covariance etc..
    {
      // scoped timer
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      ;

      SystemInfo system_infos = this->bookkeeper_.getSystemInfos();
      int        M            = system_infos.aggr_dim_mes;
      int        N            = system_infos.aggr_dim_keys;

      uint nnz = this->bookkeeper_.getSystemInfos().nnz;

      // the big steps: fill the system (sparse matrix A and rhs vector b)
      auto [A, b] = assemble_system(M, N, nnz);
      // and solve the system
      auto   [Xmap,qr_error,rnnz]                 = solveQR(A, b);
      // TODO: in NL , add this to the linpoint
      this->bookkeeper_.set_syst_Rnnz(rnnz);
      this->bookkeeper_.set_syst_resolution_error(qr_error);

      // given the map, compute NLL error
      // double aggregate_factors_error = compute_factor_system_residual(Xmap);
      // optionaly compute the covariance
      auto [SigmaCovariance,Hnnz] = compute_covariance(A);
      this->bookkeeper_.set_syst_Hnnz(Hnnz);

#if ENABLE_DEBUG_TRACE
      {
        PROFILE_SCOPE("print console",sam_utils::JSONLogger::Instance());
        std::cout << "#### Syst: A("<< A.rows() <<","<< A.cols() <<") computed :\n";
        // only display if matrix not too big
        if ( A.rows() < 15 ) std::cout << Eigen::MatrixXd(A) << "\n\n";
        // std::cout << "#### Syst: R computed :\n" << Eigen::MatrixXd(A) <<
        // "\n\n";
        std::cout << "#### Syst: b computed :\n";
        if ( b.rows() < 15 ) std::cout << b << "\n";
        std::cout << "#### Syst: MAP computed :\n" << Xmap << '\n';
        std::cout << "#### Syst: Covariance Sigma("<< SigmaCovariance.rows() <<","<< SigmaCovariance.cols() <<") computed : \n" ;
        if (SigmaCovariance.rows()<15) std::cout << SigmaCovariance << '\n';
      }
#endif
      
      // Another factor loop that does several things while traversing.
      // dispatch xmap subvectors to marginals (with covariance if option is set), 
      // compute the error for each factor, write the factor
      // in the json graph. If option is set, compute next linearization points ().
      // update bookkeeper ?
      // OPTIMIZE: Lots of operation could be async
      this->post_process_loop(Xmap,SigmaCovariance);

    }

    // TODO: overload when no cov matrix is given
    void post_process_loop(const Eigen::VectorXd & Xmap, const Eigen::MatrixXd & SigmaCov, sam_utils::JSONLogger& logger = sam_utils::JSONLogger::Instance())
    {
      PROFILE_FUNCTION( sam_utils::JSONLogger::Instance() );
      // declare a json structure to hold the factor graph (will be attatched to the logger)
      Json::Value json_graph;
      // first define a set to registered that a marginal has been treated,
      // since we loop the factors, we encounter the same key several times.
      std::unordered_set<std::string> already_processed_keys = {};  // TODO: check the size of the set = nb of var; (after the loop)
              
      // accumulated factor error
      double accumulated_factor_error = 0;

      // principle: loop the factors, write the 'factors' in the logger
      sam_tuples::for_each_in_tuple(
          this->all_factors_tuple_,
          [&Xmap,&json_graph, this, &SigmaCov,&already_processed_keys,&accumulated_factor_error](auto& vect_of_f, auto I)
          {
            for (auto& factor : vect_of_f)
            {
              PROFILE_SCOPE( (factor.factor_id.c_str()) , sam_utils::JSONLogger::Instance() );
         
              sam_tuples::for_each_in_tuple(factor.keys_set,
                                        [this,&Xmap,&SigmaCov, &accumulated_factor_error, &already_processed_keys,&json_graph](auto& kcm, auto kcmIdx)
                        {
                            PROFILE_SCOPE(kcm.key_id.c_str(), sam_utils::JSONLogger::Instance() );
                            using kcm_keymeta_t = typename std::remove_const_t<std::decay_t<decltype(kcm)>>::KeyMeta_t;
                            constexpr std::size_t kN = kcm_keymeta_t::kN;
                            // check if key_id already processed
                            auto searched = already_processed_keys.find( kcm.key_id );
                            if (searched == already_processed_keys.end())
                            {
                                // pick this key marginal map from the big Xmap vector and the big Cov matrix
                                Eigen::Vector<double,kN> xmap_marg = Xmap.block<kN,1>(this->bookkeeper_.getKeyInfos(kcm.key_id).sysidx, 0);
                                Eigen::Matrix<double,kN,kN> cov_marg = SigmaCov.block<kN,kN>(
                                              this->bookkeeper_.getKeyInfos(kcm.key_id).sysidx
                                            , this->bookkeeper_.getKeyInfos(kcm.key_id).sysidx);

                                // update the marginal container
                                Marginal<kcm_keymeta_t> marg(xmap_marg,cov_marg); // OPTIMIZE: perf fwd (carefully, marg is needed in the write)
                                this->all_marginals_.insertt(kcm.key_id,marg);
                                // update the linearization point .  NOTE: must also be done in the 'else' branch 
                                if constexpr (isSystFullyLinear)
                                  kcm.update_key_mean(xmap_marg); // FIX:
                                else
                                  kcm.set_key_mean(xmap_marg); // FIX:

                                
                                // save the marginal in the json graph
                                Json::Value json_marginal = write_marginal(marg, kcm.key_id);
                                json_graph["marginals"].append(json_marginal);
                                // finally, save the fact that this key_id marginal has been processed 
                                already_processed_keys.insert(kcm.key_id);
                            }
                            else
                            {
                                // update the linearization point
                                Eigen::Vector<double,kN> xmap_marg = 
                                  this->all_marginals_.template findt<kcm_keymeta_t>(kcm.key_id).value().mean;
                                if constexpr (isSystFullyLinear)
                                  kcm.update_key_mean(xmap_marg); // FIX: 
                                else
                                  kcm.set_key_mean(xmap_marg); // FIX:
                            }
                        });
              // compute factor error (uses the lin point)
              factor.norm_at_lin_point =factor.compute_lin_point_factor_norm();
              accumulated_factor_error += factor.norm_at_lin_point;
              //------------------------------------------------------------------//
              //                  Json graph: write  the factor                   //
              //------------------------------------------------------------------//
              Json::Value json_factor = write_factor(factor);
              json_graph["factors"].append(json_factor);
            }
          });
      {
        PROFILE_SCOPE("join log results", sam_utils::JSONLogger::Instance() );
        this->bookkeeper_.set_syst_residual_error(accumulated_factor_error);
        json_graph["header"] = write_header(this->bookkeeper_.getSystemInfos());
        logger.writeGraph(json_graph);
      }
    }

    Json::Value write_header(const SystemInfo & sysinfo)
    {
      Json::Value json_header;
      json_header["robot_id"] = "A"; // TODO:
      json_header["seq"] = 0; // TODO:
      json_header["base_unit"] = 0.15;
      json_header["QRerror"] = sysinfo.residual_error;
      json_header["Rnnz"] = sysinfo.Rnnz;
      json_header["Hnnz"] = sysinfo.Hnnz;
      json_header["residual_error"] = sysinfo.residual_error;
      // TODO: variable order  :  "variable_order"
      return json_header;
    }

    // TODO: move this method as a friend of the marginal base
    template <typename MG>
      Json::Value write_marginal(const MG marginal_ptr,const std::string & var_id)
    {
      Json::Value json_marginal;
       json_marginal["var_id"] = var_id; 
       json_marginal["category"] = MG::element_type::KeyMeta_t::kKeyName ;
       // json_marginal["kind"] = "2D" ;
       Json::Value json_mean;
       for (std::size_t i = 0; i< MG::element_type::KeyMeta_t::components.size(); i++)
       {
         json_mean[MG::element_type::KeyMeta_t::components[i]] = *(marginal_ptr->mean_ptr)(i,0);
       }
       json_marginal["mean"] = json_mean ; 
        auto [sig,rot] = marginal_ptr->get_visual_2d_covariance();
       json_marginal["covariance"]["sigma"].append(sig[0]); 
       json_marginal["covariance"]["sigma"].append(sig[1]);
       json_marginal["covariance"]["rot"] = rot;

       return json_marginal;
    }


    // TODO: move this method as a friend of the factor base
    template <typename FT>
    Json::Value write_factor(const FT & factor) const
    {
              Json::Value json_factor;
              json_factor["factor_id"] = factor.factor_id;
              json_factor["type"]      = FT::kFactorLabel;
              // write the vars_id
              Json::Value json_factor_vars_id;
              sam_tuples::for_each_in_const_tuple(factor.keys_set,
                                            [&json_factor_vars_id](const auto& keycc, auto I)
                                            { json_factor_vars_id.append(keycc.key_id); });
              json_factor["vars_id"] = json_factor_vars_id;
              json_factor["MAPerror"] = factor.norm_at_lin_point;
              // json_factor["measurement"] = factor_type;
              // append in the json 'graph > factors'
              return json_factor;
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


    std::tuple<std::vector<Eigen::Triplet<double>> , Eigen::VectorXd> construct_A_b_from_factors()
    {
      std::vector<Eigen::Triplet<double>> sparseA_triplets;
      Eigen::VectorXd b(this->bookkeeper_.getSystemInfos().aggr_dim_mes);
      uint64_t line_counter = 0;
      // TODO: declare sparseA_triplets and b here
      sam_tuples::for_each_in_tuple(
        this->all_factors_tuple_,
        [this,&sparseA_triplets,&b,&line_counter](auto& vect_of_f, auto I) // NOTE: I unusable (not constexpr-able)
        {
          using factor_t =typename std::decay_t<decltype(vect_of_f)>::value_type;
#if ENABLE_DEBUG_TRACE
          std::cout << "### Looping over factors of type " << factor_t::kFactorLabel
                    << "\n";
#endif
          for(auto & factor : vect_of_f)
          {
            PROFILE_SCOPE( factor.factor_id.c_str() ,sam_utils::JSONLogger::Instance());
            //------------------------------------------------------------------//
            //                        refactoring notes                         //
            //------------------------------------------------------------------//
            // declare Ai,bi
            typename factor_t::measure_vect_t b_i;
            std::vector<Eigen::Triplet<double>> Ai_triplets; 
            Ai_triplets.reserve(factor_t::kN*factor_t::kM);
            if constexpr (isSystFullyLinear)
            {
              b_i = factor.rosie;
              // Ai_triplets, TODO: do a returned value pattern for clarity
              // compute \rho*\frac{\partial h}{\partial x} and fill the triplets with the correct
              // indexes of the wider system
              std::apply(
                  [this, &Ai_triplets, line_counter](auto&&... keycc)
                  { (this->compute_partialA_and_fill_triplet(keycc, Ai_triplets, line_counter), ...); },
                  factor.keys_set);
            }
            else
            {
              // NOTE: this assumes lin points already been set elsewhere
              b_i = factor.compute_b_nl(); // or without mean_tup as factor is supposed to have it inside
              // TODO: do a returned pattern Ai_triplets = ...  (presumably the same than the std::apply above)
              // logic is done internally
              std::apply(
                  [this, &Ai_triplets, line_counter](auto&&... keycc)
                  { (this->compute_partialA_and_fill_triplet(keycc, Ai_triplets, line_counter), ...); },
                  factor.keys_set);
            }
            // Append b_i into b
            constexpr int mesdim                = factor_t::kM;
            b.block<mesdim, 1>(line_counter, 0) = b_i;
            // Append Ai_triplets in sparseA_triplets
            sparseA_triplets.insert(std::end(sparseA_triplets),std::begin(Ai_triplets),std::end(Ai_triplets));
            // increment the line number by as many lines filled here
            line_counter += mesdim; // NOTE: make sure it is at the end of the loop
#if ENABLE_DEBUG_TRACE
            std::cout << " bi: \n" << b_i << "\n";
#endif
          }
        }
      );
      return {sparseA_triplets,b};
    }

    /**
     * @brief compute a part of a factor's A matrix, only the columns associated
     * with one key are computed. Note that if factor is univariate, then partA
     * = A
     *
     * @param keycc
     * @param triplets
     * @param line_counter
     */
    template <typename KeyContextConduct>
    void compute_partialA_and_fill_triplet(KeyContextConduct&                   keycc,
                                           std::vector<Eigen::Triplet<double>>& triplets,
                                           int                                  line_counter)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      // compute partial A (automatically deals with NL or linear case)
      auto partA = keycc.compute_part_A();

      // The following code index the partA matrix into the system's indexes

      // get the col in systA (the big A of the system)
      int colInBigA = this->bookkeeper_.getKeyInfos(keycc.key_id).sysidx;
      // reshape the partA matrix, so that it is easier to loop. (column major)
      auto partA1d = partA.reshaped();
      // now, loop and write
      int mesdim = partA.rows();
      for (int i = 0; i < partA.cols() * mesdim; i++)
      {
        // row in big A is easier, just wrap the i index & add the line
        // counter
        int row = line_counter + (i % mesdim);
        // col idx in big A : we know
        int col = colInBigA + i / mesdim;
        triplets.emplace_back(row, col, partA1d[i]);
      }
    }

    /**
     * @brief fill A & b system matrices
     *
     * @param dim_mes
     * @param dim_keys
     * @param nnz
     *
     * @return
     */
    std::tuple<Eigen::SparseMatrix<double>, Eigen::VectorXd>
        assemble_system(uint dim_mes, uint dim_keys, uint nnz)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      ;
#if ENABLE_DEBUG_TRACE
      std::cout << "starting filling system of size M= " << dim_mes << " , N= " << dim_keys
                << " , NNZ= " << nnz << '\n';
#endif
      // declare matrix A and b
      Eigen::SparseMatrix<double> A(dim_mes,
                                    dim_keys);   // colum-major (default)
      // triplets to fill the matrix A
      std::vector<Eigen::Triplet<double>> triplets;
      triplets.reserve(nnz);
      // loop over all factors
      // fill in the triplets and the rhs
      // TODO: is there a way to use incdex_sequence rather than the if constexpr
      // recursive pattern, the goal is to handle return values more gracefully
      auto [A_triplets, b] = this->construct_A_b_from_factors();
      A.setFromTriplets(A_triplets.begin(), A_triplets.end());
      return {A, b};
    }

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
    template <std::size_t I = 0, typename FT>
    void place_factor_in_container(const std::string&                          factor_id,
                                   const typename FT::measure_vect_t&          mes_vect,
                                   const typename FT::measure_cov_t&           measure_cov,
                                   const std::array<std::string, FT::kNbKeys>& keys_id)
    {
      // beginning of static recursion (expanded at compile time)
      if constexpr (I == S_)
        return;
      else
      {
        // if this is the type we are looking for, emplace back in
        if constexpr (std::is_same_v<FT, factor_type_in_tuple_t<I>>)
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
                            -> typename FT::tuple_of_opt_part_state_t
              {
                return 
                { 
                  this->all_marginals_
                  .template find_mean_v<typename std::tuple_element_t<J, typename FT::KeysSet_t>::KeyMeta_t>(keys_id[J])
                ... 
                };
              }
            );

          // It is probable that the above tuple contains std::nullopt.
          // Attempt to guess the full init point for this factor by using the measurement if necessary.
          // If we don't have enough data to fill in the blank, then `opt_tuple_of_init_point = std::nullopt`
          std::optional<typename FT::tuple_of_part_state_t> opt_tuple_of_init_point 
            = FT::guess_init_key_points(tuple_of_opt_means_ptr,mes_vect); // NOTE: heap allocation (make_shared)

          if (opt_tuple_of_init_point.has_value()) // we have all the means for the key
          {
            // Add new init point in marginal container
            sam_tuples::for_each_in_const_tuple(tuple_of_opt_means_ptr,
            [this,&keys_id](const auto & opt_mean_ptr, auto J)
            {
              // if the mean was not found ante-previously, insert it using the guesser result
              if (!opt_mean_ptr.has_value())
              {
                // isolate the shared ptr to the guessed init point
                auto guessed_init_point_ptr = std::get<J>(opt_tuple_of_init_point.value());
                // make a new marginal from the guessed init point
                using marginal_type = typename  std::tuple_element<J,typename marginals_t::marginals_containers_t>::mapped_type::element_type; 
                std::shared_ptr<marginal_type> new_marginal_ptr = std::make_shared<marginal_type>(guessed_init_point_ptr); // NOTE: HEAP allocation  (make_shared)
                // insert the (shared ptr) marginal we just created in the system's marginal container
                this->all_marginals_.
                  template insert_in_marginal_container<marginal_type>
                  (keys_id[J],new_marginal_ptr);
                // TODO: update the bookkeeper ? I think its already done ( CHECK: )
              }
            });

            // pass the tuple of ptr of the key means to the factor ctor 
            // TODO: URGENT:
            std::get<I>(this->all_factors_tuple_)
                  .emplace_back(factor_id, mes_vect, measure_cov, keys_id, opt_tuple_of_init_point.value());
#if ENABLE_DEBUG_TRACE
            std::cout << "\t\t:: Factor " << factor_id << " properly integrated in NL system.\n";
#endif
          }
          else
          {
            // TODO: FEATURE: emplace factor in a staging container if 
            // TODO: more detail (which key.s failed etc..)
            throw std::runtime_error("Unable to determine all init points for this factor");
          }
              
// Debug consistency check of everything
#if ENABLE_RUNTIME_CONSISTENCY_CHECKS
          // 1. checking if the bookkeeper is consistent with itself (systemInfo
          // vs whats on the std::maps)
          assert(this->bookkeeper_.are_dimensions_consistent());
          // 2. checking if the bookkeeper is consistent with the tuples of
          // vector holding the factors
          assert(this->is_system_consistent());
#endif
        }
        // recursion :  compile time call
        place_factor_in_container<I + 1, FT>(factor_id, mes_vect, measure_cov, keys_id);
      }
    }

    template <typename FT>
    void add_keys_to_bookkeeper(const std::array<std::string, FT::kNbKeys>& keys_id,
                                const std::string&                          factor_id)
    {
      add_keys_to_bookkeeper_impl<FT>(factor_id, keys_id, std::make_index_sequence<FT::kNbKeys> {});
    }

    template <typename FT, std::size_t... I>
    void add_keys_to_bookkeeper_impl(const std::string&                          factor_id,
                                     const std::array<std::string, FT::kNbKeys>& keys_id,
                                     std::index_sequence<I...>)
    {
      (dosomething(factor_id, keys_id[I], std::tuple_element_t<I, typename FT::KeysSet_t>::kN),
       ...);
    }
    void dosomething(const std::string& factor_id,
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
    std::tuple<Eigen::VectorXd,double,double> solveQR(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b)
    // TODO: add a solverOpts variable: check rank or not, check success
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
      if (solver.rank() < A.cols())
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
      // residual error
      auto residual_error = solver.matrixQ().transpose() * b;
#if ENABLE_DEBUG_TRACE
      {
        PROFILE_SCOPE("print console",sam_utils::JSONLogger::Instance());
        std::cout << "### Syst solver : residual value: " << residual_error.norm() << "\n";
        std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
        std::cout << "### Syst solver :  nnz in square root : " << solver.matrixR().nonZeros()
                  << " (from " << A.nonZeros() << ") in Hessian."
                  << "\n";
        if ( Eigen::MatrixXd(solver.matrixR()).rows() < 15 )
        {
          std::cout << "### Syst solver : matrix R : \n" << Eigen::MatrixXd(solver.matrixR()) << '\n';
        }
      }
#endif
      return {map,std::pow(residual_error.norm(),2),solver.matrixR().nonZeros()};
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
    template <size_t I>
    using factor_type_in_tuple_t =
        typename std::tuple_element<I, decltype(all_factors_tuple_)>::type::value_type;
  };

};   // namespace SAM
#endif
