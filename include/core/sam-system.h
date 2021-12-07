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

    // TODO: isSystFullyLinear (non type bool template parameter)
    static constexpr const bool isSystFullyLinear = FACTOR_T::isLinear && ( FACTORS_Ts::isLinear && ... );

    SamSystem(const std::string & agent_id):agent_id(agent_id) { PROFILE_FUNCTION(sam_utils::JSONLogger::Instance()); }

    std::tuple<Eigen::MatrixXd,double> compute_covariance(const Eigen::SparseMatrix<double> & A)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      auto At = Eigen::MatrixXd(A.transpose());
      auto H = At*A;
      return {H.inverse(),H.nonZeros()};
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


      // recursively find, at compile time, the corresponding container (amongst
      // the ones in the tuple) to emplace back the factor FT
      place_factor_in_container<0, FT>(factor_id, mes_vect, measure_cov, keys_id);
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
            // there are several loops in the factor kcc, I consider thats ok, micro-optimizing it would make readability more difficult than it already is 
            for (auto& factor : vect_of_f)
            {
              PROFILE_SCOPE( (factor.factor_id.c_str()) , sam_utils::JSONLogger::Instance() );
              // std::apply -> for each kcc of that factor, if unprocessed, update the marginal with xmap & cov 
              //               add the key_id in the process_keys set
              //------------------------------------------------------------------//
              //           From the MAP vector Xmap, and optionally the           //
              //                          associated cov,                         //
              //            Write the marginals, (amounts to dispatch             //
              //           subcomponents of xmap/cov to our structure)            //
              //------------------------------------------------------------------//

              // TODO: URGENT: xmap -> marginalcontainer ok, but fill also the linearization in each factor.kcm
              // auto mean_tup = sam_tuples::reduce_tuple_variadically(factor.keys_set, 
              //   [this]<std::size_t... II>(const auto & tup_el, std::index_sequence<II...>)
              //   {
              //       auto returned_tup = std::make_tuple( 
              //   this->all_marginals_.template find<typename std::remove_const_t<std::decay_t<decltype(std::get<II>(tup_el))>>::KeyMeta_t>
              //   (std::get<II>(tup_el).key_id).value().mean
              //    ...
              //   );
              //   static_assert(sizeof...(II) == std::tuple_size_v<decltype(returned_tup)>);
              //   return returned_tup;
              //   }
         
              sam_tuples::for_each_in_const_tuple(factor.keys_set,
                                        [this,&Xmap,&SigmaCov, &already_processed_keys,&json_graph](const auto& kcc, auto kccIdx)
                        {
                            // check if key_id already processed
                            auto search = already_processed_keys.find( kcc.key_id );
                            if (search == already_processed_keys.end())
                            {
                                PROFILE_SCOPE(kcc.key_id.c_str(), sam_utils::JSONLogger::Instance() );
                                // // fill the marginal
                                using kcc_keymeta_t = typename std::remove_const_t<std::decay_t<decltype(kcc)>>::KeyMeta_t;
                                // std::cout << kcc_keymeta_t::kN << '\n';
                                constexpr std::size_t kN = kcc_keymeta_t::kN;
                                Eigen::Vector<double,kN> xmap_marg = Xmap.block<kN,1>(this->bookkeeper_.getKeyInfos(kcc.key_id).sysidx, 0);
                                Eigen::Matrix<double,kN,kN> cov_marg = SigmaCov.block<kN,kN>(
                                            this->bookkeeper_.getKeyInfos(kcc.key_id).sysidx
                                          , this->bookkeeper_.getKeyInfos(kcc.key_id).sysidx);
                                // std::cout << std::tuple_element_t<kccIdx,typename decltype(factor)::KeysSet_t>::kN << '\n';
                                // this->all_marginals_.insert<typename std::decay_t<decltype(kcc)>::KeyMeta_t>(kcc.key_id,xmap_marg,cov_marg);

                                Marginal<kcc_keymeta_t> marg(xmap_marg,cov_marg); // OPTIMIZE: (carefully, marg is needed in the write)
                                this->all_marginals_.insertt(kcc.key_id,marg);
                                
                                // save the marginal in the json graph
                                Json::Value json_marginal = write_marginal(marg, kcc.key_id);
                                json_graph["marginals"].append(json_marginal);
                                // finally 
                                already_processed_keys.insert(kcc.key_id);
                            }
                        });
              //------------------------------------------------------------------//
              //             compute factor error, accumulate errors              //
              //------------------------------------------------------------------//
              // loop the kcc of a factor
              std::apply(
                  [this, &Xmap, &accumulated_factor_error, &factor](auto... kcc) // TODO: remove Xmap process, replace by marginalcontainer element (by copy probably)
                  {
                    PROFILE_SCOPE("compute error", sam_utils::JSONLogger::Instance() );
                    // get the global idx
                    // std::size_t globalIdxOfKey = ;
                    // now we know how extract a subcomponent of xmap
            //
                    // WARNING: may revisit for NL, or move that line in the factors jurisdiction
                    auto innovation = ((kcc.compute_part_A()
                                        * Xmap.block(this->bookkeeper_.getKeyInfos(kcc.key_id).sysidx, // TODO: replace by a call to marginal container
                                                     0,
                                                     decltype(kcc)::kN,
                                                     1))
                                       + ...)
                                      - factor.rosie; // TODO: adapt for NL
                    factor.error = std::pow(innovation.norm(), 2);
                    // this factor norm2 squared is added to the total error
                    accumulated_factor_error += factor.error;
                  },
            factor.keys_set);
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
      // TODO: cout in std output (if enable debug trace flag is on)
    }

    Json::Value write_header(const SystemInfo & sysinfo)
    {
      // "header": {
      //   "robot_id": "A",
      //   "base_unit": 0.15, // deprecated
      //   "seq": 0,          // TODO:  rule on this one
      //   "variable_order": ["x0", "x3", "x2", "x4", "l2", "x1", "l1"], // optional
      //   "residual_error": 1654
      // },
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

    // TODO: move this method as a friend of the factor base
    template <typename MG>
      Json::Value write_marginal(const MG& marginal,const std::string & var_id)
    {
      Json::Value json_marginal;
       json_marginal["var_id"] = var_id; 
       json_marginal["category"] = MG::KeyMeta_t::kKeyName ;
       // json_marginal["kind"] = "2D" ;
          Json::Value json_mean;
        for (std::size_t i = 0; i< MG::KeyMeta_t::components.size(); i++)
        {
          json_mean[MG::KeyMeta_t::components[i]] = marginal.mean(i,0);
        }
       json_marginal["mean"] = json_mean ; 
        auto [sig,rot] = marginal.get_visual_2d_covariance();
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
              // json_factor["type"]      = std::decay_t<decltype(vect_of_f[0])>::kFactorLabel;
              json_factor["type"]      = FT::kFactorLabel;   // NOTE: is good ?
              // write the vars_id
              Json::Value json_factor_vars_id;
              sam_tuples::for_each_in_const_tuple(factor.keys_set,
                                            [&json_factor_vars_id](const auto& keycc, auto I)
                                            { json_factor_vars_id.append(keycc.key_id); });
              json_factor["vars_id"] = json_factor_vars_id;
              // json_factor["MAPerror"] = factor_type;
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


    /**
     * @brief loop over factors and get the data to fill the system (sparse
     * matrix A through list of triplets and rhs vector)
     *
     * @tparam I
     * @param triplets
     * @param b
     * @param line_counter
     */
    // template <std::size_t I = 0>       // TODO: refactor, use std::apply, or my custom for_each_tuple. It will permits to
    void loop_over_factors(std::vector<Eigen::Triplet<double>>& sparseA_triplets, // TODO: move as returned value
                           Eigen::VectorXd&                     b, // TODO: move as returned value
                           int                                  line_counter = 0)
    {
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
            // //------------------------------------------------------------------//
            // //                      end refactoring notes                       //
            // //------------------------------------------------------------------//
            // //------------------------------------------------------------------//
            // //                           compute b_i                            //
            // //------------------------------------------------------------------//
            // typename factor_t::measure_vect_t factorb;
            // if constexpr ( isSystFullyLinear )
            // {
            //     factorb = factor.rosie;
            // }
            // else
            // {
            //   auto mean_tup = sam_tuples::reduce_tuple_variadically(factor.keys_set, 
            //     [this]<std::size_t... II>(const auto & tup_el, std::index_sequence<II...>)
            //     {
            //         auto returned_tup = std::make_tuple( 
            //     this->all_marginals_.template find<typename std::remove_const_t<std::decay_t<decltype(std::get<II>(tup_el))>>::KeyMeta_t>
            //     (std::get<II>(tup_el).key_id).value().mean
            //      ...
            //     );
            //     static_assert(sizeof...(II) == std::tuple_size_v<decltype(returned_tup)>);
            //     return returned_tup;
            //     }
            //   );
            //   factorb = factor.compute_b(mean_tup);
            // }
#if ENABLE_DEBUG_TRACE
            std::cout << " bi: \n" << b_i << "\n";
#endif
            // //------------------------------------------------------------------//
            // //                      integrates b_i into b                       //
            // //------------------------------------------------------------------//
            // // Fill the vector b of that factor in the system vector b
            // constexpr int mesdim                = factor_t::kM;
            // b.block<mesdim, 1>(line_counter, 0) = factorb;
            // //------------------------------------------------------------------//
            // //                            compute Ai                            //
            // //------------------------------------------------------------------//
            // // suckless method: consider the A matrix of the factor (not the
            // // system) iterate over the keycontext, compute A, and fill the
            // // triplet list
            // std::apply(
            //     [this, &sparseA_triplets, line_counter](auto&&... keycc)
            //     { ((this->compute_partialA_and_fill_triplet(keycc, sparseA_triplets, line_counter)), ...); },
            //     factor.keys_set);

            // increment the line number by as many lines filled here
            line_counter += mesdim; // NOTE: make sure it is at the end of the loop
          }
        }
      );
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
      // compute partial A (partial = only a block of the A of the factor)
      auto partA = keycc.compute_part_A(); // TODO: change for NL pb: auto partA = keycc.compute_part_A<isSystFullyLinear>(); 
      // TODO: NL -> rho*H (non const) , L in <false> -> roach (const), L in <true> -> roach (const)

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
      Eigen::VectorXd             b(dim_mes);
      // triplets to fill the matrix A
      std::vector<Eigen::Triplet<double>> triplets;
      triplets.reserve(nnz);
      // loop over all factors
      // fill in the triplets and the rhs
      // TODO: is there a way to use incdex_sequence rather than the if constexpr
      // recursive pattern, the goal is to handle return values more gracefully
      this->loop_over_factors(triplets, b);
      A.setFromTriplets(triplets.begin(), triplets.end());
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

          // TODO: initialize lin point

          std::get<I>(this->all_factors_tuple_)
              .emplace_back(factor_id, mes_vect, measure_cov, keys_id);
#if ENABLE_DEBUG_TRACE
          std::cout << "\t\t:: Factor " << factor_id << " properly integrated in system.\n";
#endif

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
