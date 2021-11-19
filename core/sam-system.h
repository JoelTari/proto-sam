#ifndef SAM_SYSTEM_H_
#define SAM_SYSTEM_H_

#include "anchor.hpp"
#include "bookkeeper.h"
#include "config.h"
#include "utils.h"

#include <functional>
// #include "definitions.h"

// #include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
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
    SamSystem() { PROFILE_FUNCTION(sam_utils::JSONLogger::Instance()); }

    template <typename FT>
    void
        register_new_factor(const std::string&                 factor_id,
                            const typename FT::measure_vect_t& mes_vect,
                            const typename FT::measure_cov_t&  measure_cov,
                            const std::array<std::string, FT::kNbKeys>& keys_id)
    {
      static_assert(
          std::is_same_v<FT,
                         FACTOR_T> || (std::is_same_v<FT, FACTORS_Ts> || ...),
          "This type of factor doesnt exist ");
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      // check if factor id exists already
      // TODO: consistent management failure (throw ? return value false ?
      // std::optional ?)
      if (this->bookkeeper_.factor_id_exists(factor_id))
        throw std::runtime_error("Factor id already exists");


      // recursively find, at compile time, the corresponding container (amongst
      // the ones in the tuple) to emplace back the factor FT
      place_factor_in_container<0, FT>(factor_id,
                                       mes_vect,
                                       measure_cov,
                                       keys_id);
    }

    double compute_factor_system_residual(const Eigen::VectorXd & xmap)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      unwrap_system_residual(xmap,std::make_index_sequence<S_>{});
    }

    template <std::size_t...I>
    double unwrap_system_residual(const Eigen::VectorXd & xmap,std::index_sequence<I...>)
    {
      return ( sum_errors<I>(xmap) + ... ); // one term of each type of factor that represent the sum of all its vector
    }

    // sum all factor errors in a vector of factors
    template<size_t I>
    double sum_errors(const Eigen::VectorXd & xmap)
    {
      // access the number of keys this type of factor hold
       constexpr int NbKeys = factor_type_in_tuple_t<I>::kNbKeys;
       double sum=0;
       for (const auto & factor : std::get<I>(all_factors_tuple_))
       {
         // need to compute tailored state vector
         // each factor must only receive an ordered subset of xmap
         // 1. get keys of factor
         // 2. get global idx of those keys
         // 3. make the subblock of xmap
         // 4. pass the argument
         //  xmap.block< FTNkey ,1 >(globalIdxofKey,0)
         // auto keys = factor.keys_set;
         // for (const auto & keycc : factor.keys_set)
         //     std::cout << "bip boop\n";
         // sum+=factor.compute_error(xmap);
         // std::apply([](auto ...x){std::make_tuple(some_function(x)...);} , the_tuple);
       }
       return sum;
    }


    void smooth_and_map()
    // TODO: add a solverOpts variable: check rank or not, check success, write
    // bookkeeper etc..
    {
      // scoped timer
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      ;

      SystemInfo system_infos = this->bookkeeper_.getSystemInfos();
      int        M            = system_infos.aggr_dim_mes;
      int        N            = system_infos.aggr_dim_keys;

      uint nnz = this->bookkeeper_.getSystemInfos().nnz;

      // the big steps: fill the system (sparse matrix A and rhs vector b)
      auto [A, b] = fill_system(M, N, nnz);
      // and solve the system
      auto Xmap = solve_system(A, b);
#if ENABLE_DEBUG_TRACE
      std::cout << "#### Syst: A computed :\n" << Eigen::MatrixXd(A) << "\n\n";
      // std::cout << "#### Syst: R computed :\n" << Eigen::MatrixXd(A) <<
      // "\n\n";
      std::cout << "#### Syst: b computed :\n" << b << "\n";
      std::cout << "#### Syst: MAP computed :\n" << Xmap << '\n';
#endif
      
      double factor_error = compute_factor_system_residual(Xmap);

      // CONTINUE: HERE
      // keep the records: update the bookkeeper
      write_factor_graph(sam_utils::JSONLogger::Instance());
    }

    /**
     * @brief With current graph, containing  the last recorded results, from
     * the bookkeeper WARNING: perhaps use it only in the bookkeeper, or another
     * new class (inverse dependency) WARNING: single responsibility principle
     * is broken
     */
    void write_factor_graph(sam_utils::JSONLogger& logger)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      Json::Value json_graph;

      // TODO: fill a 'graph' field in the json logger
      // principle: loop the factors, write the 'factors' in the logger 
      for_each_in_tuple(this->all_factors_tuple_, 
        [&json_graph](const auto & vect_of_f, auto I)
        {
          std::string factor_type = std::decay_t<decltype(vect_of_f[0])>::kFactorLabel;
          for (const auto & factor : vect_of_f)
          {
            Json::Value json_factor;
            json_factor["factor_id"] = factor.factor_id;
            json_factor["type"] = factor_type;   // CONTINUE: HERE
            // write the vars_id
            Json::Value json_factor_vars_id;
            for_each_in_tuple(factor.keys_set,[&json_factor_vars_id](const auto & keycc, auto I)
              {
                  json_factor_vars_id.append(keycc.key_id);
              }
            );
            json_factor["vars_id"] = json_factor_vars_id;
            // json_factor["MAPerror"] = factor_type;
            // json_factor["measurement"] = factor_type;
            // append in the json 'graph > factors'
            json_graph["factors"].append(json_factor);
          }
        }
      );
      // Access the keys of each of those factor
      // loop the keys in the bookkeeper to write the 'marginals' in the logger
      // also use the cov matrix to extract the marginal covariance 
      // CONTINUE: TODO: the keys -> mean covariance var_id category kind

      logger.writeGraph(json_graph);
      // TODO: cout in std output (if enable debug trace flag is on)
    }

#if ENABLE_DEBUG_TRACE
    /**
     * @brief dummy iteration over tuple of std::vector
     */
    template <size_t I = 0>
    void IterFactorInfos()
    {
      if constexpr (I == S_)
        return;
      else
      {
        std::cout << factor_type_in_tuple_t<I>::kFactorLabel
                  << "  -- Number of elements registered : "
                  << std::get<I>(this->all_factors_tuple_).size() << "\n";

        // if this collection is not empty, print the details of the factors
        if (std::get<I>(this->all_factors_tuple_).size() > 0)
        {
          for (const auto& factor : std::get<I>(this->all_factors_tuple_))
          {
            std::cout << "\t";
            factor_print(factor);
            std::cout << "\n";
          }
        }
        // recursion :  compile time call
        IterFactorInfos<I + 1>();
      }
    }
#endif

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

    // there's at least one factor, the rest are expanded
    std::tuple<std::vector<FACTOR_T>, std::vector<FACTORS_Ts>...>
        all_factors_tuple_;

    /**
     * @brief how many different types of factor there are
     */
    constexpr static const size_t S_
        = std::tuple_size<decltype(all_factors_tuple_)>::value;


    /**
     * @brief loop over factors and get the data to fill the system (sparse
     * matrix A through list of triplets and rhs vector)
     *
     * @tparam I
     * @param triplets
     * @param b
     * @param line_counter
     */
    template <std::size_t I = 0>
    void loop_over_factors(std::vector<Eigen::Triplet<double>>& triplets,
                           Eigen::VectorXd&                     b,
                           int line_counter = 0)
    {
      if constexpr (I == S_)
        return;
      else
      {
#if ENABLE_DEBUG_TRACE
        std::cout << "### Looping over factors of type "
                  << factor_type_in_tuple_t<I>::kFactorLabel << "\n";
#endif
        for (auto& factor :
             std::get<I>(this->all_factors_tuple_))   // may not be constant
        {
          // update the factor's A and b matrices (new lin point)
          // if constexpr nonlinear factor  =>  set lin point (given by
          // bookkeeper)
          // auto [factorA, factorb] = factor.compute_A_b();
          auto factorb = factor.compute_rosie();   // TODO: change for NL
#if ENABLE_DEBUG_TRACE
          std::cout << " b: \n" << factorb << "\n";
#endif

          // easy part : fill the rhs b from factor.b
          constexpr int mesdim                = factor_type_in_tuple_t<I>::kM;
          b.block<mesdim, 1>(line_counter, 0) = factorb;
          // suckless method: consider the A matrix of the factor (not the
          // system) iterate over the keycontext, compute A, and fill the
          // triplet list
          std::apply(
              [this, &triplets, line_counter](auto&&... keycc)
              {
                ((this->compute_partialA_and_fill_triplet(keycc,
                                                          triplets,
                                                          line_counter)),
                 ...);
              },
              factor.keys_set);

          // increment the line number by as many lines filled here
          line_counter += mesdim;
        }
        // compile-time recursion
        loop_over_factors<I + 1>(triplets, b, line_counter);
      }
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
    void compute_partialA_and_fill_triplet(
        KeyContextConduct&                   keycc,
        std::vector<Eigen::Triplet<double>>& triplets,
        int                                  line_counter)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      // compute partial A (partial = only a block of the A of the factor)
      auto partA = keycc.compute_part_A();
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
        fill_system(uint dim_mes, uint dim_keys, uint nnz)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      ;
#if ENABLE_DEBUG_TRACE
      std::cout << "starting filling system of size M= " << dim_mes
                << " , N= " << dim_keys << " , NNZ= " << nnz << '\n';
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
    void place_factor_in_container(
        const std::string&                          factor_id,
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
          // During this step, the factor is garanted to be pushed in the system
          // so we should update bookkeeper
          //  1. for each key
          //      - check if the keys exists if key doesn't exist, add it (and
          //      it's meta)
          //      - add this factor id to the keyinfo list
          //  2. add an element in FactorInfo

          // for each key of this factor.
          // std::apply(,FT::KeysSet);
          // for (std::size_t i = 0; i < keys_id.size(); i++)
          // {
          //   // check if the key exists, if it doesn't, we will catch
          //   // TODO: a standard if/else might be more desirable (or
          //   std::optional) try
          //   {
          //     this->bookkeeper_.getKeyInfos(keys_id[i]);
          //   }
          //   catch (int e)
          //   {
          //     // add the key, the variable size is accessed via the factor
          //     Meta this->bookkeeper_.add_key(keys_id[i], FT::KeysSet_t::kNb);
          //     // BUG:
          //   }
          //   // each key has a list of factors_id that it is connected, so add
          //   // this factor_id to it
          //   this->bookkeeper_.add_factor_id_to_key(keys_id[i], factor_id);
          // }
          add_keys_to_bookkeeper<FT>(keys_id, factor_id);
          // add the factor_id with its infos in the bookkeeper
          // last argument is a conversion from std::array to std::vector
          this->bookkeeper_.add_factor(factor_id,
                                       FT::kN,
                                       FT::kM,
                                       {keys_id.begin(), keys_id.end()});


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
        place_factor_in_container<I + 1, FT>(factor_id,
                                             mes_vect,
                                             measure_cov,
                                             keys_id);
      }
    }

    template <typename FT>
    void add_keys_to_bookkeeper(
        const std::array<std::string, FT::kNbKeys>& keys_id,
        const std::string&                          factor_id)
    {
      add_keys_to_bookkeeper_impl<FT>(factor_id,
                                      keys_id,
                                      std::make_index_sequence<FT::kNbKeys> {});
    }

    template <typename FT, std::size_t... I>
    void add_keys_to_bookkeeper_impl(
        const std::string&                          factor_id,
        const std::array<std::string, FT::kNbKeys>& keys_id,
        std::index_sequence<I...>)
    {
      (dosomething(factor_id,
                   keys_id[I],
                   std::tuple_element_t<I, typename FT::KeysSet_t>::kN),
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
    Eigen::VectorXd solve_system(const Eigen::SparseMatrix<double>& A,
                                 const Eigen::VectorXd&             b)
    // TODO: add a solverOpts variable: check rank or not, check success
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      ;
      // solver
      Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
          solver;
      // MAP
      solver.compute(A);

      // rank check: not considered a consistency check
      if (solver.rank() < A.cols()) throw "RANK DEFICIENT PROBLEM";
      Eigen::VectorXd map = solver.solve(b);
      // residual error
#if ENABLE_DEBUG_TRACE
      auto residual_error = solver.matrixQ().transpose() * b;
      std::cout << "### Syst solver : residual value: " << residual_error.norm()
                << "\n";
      std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS")
                << "\n";
      std::cout << "### Syst solver :  nnz in square root : "
                << solver.matrixR().nonZeros() << " (from " << A.nonZeros()
                << ") in Hessian."
                << "\n";
      std::cout << "### Syst solver : matrix R : \n"
                << Eigen::MatrixXd(solver.matrixR()) << '\n';
#endif
      return map;
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
    using factor_type_in_tuple_t = typename std::
        tuple_element<I, decltype(all_factors_tuple_)>::type::value_type;
  };

};   // namespace SAM
#endif
