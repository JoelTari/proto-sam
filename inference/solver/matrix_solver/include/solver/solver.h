#pragma once

// mainly some wrappers around Eigen Sparse Matrix solver to interface well with the system

#include "utils/config.h"
#include "utils/utils.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <optional>

// externals solver wrapped in Eigen ->
// https://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html
#include <Eigen/CholmodSupport>   // -> cholmod supernodalLLT, suitesparse required
#include <Eigen/PardisoSupport>   // -> mkl required
// #include <Eigen/UmfPackSupport> // multifrontal sequential LU
// #include <Eigen/PaStiXSupport>  // -> supernodal parallel (PastixLLT and PastixLU) requires
// PaStiX library #include <Eigen/SuperLUSupport> // Requires SuperLU lib
#include <Eigen/SPQRSupport>   // suitesparse' SPQR

namespace sam::Inference
{
  //------------------------------------------------------------------//
  //                      Eigen Sparse QR solver                      //
  //------------------------------------------------------------------//
  struct SolverOptionsSparseQR
  {
    //       members could be: cache_R or not , ordering method ect...
    //       have sane default too
    bool compute_covariance = true;
    bool compute_residual   = true;

    bool                                       use_default_ordering = true;
    std::optional<Eigen::SparseMatrix<double>> custom_ordering;   // difficult

    SolverOptionsSparseQR(bool compute_covariance) : compute_covariance(compute_covariance) {}

    SolverOptionsSparseQR() {}
  };

  // stats that are specific to this type of solver (eg QR, chol, naive etc..)
  struct SolverStatsSparseQR
  {
    bool                  success;
    int                   rnnz;
    int                   rank;
    std::string           report_str;
    std::optional<double> residual;   // different from the NLog from a constant
    SolverOptionsSparseQR input_options;
  };

  struct SolverSparseQR
  {
    // TODO: have a cache substructure : R, ordering (or P s.t AP = QR)

    using Stats_t      = SolverStatsSparseQR;
    using Options_t    = SolverOptionsSparseQR;
    using MaP_t        = Eigen::VectorXd;
    using Covariance_t = Eigen::MatrixXd;

    constexpr static const char name[] = "SparseQR";

    static Eigen::MatrixXd compute_covariance(const Eigen::SparseMatrix<double>& A);

    static std::tuple<MaP_t, std::optional<Covariance_t>, SolverStatsSparseQR>
        solve(const Eigen::SparseMatrix<double>& A,
              const Eigen::VectorXd&             b,
              const SolverOptionsSparseQR&       options = SolverOptionsSparseQR());
  };

  //------------------------------------------------------------------//
  //                       NAIVE SPARSE SOLVER                        //
  //------------------------------------------------------------------//
  struct SolverOptionsSparseNaive
  {
    //       members could be: cache_R or not , ordering method ect...
    //       have sane default too
    bool compute_covariance = true;
    bool compute_residual   = true;

    bool                                       use_default_ordering = true;
    std::optional<Eigen::SparseMatrix<double>> custom_ordering;   // difficult

    SolverOptionsSparseNaive(bool compute_covariance) : compute_covariance(compute_covariance) {}

    SolverOptionsSparseNaive() {}
  };

  // stats that are specific to this type of solver (eg QR, chol, naive etc..)
  struct SolverStatsSparseNaive
  {
    bool                     success;
    int                      rnnz;
    int                      rank;
    std::string              report_str;
    std::optional<double>    residual;   // different from the NLog from a constant
    SolverOptionsSparseNaive input_options;
  };

  struct SolverSparseNaive
  {
    using Stats_t      = SolverStatsSparseNaive;
    using Options_t    = SolverOptionsSparseNaive;
    using MaP_t        = Eigen::VectorXd;
    using Covariance_t = Eigen::MatrixXd;

    constexpr static const char name[] = "SparseNaive";

    static Eigen::MatrixXd compute_covariance(const Eigen::SparseMatrix<double>& A);


    static std::tuple<MaP_t, std::optional<Covariance_t>, SolverStatsSparseNaive>
        solve(const Eigen::SparseMatrix<double>& A,
              const Eigen::VectorXd&             b,
              const SolverOptionsSparseNaive&    options = SolverOptionsSparseNaive());
  };


  //------------------------------------------------------------------//
  //             Sparse Cholesky solver (simplicial LLT)              //
  //------------------------------------------------------------------//
  struct SolverOptionsSparseSimplicialLLT
  {
    //       members could be: cache_R or not , ordering method ect...
    //       have sane default too
    bool compute_covariance = true;
    bool compute_residual   = true;

    bool                                       use_default_ordering = true;
    std::optional<Eigen::SparseMatrix<double>> custom_ordering;   // difficult

    SolverOptionsSparseSimplicialLLT(bool compute_covariance)
        : compute_covariance(compute_covariance)
    {
    }

    SolverOptionsSparseSimplicialLLT() {}
  };

  struct SolverStatsSparseSimplicialLLT
  {
    bool success;
    int  lnnz;
    // int rank; // no rank in cholesky
    std::string                      report_str;
    std::optional<double>            residual;   // different from the NLog from a constant
    SolverOptionsSparseSimplicialLLT input_options;
  };

  struct SolverSparseSimplicialLLT
  {
    using Stats_t      = SolverStatsSparseSimplicialLLT;
    using Options_t    = SolverOptionsSparseSimplicialLLT;
    using MaP_t        = Eigen::VectorXd;
    using Covariance_t = Eigen::MatrixXd;

    constexpr static const char name[] = "SparseSimplicialLLT";

    static std::tuple<MaP_t, std::optional<Covariance_t>, SolverStatsSparseSimplicialLLT>
        solve(const Eigen::SparseMatrix<double>&      A,
              const Eigen::VectorXd&                  b,
              const SolverOptionsSparseSimplicialLLT& options = SolverOptionsSparseSimplicialLLT());
  };


  //------------------------------------------------------------------//
  //            PARDISIO SPARSE SOLVER (intel's cholesky)             //
  //------------------------------------------------------------------//
  struct SolverOptionsSparsePardisoLLT
  {
    //       members could be: cache_R or not , ordering method ect...
    //       have sane default too
    bool compute_covariance = true;
    bool compute_residual   = true;

    bool                                       use_default_ordering = true;
    std::optional<Eigen::SparseMatrix<double>> custom_ordering;   // difficult

    SolverOptionsSparsePardisoLLT(bool compute_covariance) : compute_covariance(compute_covariance)
    {
    }

    SolverOptionsSparsePardisoLLT() {}
  };

  struct SolverStatsSparsePardisoLLT
  {
    bool success;
    int  lnnz;
    // int rank; // no rank in cholesky
    std::string                   report_str;
    std::optional<double>         residual;   // different from the NLog from a constant
    SolverOptionsSparsePardisoLLT input_options;
  };

  struct SolverSparsePardisoLLT
  {
    using Stats_t      = SolverStatsSparsePardisoLLT;
    using Options_t    = SolverOptionsSparsePardisoLLT;
    using MaP_t        = Eigen::VectorXd;
    using Covariance_t = Eigen::MatrixXd;

    constexpr static const char name[] = "PardisoLLT";

    static std::tuple<MaP_t, std::optional<Covariance_t>, SolverStatsSparsePardisoLLT>
        solve(const Eigen::SparseMatrix<double>&   A,
              const Eigen::VectorXd&               b,
              const SolverOptionsSparsePardisoLLT& options = SolverOptionsSparsePardisoLLT());
  };

  //------------------------------------------------------------------//
  //             SPQR solver (eigen wrapper around SPQR)              //
  //------------------------------------------------------------------//
  struct SolverOptionsSPQR
  {
    //       members could be: cache_R or not , ordering method ect...
    //       have sane default too
    bool compute_covariance = true;
    bool compute_residual   = true;

    bool                                       use_default_ordering = true;
    std::optional<Eigen::SparseMatrix<double>> custom_ordering;   // difficult

    SolverOptionsSPQR(bool compute_covariance) : compute_covariance(compute_covariance) {}

    SolverOptionsSPQR() {}
  };

  // stats that are specific to this type of solver (eg QR, chol, naive etc..)
  struct SolverStatsSPQR
  {
    bool                  success;
    int                   rnnz;
    int                   rank;
    std::string           report_str;
    std::optional<double> residual;   // different from the NLog from a constant
    SolverOptionsSPQR     input_options;
  };

  struct SolverSPQR
  {
    // TODO: have a cache substructure : R, ordering

    using Stats_t      = SolverStatsSPQR;
    using Options_t    = SolverOptionsSPQR;
    using MaP_t        = Eigen::VectorXd;
    using Covariance_t = Eigen::MatrixXd;

    constexpr static const char name[] = "SPQR";

    static Eigen::MatrixXd compute_covariance(const Eigen::SparseMatrix<double>& A);

    static std::tuple<MaP_t, std::optional<Covariance_t>, SolverStatsSPQR>
        solve(const Eigen::SparseMatrix<double>& A,
              const Eigen::VectorXd&             b,
              const SolverOptionsSPQR&           options = SolverOptionsSPQR());
  };


  //------------------------------------------------------------------//
  //                  Sparse Cholmod supernodal LLT                   //
  //------------------------------------------------------------------//
  struct SolverOptionsSparseSupernodalLLT
  {
    //       members could be: cache_R or not , ordering method ect...
    //       have sane default too
    bool compute_covariance = true;
    bool compute_residual   = true;

    bool                                       use_default_ordering = true;
    std::optional<Eigen::SparseMatrix<double>> custom_ordering;   // difficult

    SolverOptionsSparseSupernodalLLT(bool compute_covariance)
        : compute_covariance(compute_covariance)
    {
    }

    SolverOptionsSparseSupernodalLLT() {}
  };

  struct SolverStatsSparseSupernodalLLT
  {
    bool success;
    int  lnnz;
    // int rank; // no rank in cholesky
    std::string                      report_str;
    std::optional<double>            residual;   // different from the NLog from a constant
    SolverOptionsSparseSupernodalLLT input_options;
  };

  struct SolverSparseSupernodalLLT
  {
    using Stats_t      = SolverStatsSparseSupernodalLLT;
    using Options_t    = SolverOptionsSparseSupernodalLLT;
    using MaP_t        = Eigen::VectorXd;
    using Covariance_t = Eigen::MatrixXd;

    constexpr static const char name[] = "SparseCholdmodSupernodalLLT";

    static Eigen::MatrixXd compute_covariance(const Eigen::SparseMatrix<double>& A);

    static std::tuple<MaP_t, std::shared_ptr<std::optional<Covariance_t>>, SolverStatsSparseSupernodalLLT>
        solve(const Eigen::SparseMatrix<double>&      A,
              const Eigen::VectorXd&                  b,
              const SolverOptionsSparseSupernodalLLT& options = SolverOptionsSparseSupernodalLLT());
  };

}   // namespace sam::Inference
