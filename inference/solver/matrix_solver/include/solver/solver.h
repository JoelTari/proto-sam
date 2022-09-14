#pragma once

#include <Eigen/Sparse>
#include <optional>
#include "utils/config.h"
#include "utils/utils.h"

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
    bool compute_residual = true;

    bool use_default_ordering = true;
    std::optional< Eigen::SparseMatrix<double>> custom_ordering; // difficult

    SolverOptionsSparseQR(bool compute_covariance)
      :compute_covariance(compute_covariance)
    {}

    SolverOptionsSparseQR(){}
  };

  // stats that are specific to this type of solver (eg QR, chol, naive etc..)
  struct SolverStatsSparseQR
  {
    bool success;
    int rnnz;
    int rank;
    std::string report_str;
    std::optional<double> residual; // different from the NLog from a constant
    SolverOptionsSparseQR input_options;
  };

  struct SolverSparseQR
  {
    // TODO: have a cache substructure : R, ordering

    using Stats_t = SolverStatsSparseQR;
    using Options_t = SolverOptionsSparseQR;
    using MaP_t = Eigen::VectorXd;
    using Covariance_t = Eigen::MatrixXd;

    static Eigen::MatrixXd compute_covariance(const Eigen::SparseMatrix<double> & A);
    
    static 
    std::tuple<MaP_t,std::optional<Covariance_t>, SolverStatsSparseQR> 
      solve(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b, const SolverOptionsSparseQR & options = SolverOptionsSparseQR() );
  };

  //------------------------------------------------------------------//
  //                       NAIVE SPARSE SOLVER                        //
  //------------------------------------------------------------------//
  struct SolverOptionsSparseNaive
  {
  //       members could be: cache_R or not , ordering method ect...
  //       have sane default too
    bool compute_covariance = true;
    bool compute_residual = true;

    bool use_default_ordering = true;
    std::optional< Eigen::SparseMatrix<double>> custom_ordering; // difficult

    SolverOptionsSparseNaive(bool compute_covariance)
      :compute_covariance(compute_covariance)
    {}

    SolverOptionsSparseNaive(){}
  };

  // stats that are specific to this type of solver (eg QR, chol, naive etc..)
  struct SolverStatsSparseNaive
  {
    bool success;
    int rnnz;
    int rank;
    std::string report_str;
    std::optional<double> residual; // different from the NLog from a constant
    SolverOptionsSparseNaive input_options;
  };

  struct SolverSparseNaive
  {
    using Stats_t = SolverStatsSparseNaive;
    using Options_t = SolverOptionsSparseNaive;
    using MaP_t = Eigen::VectorXd;
    using Covariance_t = Eigen::MatrixXd;

    static Eigen::MatrixXd compute_covariance(const Eigen::SparseMatrix<double> & A);
    

    static 
    std::tuple<MaP_t,std::optional<Covariance_t>, SolverStatsSparseNaive> 
      solve(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b, const SolverOptionsSparseNaive & options = SolverOptionsSparseNaive() );

  };


  //------------------------------------------------------------------//
  //                      CHOLESKY SPARSE SOLVER                      //
  //------------------------------------------------------------------//
  struct SolverOptionsSparseCholesky
  {
  //       members could be: cache_R or not , ordering method ect...
  //       have sane default too
    bool compute_covariance = true;
    bool compute_residual = true;

    bool use_default_ordering = true;
    std::optional< Eigen::SparseMatrix<double>> custom_ordering; // difficult

    SolverOptionsSparseCholesky(bool compute_covariance)
      :compute_covariance(compute_covariance)
    {}

    SolverOptionsSparseCholesky(){}
  };

  struct SolverStatsSparseCholesky
  {
    bool success;
    int lnnz;
    // int rank; // no rank in cholesky
    std::string report_str;
    std::optional<double> residual; // different from the NLog from a constant
    SolverOptionsSparseCholesky input_options;
  };

  struct SolverSparseCholesky
  {
    using Stats_t = SolverStatsSparseCholesky;
    using Options_t = SolverOptionsSparseCholesky;
    using MaP_t = Eigen::VectorXd;
    using Covariance_t = Eigen::MatrixXd;

    static 
    std::tuple<MaP_t,std::optional<Covariance_t>, SolverStatsSparseCholesky> 
      solve(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b, const SolverOptionsSparseCholesky & options = SolverOptionsSparseCholesky() );
    

  };
}
