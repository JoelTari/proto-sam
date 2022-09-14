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

    static Eigen::MatrixXd compute_covariance(const Eigen::SparseMatrix<double> & A)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      auto At = Eigen::MatrixXd(A.transpose());
      auto H = At*A;
      return H.inverse() ; // inverse done through partial LU
    }

    static 
    std::tuple<MaP_t,std::optional<Covariance_t>, SolverStatsSparseQR> 
      solve(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b, const SolverOptionsSparseQR & options = SolverOptionsSparseQR() )
    // TODO: add a solverOpts variable: check rank or not, check success, count the nnz of R or not
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      // stats
      SolverStatsSparseQR stats;
      // solver
      Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
      // MAP
      {
        PROFILE_SCOPE("QR decomposition",sam_utils::JSONLogger::Instance());
        {
          PROFILE_SCOPE("analyse pattern",sam_utils::JSONLogger::Instance());
          solver.analyzePattern(A);
        }
        {
          PROFILE_SCOPE("factorization",sam_utils::JSONLogger::Instance());
          solver.factorize(A); // complex
        }
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
        // info : enum Success, NumericalIssue, NoConvergence, InvalidInput
      }
#endif
      stats.success = (solver.info() == 0);
      stats.report_str = "";   //  str(info);
      stats.rnnz = solver.matrixR().nonZeros();
      stats.input_options = options;
      stats.rank = solver.rank();
      // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

      // if options.cache save matrixR
      // R = solver.matrixR().topLeftCorner(rank(),rank())
      
      std::optional<Covariance_t> optional_covariance;
      if (options.compute_covariance)
      {
        optional_covariance = SolverSparseQR::compute_covariance(A);
      }
      else optional_covariance = std::nullopt;



      return {map, optional_covariance, stats}; // R nnz number set at 0 (unused)
    }

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

    static Eigen::MatrixXd compute_covariance(const Eigen::SparseMatrix<double> & A)
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

      auto At = Eigen::MatrixXd(A.transpose());
      auto H = At*A;
      return H.inverse(); // inverse done through partial LU
    }

    static 
    std::tuple<MaP_t,std::optional<Covariance_t>, SolverStatsSparseNaive> 
      solve(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b, const SolverOptionsSparseNaive & options = SolverOptionsSparseNaive() )
    {
      PROFILE_SCOPE("Solve Eigen Sparse Naive",sam_utils::JSONLogger::Instance());
      // stats
      SolverStatsSparseNaive stats;

      auto S = compute_covariance(A);
  
      // MAP
      Eigen::VectorXd X_map;
      {
        PROFILE_SCOPE("Covariance times information vector",sam_utils::JSONLogger::Instance());

        X_map = S*A.transpose()*b;
      }


      stats.success = 1;
      stats.report_str = "";   //  str(info);
      stats.input_options = options;
      // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

      // if options.cache save covariance
      // R = solver.matrixR().topLeftCorner(rank(),rank())
      
      std::optional<Covariance_t> optional_covariance = S;

      return {X_map, optional_covariance, stats};
    }

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
      solve(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b, const SolverOptionsSparseCholesky & options = SolverOptionsSparseCholesky() )
    {
      PROFILE_SCOPE("solve Eigen Sparse Cholesky",sam_utils::JSONLogger::Instance());
      Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
      Stats_t stats;
      Eigen::SparseMatrix<double> H = A.transpose()*A;
      // MAP
      {
        PROFILE_SCOPE("Cholesky decomposition",sam_utils::JSONLogger::Instance());
        {
          PROFILE_SCOPE("analyse pattern",sam_utils::JSONLogger::Instance());
          solver.analyzePattern(H);
        }
        {
          PROFILE_SCOPE("factorization",sam_utils::JSONLogger::Instance());
          solver.factorize(H); // complex
        }
      }

      auto back_substitution = [](auto & solver, auto & b)
        {
          PROFILE_SCOPE("Back-Substitution",sam_utils::JSONLogger::Instance());
          Eigen::VectorXd map = solver.solve(b);
          return map;
        };
      Eigen::VectorXd map = back_substitution(solver,A.transpose()*b);
#if ENABLE_DEBUG_TRACE
      {
        std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
        std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
      }
#endif


      stats.success = (solver.info() == 0);
      stats.report_str = "";   //  str(info);
      stats.lnnz = Eigen::SparseMatrix<double>(solver.matrixL()).nonZeros();
      stats.input_options = options;
      // stats.rank = solver.rank(); // no rank() in cholesky
      // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

      // if options.cache save matrixL

      std::optional<Covariance_t> optional_covariance;
      if (options.compute_covariance)
      {
        PROFILE_SCOPE("compute covariance", sam_utils::JSONLogger::Instance());
        // Eigen::MatrixXd Linv = Eigen::MatrixXd(solver.matrixL()).inverse();
        // optional_covariance = Linv.transpose()*Linv;
        optional_covariance = Eigen::MatrixXd(H).inverse();
      }
      else 
        optional_covariance = std::nullopt;

      return {map, optional_covariance, stats};
    }

  };
}
