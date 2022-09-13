#include <Eigen/Sparse>
#include <optional>
#include "utils/config.h"
#include "utils/utils.h"

namespace sam::Inference
{
  struct SolverOptionsQR
  {
  //       members could be: cache_R or not , ordering method ect...
  //       have sane default too
    bool compute_covariance = true;
    bool compute_residual = true;

    bool use_default_ordering = true;
    std::optional< Eigen::SparseMatrix<double>> custom_ordering; // difficult

    SolverOptionsQR(bool compute_covariance)
      :compute_covariance(compute_covariance)
    {}

    SolverOptionsQR(){}
  };

  // stats that are specific to this type of solver (eg QR, chol, naive etc..)
  struct SolverStatsQR
  {
    bool success;
    int rnnz;
    int rank;
    std::string report_str;
    std::optional<double> residual; // different from the NLog from a constant
    SolverOptionsQR input_options;
  };

  struct SolverQR
  {
    // TODO: have a cache substructure : R, ordering

    using Stats_t = SolverStatsQR;
    using Options_t = SolverOptionsQR;
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
    std::tuple<MaP_t,std::optional<Covariance_t>, SolverStatsQR> 
      solve(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b, const SolverOptionsQR & options = SolverOptionsQR() )
    // TODO: add a solverOpts variable: check rank or not, check success, count the nnz of R or not
    {
      PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
      // stats
      SolverStatsQR stats;
      // solver
      Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
      // MAP
      {
        PROFILE_SCOPE("QR decomposition",sam_utils::JSONLogger::Instance());
        solver.compute(A); // analysePattern() & factorize()
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
      // stats.ordering = solver.colsPermutation(); (that depends of what I want actually)

      // if options.cache save matrixR
      // R = solver.matrixR().topLeftCorner(rank(),rank())
      
      std::optional<Covariance_t> optional_covariance;
      if (options.compute_covariance)
      {
        optional_covariance = SolverQR::compute_covariance(A);
      }
      else optional_covariance = std::nullopt;



      return {map, optional_covariance, SolverStatsQR()}; // R nnz number set at 0 (unused)
    }

  };

  
}
