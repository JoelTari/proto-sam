#include "solver/solver.h"

using namespace ::sam::Inference;

Eigen::MatrixXd
    sam::Inference::SolverSparseQR::compute_covariance(const Eigen::SparseMatrix<double>& A)
{
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  Eigen::SparseMatrix<double>                       H = A.transpose() * A;
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> invsolver;
  invsolver.compute(H);
  Eigen::SparseMatrix<double> I(H.rows(), H.cols());
  I.setIdentity();
  auto H_inv = invsolver.solve(I);
  return H_inv;
  // Eigen::MatrixXd Hdense(H);
  // // Eigen::MatrixXd cov = Hdense.inverse();
  // return Eigen::MatrixXd(H).inverse();   // inverse done through partial LU
}


std::tuple<typename SolverSparseQR::MaP_t,
           std::optional<typename SolverSparseQR::Covariance_t>,
           typename SolverSparseQR::Stats_t>
    SolverSparseQR::solve(const Eigen::SparseMatrix<double>&        A,
                          const Eigen::VectorXd&                    b,
                          const typename SolverSparseQR::Options_t& options)
{
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());
  // stats
  SolverStatsSparseQR stats;
  // solver
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
  // MAP
  {
    PROFILE_SCOPE("QR decomposition", sam_utils::JSONLogger::Instance());
    {
      PROFILE_SCOPE("analyse pattern", sam_utils::JSONLogger::Instance());
      solver.analyzePattern(A);
    }
    {
      PROFILE_SCOPE("factorization", sam_utils::JSONLogger::Instance());
      solver.factorize(A);   // complex
    }
  }

  auto back_substitution = [](auto& solver, auto& b)
  {
    PROFILE_SCOPE("Back-Substitution", sam_utils::JSONLogger::Instance());
    Eigen::VectorXd map = solver.solve(b);
    return map;
  };
  auto map = back_substitution(solver, b);
#if ENABLE_DEBUG_TRACE
  {
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
    // info : enum Success, NumericalIssue, NoConvergence, InvalidInput
  }
#endif
  stats.success       = (solver.info() == 0);
  stats.report_str    = "";   //  str(info);
  stats.rnnz          = solver.matrixR().nonZeros();
  stats.input_options = options;
  stats.rank          = solver.rank();
  // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

  // if options.cache save matrixR
  // R = solver.matrixR().topLeftCorner(rank(),rank())

  std::optional<Covariance_t> optional_covariance;
  if (options.compute_covariance) { optional_covariance = SolverSparseQR::compute_covariance(A); }
  else
    optional_covariance = std::nullopt;


  return {map, optional_covariance, stats};   // R nnz number set at 0 (unused)
}


//------------------------------------------------------------------//
//                       Naive Sparse Solver                        //
//------------------------------------------------------------------//
Eigen::MatrixXd SolverSparseNaive::compute_covariance(const Eigen::SparseMatrix<double>& A)
{
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  Eigen::SparseMatrix<double>                       H = A.transpose() * A;
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> invsolver;
  invsolver.compute(H);
  Eigen::SparseMatrix<double> I(H.rows(), H.cols());
  I.setIdentity();
  auto H_inv = invsolver.solve(I);
  return H_inv;
}

std::tuple<SolverSparseNaive::MaP_t,
           std::optional<SolverSparseNaive::Covariance_t>,
           SolverSparseNaive::Stats_t>
    SolverSparseNaive::solve(const Eigen::SparseMatrix<double>&  A,
                             const Eigen::VectorXd&              b,
                             const SolverSparseNaive::Options_t& options)
{
  PROFILE_SCOPE("Solve Eigen Sparse Naive", sam_utils::JSONLogger::Instance());
  // stats
  SolverStatsSparseNaive stats;

  auto S = compute_covariance(A);

  // MAP
  Eigen::VectorXd X_map;
  {
    PROFILE_SCOPE("Xmap = Covariance times information vector", sam_utils::JSONLogger::Instance());

    X_map = S * A.transpose() * b;
  }


  stats.success       = 1;
  stats.report_str    = "";   //  str(info);
  stats.input_options = options;
  // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

  // if options.cache save covariance
  // R = solver.matrixR().topLeftCorner(rank(),rank())
  std::optional<Covariance_t> optional_covariance;
  if (options.compute_covariance)
    optional_covariance = S;
  else
    optional_covariance = std::nullopt;

  // Note: return covariance value seems to create a copy
  return {X_map, optional_covariance, stats};
}

//------------------------------------------------------------------//
//                      Sparse Cholesky Solver                      //
//------------------------------------------------------------------//
std::tuple<SolverSparseCholesky::MaP_t,
           std::optional<SolverSparseCholesky::Covariance_t>,
           SolverSparseCholesky::Stats_t>
    SolverSparseCholesky::solve(const Eigen::SparseMatrix<double>&     A,
          const Eigen::VectorXd&                 b,
          const SolverSparseCholesky::Options_t& options)
{
  PROFILE_SCOPE("solve Eigen Sparse Cholesky", sam_utils::JSONLogger::Instance());
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
  SolverSparseCholesky::Stats_t                                           stats;
  Eigen::SparseMatrix<double>                       H = A.transpose() * A;
  // MAP
  {
    PROFILE_SCOPE("Cholesky decomposition", sam_utils::JSONLogger::Instance());
    {
      PROFILE_SCOPE("analyse pattern", sam_utils::JSONLogger::Instance());
      solver.analyzePattern(H);
    }
    {
      PROFILE_SCOPE("factorization", sam_utils::JSONLogger::Instance());
      solver.factorize(H);   // complex
    }
  }

  auto back_substitution = [](auto& solver, auto& b)
  {
    PROFILE_SCOPE("Back-Substitution", sam_utils::JSONLogger::Instance());
    Eigen::VectorXd map = solver.solve(b);
    return map;
  };
  Eigen::VectorXd map = back_substitution(solver, A.transpose() * b);
#if ENABLE_DEBUG_TRACE
  {
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
  }
#endif


  stats.success       = (solver.info() == 0);
  stats.report_str    = "";   //  str(info);
  stats.lnnz          = Eigen::SparseMatrix<double>(solver.matrixL()).nonZeros();
  stats.input_options = options;
  // stats.rank = solver.rank(); // no rank() in cholesky
  // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

  // if options.cache save matrixL

  std::optional<SolverSparseCholesky::Covariance_t> optional_covariance;
  if (options.compute_covariance)
  {
    PROFILE_SCOPE("compute covariance", sam_utils::JSONLogger::Instance());
    Eigen::SparseMatrix<double> I(H.rows(), H.cols());
    I.setIdentity();
    optional_covariance = solver.solve(I);
  }
  else
    optional_covariance = std::nullopt;

  return {map, optional_covariance, stats};
}
