#include "solver/solver.h"

using namespace ::sam::Inference;

Eigen::MatrixXd
    sam::Inference::SolverSparseQR::compute_covariance(const Eigen::SparseMatrix<double>& A)
{
  Eigen::SparseMatrix<double>                       H = A.transpose() * A;
  // Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> invsolver;
  // invsolver.compute(H);
  // Eigen::SparseMatrix<double> I(H.rows(), H.cols());
  // I.setIdentity();
  // auto H_inv = invsolver.solve(I);
  // return H_inv;
  return Eigen::MatrixXd(H).inverse();
  // roughly, when dense inverse takes 3s, SparseLU and SimplicialLLT takes 38s, SparseQR takes way longer
  // so the dense .inverse() method is best performing (all tests done with -o3 + mkl BLAS/LAPACK + TBB active)
  // The dense method is the only one that manages to use all cores.
}


std::tuple<typename SolverSparseQR::MaP_t,
           std::optional<typename SolverSparseQR::Covariance_t>,
           typename SolverSparseQR::Stats_t>
    SolverSparseQR::solve(const Eigen::SparseMatrix<double>&        A,
                          const Eigen::VectorXd&                    b,
                          const typename SolverSparseQR::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSparseQR::name);
  PROFILE_SCOPE(scope_name.c_str(),sam_utils::JSONLogger::Instance());
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
  PROFILE_SCOPE("compute_covariance: dense",sam_utils::JSONLogger::Instance());

  Eigen::SparseMatrix<double>                       H = A.transpose() * A;
  // Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> invsolver;
  // invsolver.compute(H);
  // Eigen::SparseMatrix<double> I(H.rows(), H.cols());
  // I.setIdentity();
  // auto H_inv = invsolver.solve(I);
  // return H_inv;
  return Eigen::MatrixXd(H).inverse();
  // roughly, when dense inverse takes 3s, SparseLU and SimplicialLLT takes 38s, SparseQR takes way longer
  // so the dense .inverse() method is best performing (all tests done with -o3 + mkl BLAS/LAPACK + TBB active)
  // The dense method is the only one that manages to use all cores.
}

std::tuple<SolverSparseNaive::MaP_t,
           std::optional<SolverSparseNaive::Covariance_t>,
           SolverSparseNaive::Stats_t>
    SolverSparseNaive::solve(const Eigen::SparseMatrix<double>&  A,
                             const Eigen::VectorXd&              b,
                             const SolverSparseNaive::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSparseNaive::name);
  PROFILE_SCOPE(scope_name.c_str(), sam_utils::JSONLogger::Instance());
  // stats
  SolverStatsSparseNaive stats;

  auto S = compute_covariance(A);

  // MAP
  Eigen::VectorXd X_map;
  {
    PROFILE_SCOPE("Xmap = Covariance times information vector", sam_utils::JSONLogger::Instance());

    X_map = S *( A.transpose() * b); // the parenthesis are important (e.g. : 11 ms vs 210 ms without !)
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

  // FIX: return covariance value seems to create a copy:   110 ms, maybe just use a shared_ptr
  // this is probably a quirk of optional that prevent RVO of the big underlying object
  return {X_map, optional_covariance, stats};
}

//------------------------------------------------------------------//
//                      Sparse SimplicialLLT Solver                      //
//------------------------------------------------------------------//
std::tuple<SolverSparseSimplicialLLT::MaP_t,
           std::optional<SolverSparseSimplicialLLT::Covariance_t>,
           SolverSparseSimplicialLLT::Stats_t>
    SolverSparseSimplicialLLT::solve(const Eigen::SparseMatrix<double>&     A,
          const Eigen::VectorXd&                 b,
          const SolverSparseSimplicialLLT::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSparseSimplicialLLT::name);
  PROFILE_SCOPE(scope_name.c_str(), sam_utils::JSONLogger::Instance());
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
  SolverSparseSimplicialLLT::Stats_t                                           stats;
  Eigen::SparseMatrix<double>                       H = A.transpose() * A;
  // MAP
  {
    PROFILE_SCOPE("SimplicialLLT decomposition", sam_utils::JSONLogger::Instance());
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

  std::optional<SolverSparseSimplicialLLT::Covariance_t> optional_covariance;
  if (options.compute_covariance)
  {
    PROFILE_SCOPE("compute covariance: dense", sam_utils::JSONLogger::Instance());
    optional_covariance = Eigen::MatrixXd(H).inverse();
  }
  else
    optional_covariance = std::nullopt;

  return {map, optional_covariance, stats};
}

//------------------------------------------------------------------//
//                      Sparse PardisoLLT Solver                      //
//------------------------------------------------------------------//
std::tuple<SolverSparsePardisoLLT::MaP_t,
           std::optional<SolverSparsePardisoLLT::Covariance_t>,
           SolverSparsePardisoLLT::Stats_t>
    SolverSparsePardisoLLT::solve(const Eigen::SparseMatrix<double>&     A,
          const Eigen::VectorXd&                 b,
          const SolverSparsePardisoLLT::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSparsePardisoLLT::name);
  PROFILE_SCOPE(scope_name.c_str(), sam_utils::JSONLogger::Instance());
  Eigen::PardisoLLT<Eigen::SparseMatrix<double>> solver;
  SolverSparsePardisoLLT::Stats_t                                           stats;
  Eigen::SparseMatrix<double>                       H = A.transpose() * A;
  // MAP
  {
    PROFILE_SCOPE("PardisoLLT decomposition", sam_utils::JSONLogger::Instance());
    {
      PROFILE_SCOPE("analyse pattern", sam_utils::JSONLogger::Instance());
      solver.analyzePattern(H);
    }
    {
      PROFILE_SCOPE("factorization", sam_utils::JSONLogger::Instance());
      solver.factorize(H);   // complex
    }
  }

  auto back_substitution = [](auto& solver, const auto& b)
  {
    PROFILE_SCOPE("Back-Substitution", sam_utils::JSONLogger::Instance());
    Eigen::VectorXd map = solver.solve(b);
    return map;
  };
  Eigen::VectorXd map = back_substitution(solver, Eigen::VectorXd(A.transpose() * b) );
#if ENABLE_DEBUG_TRACE
  {
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
  }
#endif


  stats.success       = (solver.info() == 0);
  stats.report_str    = "";   //  str(info);
  // stats.lnnz          = Eigen::SparseMatrix<double>(solver.()).nonZeros();
  stats.input_options = options;
  // stats.rank = solver.rank(); // no rank() in cholesky
  // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

  // if options.cache save matrixL

  std::optional<SolverSparsePardisoLLT::Covariance_t> optional_covariance;
  if (options.compute_covariance)
  {
    PROFILE_SCOPE("compute covariance: dense", sam_utils::JSONLogger::Instance());
    // todo: maybe see if, by chance, pardiso solver over I exploits multiple cores
    // (see the comment in the compute_covariance methods for reference)
    optional_covariance = Eigen::MatrixXd(H).inverse();
  }
  else
    optional_covariance = std::nullopt;

  return {map, optional_covariance, stats};
}

//------------------------------------------------------------------//
//             SPQR solver (eigen wrapper around SPQR)              //
//------------------------------------------------------------------//
Eigen::MatrixXd
    sam::Inference::SolverSPQR::compute_covariance(const Eigen::SparseMatrix<double>& A)
{
  PROFILE_SCOPE("compute covariance: dense",sam_utils::JSONLogger::Instance());
  Eigen::SparseMatrix<double>                       H = A.transpose() * A;
  // Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> invsolver;
  // invsolver.compute(H);
  // Eigen::SparseMatrix<double> I(H.rows(), H.cols());
  // I.setIdentity();
  // auto H_inv = invsolver.solve(I);
  // return H_inv;
  return Eigen::MatrixXd(H).inverse();
  // roughly, when dense inverse takes 3s, SparseLU and SimplicialLLT takes 38s, SPQR takes way longer
  // so the dense .inverse() method is best performing (all tests done with -o3 + mkl BLAS/LAPACK + TBB active)
  // The dense method is the only one that manages to use all cores.
}

// https://eigen.tuxfamily.org/dox/classEigen_1_1SPQR.html

std::tuple<typename SolverSPQR::MaP_t,
           std::optional<typename SolverSPQR::Covariance_t>,
           typename SolverSPQR::Stats_t>
    SolverSPQR::solve(const Eigen::SparseMatrix<double>&        A,
                          const Eigen::VectorXd&                    b,
                          const typename SolverSPQR::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSPQR::name);
  PROFILE_SCOPE( scope_name.c_str() ,sam_utils::JSONLogger::Instance());
  // stats
  SolverStatsSPQR stats;
  // solver
  auto declare_solver_and_attach_A = [](auto & A){ 
    PROFILE_SCOPE("Create solver and decompose",sam_utils::JSONLogger::Instance());
    return Eigen::SPQR<Eigen::SparseMatrix<double>>(A); 
  };
  auto solver = declare_solver_and_attach_A(A);
  // MAP
  // {
    // PROFILE_SCOPE("QR decomposition", sam_utils::JSONLogger::Instance());
    // solver.
    // {
    //   PROFILE_SCOPE("analyse pattern", sam_utils::JSONLogger::Instance());
    //   solver.analyzePattern(A);
    // }
    // {
    //   PROFILE_SCOPE("factorization", sam_utils::JSONLogger::Instance());
    //   solver.factorize(A);   // complex
    // }
  // }

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
  // stats.rnnz          = solver.matrixR().nonZeros();
  stats.input_options = options;
  stats.rank          = solver.rank();
  // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

  // if options.cache save matrixR
  // R = solver.matrixR().topLeftCorner(rank(),rank())

  std::optional<Covariance_t> optional_covariance;
  if (options.compute_covariance) { optional_covariance = SolverSPQR::compute_covariance(A); }
  else
    optional_covariance = std::nullopt;


  return {map, optional_covariance, stats};   // R nnz number set at 0 (unused)
}
