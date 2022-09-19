#include "solver/solver.h"

#include <future>

using namespace ::sam::Inference;

Eigen::MatrixXd
    sam::Inference::SolverSparseQR::compute_covariance(const Eigen::SparseMatrix<double>& A)
{
  Eigen::SparseMatrix<double> H = A.transpose() * A;
  // Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> invsolver;
  // invsolver.compute(H);
  // Eigen::SparseMatrix<double> I(H.rows(), H.cols());
  // I.setIdentity();
  // auto H_inv = invsolver.solve(I);
  // return H_inv;
  return Eigen::MatrixXd(H).inverse();
  // roughly, when dense inverse takes 3s, SparseLU and SimplicialLLT takes 38s, SparseQR takes way
  // longer so the dense .inverse() method is best performing (all tests done with -o3 + mkl
  // BLAS/LAPACK + TBB active) The dense method is the only one that manages to use all cores.
}


std::tuple<typename SolverSparseQR::MaP_t,
           typename SolverSparseQR::OptCovariance_ptr_t,
           typename SolverSparseQR::Stats_t>
    SolverSparseQR::solve(const Eigen::SparseMatrix<double>&        A,
                          const Eigen::VectorXd&                    b,
                          const typename SolverSparseQR::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSparseQR::name);
  PROFILE_SCOPE(scope_name.c_str());
  // stats
  SolverStatsSparseQR stats;
  // solver
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
  // MAP
  {
    PROFILE_SCOPE("QR decomposition");
    {
      PROFILE_SCOPE("analyse pattern");
      solver.analyzePattern(A);
    }
    {
      PROFILE_SCOPE("factorization");
      solver.factorize(A);   // complex
    }
  }

  auto back_substitution = [](auto& solver, auto& b)
  {
    PROFILE_SCOPE("Back-Substitution");
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

  OptCovariance_ptr_t optional_covariance_ptr;
  if (options.compute_covariance) { optional_covariance_ptr = std::make_shared<OptCovariance_t>( SolverSparseQR::compute_covariance(A)); }
  else
    optional_covariance_ptr = std::make_shared<OptCovariance_t>( std::nullopt);


  return {map, optional_covariance_ptr, stats};   // R nnz number set at 0 (unused)
}


//------------------------------------------------------------------//
//                       Naive Sparse Solver                        //
//------------------------------------------------------------------//
Eigen::MatrixXd SolverSparseNaive::compute_covariance(const Eigen::SparseMatrix<double>& A)
{
  PROFILE_SCOPE("compute_covariance: dense");

  Eigen::SparseMatrix<double> H = A.transpose() * A;
  // Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> invsolver;
  // invsolver.compute(H);
  // Eigen::SparseMatrix<double> I(H.rows(), H.cols());
  // I.setIdentity();
  // auto H_inv = invsolver.solve(I);
  // return H_inv;
  return Eigen::MatrixXd(H).inverse();
  // roughly, when dense inverse takes 3s, SparseLU and SimplicialLLT takes 38s, SparseQR takes way
  // longer so the dense .inverse() method is best performing (all tests done with -o3 + mkl
  // BLAS/LAPACK + TBB active) The dense method is the only one that manages to use all cores.
}

std::tuple<typename SolverSparseNaive::MaP_t,
           typename SolverSparseNaive::OptCovariance_ptr_t,
           typename SolverSparseNaive::Stats_t>
    SolverSparseNaive::solve(const Eigen::SparseMatrix<double>&  A,
                             const Eigen::VectorXd&              b,
                             const SolverSparseNaive::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSparseNaive::name);
  PROFILE_SCOPE(scope_name.c_str());
  // stats
  SolverStatsSparseNaive stats;

  // (not really-)optional covariance
  OptCovariance_ptr_t optional_covariance_ptr = std::make_shared<OptCovariance_t>( compute_covariance(A));

  // MAP
  Eigen::VectorXd X_map;
  {
    PROFILE_SCOPE("Xmap = Covariance times information vector");

    X_map = optional_covariance_ptr->value()
            * (A.transpose()
               * b);   // the parenthesis are important (e.g. : 11 ms vs 210 ms without !)
    // OPTIMIZE: use future for At*b
  }


  stats.success       = 1;
  stats.report_str    = "";   //  str(info);
  stats.input_options = options;
  // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

  // if options.cache save covariance
  // R = solver.matrixR().topLeftCorner(rank(),rank())

  return {X_map, optional_covariance_ptr, stats};
}

//------------------------------------------------------------------//
//                      Sparse SimplicialLLT Solver                      //
//------------------------------------------------------------------//
std::tuple<SolverSparseSimplicialLLT::MaP_t,
           SolverSparseSimplicialLLT::OptCovariance_ptr_t,
           SolverSparseSimplicialLLT::Stats_t>
    SolverSparseSimplicialLLT::solve(const Eigen::SparseMatrix<double>&          A,
                                     const Eigen::VectorXd&                      b,
                                     const SolverSparseSimplicialLLT::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSparseSimplicialLLT::name);
  PROFILE_SCOPE(scope_name.c_str());
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
  SolverSparseSimplicialLLT::Stats_t                stats;
  Eigen::SparseMatrix<double>                       H = A.transpose() * A;
  // MAP
  {
    PROFILE_SCOPE("SimplicialLLT decomposition");
    {
      PROFILE_SCOPE("analyse pattern");
      solver.analyzePattern(H);
    }
    {
      PROFILE_SCOPE("factorization");
      solver.factorize(H);   // complex
    }
  }

  auto back_substitution = [](auto& solver, auto& b)
  {
    PROFILE_SCOPE("Back-Substitution");
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

  OptCovariance_ptr_t optional_covariance_ptr;
  if (options.compute_covariance)
  {
    PROFILE_SCOPE("compute covariance: dense");
    optional_covariance_ptr = std::make_shared<OptCovariance_t>( Eigen::MatrixXd(H).inverse());
  }
  else
    optional_covariance_ptr = std::make_shared<OptCovariance_t>( std::nullopt);

  return {map, optional_covariance_ptr, stats};
}

//------------------------------------------------------------------//
//                      Sparse PardisoLLT Solver                      //
//------------------------------------------------------------------//
std::tuple<SolverSparsePardisoLLT::MaP_t,
           SolverSparsePardisoLLT::OptCovariance_ptr_t,
           SolverSparsePardisoLLT::Stats_t>
    SolverSparsePardisoLLT::solve(const Eigen::SparseMatrix<double>&       A,
                                  const Eigen::VectorXd&                   b,
                                  const SolverSparsePardisoLLT::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSparsePardisoLLT::name);
  PROFILE_SCOPE(scope_name.c_str());
  Eigen::PardisoLLT<Eigen::SparseMatrix<double>> solver;
  SolverSparsePardisoLLT::Stats_t                stats;
  Eigen::SparseMatrix<double>                    H = A.transpose() * A;
  // MAP
  {
    PROFILE_SCOPE("PardisoLLT decomposition");
    {
      PROFILE_SCOPE("analyse pattern");
      solver.analyzePattern(H);
    }
    {
      PROFILE_SCOPE("factorization");
      solver.factorize(H);   // complex
    }
  }

  auto back_substitution = [](auto& solver, const auto& b)
  {
    PROFILE_SCOPE("Back-Substitution");
    Eigen::VectorXd map = solver.solve(b);
    return map;
  };
  Eigen::VectorXd map = back_substitution(solver, Eigen::VectorXd(A.transpose() * b));
#if ENABLE_DEBUG_TRACE
  {
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
  }
#endif


  stats.success    = (solver.info() == 0);
  stats.report_str = "";   //  str(info);
  // stats.lnnz          = Eigen::SparseMatrix<double>(solver.()).nonZeros();
  stats.input_options = options;
  // stats.rank = solver.rank(); // no rank() in cholesky
  // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

  // if options.cache save matrixL

  OptCovariance_ptr_t optional_covariance_ptr;
  if (options.compute_covariance)
  {
    PROFILE_SCOPE("compute covariance: dense");
    // todo: maybe see if, by chance, pardiso solver over I exploits multiple cores
    // (see the comment in the compute_covariance methods for reference)
    optional_covariance_ptr = std::make_shared<OptCovariance_t>( Eigen::MatrixXd(H).inverse());
  }
  else
    optional_covariance_ptr = std::make_shared<OptCovariance_t>( std::nullopt);

  return {map, optional_covariance_ptr, stats};
}

//------------------------------------------------------------------//
//             SPQR solver (eigen wrapper around SPQR)              //
//------------------------------------------------------------------//
Eigen::MatrixXd sam::Inference::SolverSPQR::compute_covariance(const Eigen::SparseMatrix<double>& A)
{
  PROFILE_SCOPE("compute covariance: dense");
  Eigen::SparseMatrix<double> H = A.transpose() * A;
  // Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> invsolver;
  // invsolver.compute(H);
  // Eigen::SparseMatrix<double> I(H.rows(), H.cols());
  // I.setIdentity();
  // auto H_inv = invsolver.solve(I);
  // return H_inv;
  return Eigen::MatrixXd(H).inverse();
  // roughly, when dense inverse takes 3s, SparseLU and SimplicialLLT takes 38s, SPQR takes way
  // longer so the dense .inverse() method is best performing (all tests done with -o3 + mkl
  // BLAS/LAPACK + TBB active) The dense method is the only one that manages to use all cores.
}

// https://eigen.tuxfamily.org/dox/classEigen_1_1SPQR.html

std::tuple<typename SolverSPQR::MaP_t,
           typename SolverSPQR::OptCovariance_ptr_t,
           typename SolverSPQR::Stats_t>
    SolverSPQR::solve(const Eigen::SparseMatrix<double>&    A,
                      const Eigen::VectorXd&                b,
                      const typename SolverSPQR::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSPQR::name);
  PROFILE_SCOPE(scope_name.c_str());
  // stats
  SolverStatsSPQR stats;
  // solver
  auto declare_solver_and_attach_A = [](auto& A)
  {
    PROFILE_SCOPE("Create solver and decompose");
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
    PROFILE_SCOPE("Back-Substitution");
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
  stats.success    = (solver.info() == 0);
  stats.report_str = "";   //  str(info);
  // stats.rnnz          = solver.matrixR().nonZeros();
  stats.input_options = options;
  stats.rank          = solver.rank();
  // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

  // if options.cache save matrixR
  // R = solver.matrixR().topLeftCorner(rank(),rank())

  OptCovariance_ptr_t optional_covariance_ptr;
  if (options.compute_covariance) { optional_covariance_ptr = std::make_shared<OptCovariance_t>( SolverSPQR::compute_covariance(A)); }
  else
    optional_covariance_ptr = std::make_shared<OptCovariance_t>( std::nullopt);


  return {map, optional_covariance_ptr, stats};   // R nnz number set at 0 (unused)
}


//------------------------------------------------------------------//
//                    Sparse Cholmod supernodal                     //
//------------------------------------------------------------------//
Eigen::MatrixXd sam::Inference::SolverSparseSupernodalLLT::compute_covariance(
    const Eigen::SparseMatrix<double>& A)
{
  PROFILE_SCOPE("compute covariance: dense");
  Eigen::SparseMatrix<double>                              H = A.transpose() * A;
  Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> invsolver;
  invsolver.compute(H);
  Eigen::SparseMatrix<double> I(A.cols(), A.cols());
  I.setIdentity();
  // Eigen::MatrixXd H_inv = invsolver.solve(I);
  // return Eigen::MatrixXd(H_inv);
  return invsolver.solve(I);
  // return Eigen::MatrixXd(H).inverse();
  // roughly:
  // - dense inverse : 3s
  // - LLT supernodal : 2s
  // - SparseLU and SimplicialLLT takes 38s,
  // - sparseQR takes way longer
  // so the dense .inverse() method is best performing (all tests done with -o3 + mkl BLAS/LAPACK +
  // TBB active) The dense method is the only one that manages to use all cores.
}

std::tuple<typename SolverSparseSupernodalLLT::MaP_t,
           typename SolverSparseSupernodalLLT::OptCovariance_ptr_t,
           typename SolverSparseSupernodalLLT::Stats_t>
    SolverSparseSupernodalLLT::solve(const Eigen::SparseMatrix<double>&                   A,
                                     const Eigen::VectorXd&                               b,
                                     const typename SolverSparseSupernodalLLT::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverSparseSupernodalLLT::name);
  PROFILE_SCOPE(scope_name.c_str());
  // stats
  Stats_t stats;
  // launching some future: AtA and Atb
  std::future<Eigen::SparseMatrix<double>> H_future
      = std::async(std::launch::async,
                   [&A]() -> Eigen::SparseMatrix<double>
                   {
                     PROFILE_SCOPE("H = A^T * A");
                     return A.transpose() * A;
                   });
  std::future<Eigen::VectorXd> rhs_future
      = std::async(std::launch::async,
                   [&A, &b]() -> Eigen::VectorXd
                   {
                     PROFILE_SCOPE("A^T * b");
                     return Eigen::VectorXd(A.transpose() * b);
                   });
  // declaring the solver
  Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> solver;
  // MAP
  {
    PROFILE_SCOPE("Chol decomposition");
    auto H = H_future.get();   // can't call .get() twice
    {
      PROFILE_SCOPE("analyse pattern");
      solver.analyzePattern(H);   // 2.7 ms
    }
    {
      PROFILE_SCOPE("factorization");
      solver.factorize(H);   // complex: 8 ms
    }
  }

  auto back_substitution = [](auto& solver, const auto& rhs)
  {
    PROFILE_SCOPE("Back-Substitution");
    Eigen::VectorXd map = solver.solve(rhs);
    return map;
  };

  auto map = back_substitution(solver, rhs_future.get());
#if ENABLE_DEBUG_TRACE
  {
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
    std::cout << "### Syst solver : " << (solver.info() ? "FAIL" : "SUCCESS") << "\n";
    // info : enum Success, NumericalIssue, NoConvergence, InvalidInput
  }
#endif
  stats.success    = (solver.info() == 0);
  stats.report_str = "";   //  str(info);
  // stats.rnnz          = solver.matrixR().nonZeros();
  stats.input_options = options;
  // stats.ordering = solver.colsPermutation(); (will depend on the desired structure)

  // if options.cache save matrixR
  // R = solver.matrixR().topLeftCorner(rank(),rank())

  OptCovariance_ptr_t optional_covariance_ptr;
  if (options.compute_covariance)
  {
    PROFILE_SCOPE("compute covariance: supernodalLLT");
    Eigen::SparseMatrix<double> I(A.cols(), A.cols());
    I.setIdentity();
    optional_covariance_ptr = std::make_shared<OptCovariance_t>(solver.solve(I));
  }
  else
    optional_covariance_ptr = std::make_shared<OptCovariance_t>(std::nullopt);

  return {map, optional_covariance_ptr, stats};   // R nnz number set at 0 (unused)
}



//------------------------------------------------------------------//
//                       Naive Dense Solver                        //
//------------------------------------------------------------------//
Eigen::MatrixXd SolverDenseNaive::compute_covariance(const Eigen::MatrixXd& A)
{
  PROFILE_SCOPE("compute_covariance: dense");

  Eigen::MatrixXd H = A.transpose() * A;
  return Eigen::MatrixXd(H).inverse();
}

std::tuple<typename SolverDenseNaive::MaP_t,
           typename SolverDenseNaive::OptCovariance_ptr_t,
           typename SolverDenseNaive::Stats_t>
    SolverDenseNaive::solve(const Eigen::MatrixXd&  A,
                             const Eigen::VectorXd&              b,
                             const SolverDenseNaive::Options_t& options)
{
  std::string scope_name = "Solve with " + std::string(SolverDenseNaive::name);
  PROFILE_SCOPE(scope_name.c_str());
  // stats
  SolverStatsDenseNaive stats;

  // (not really-)optional covariance
  OptCovariance_ptr_t optional_covariance_ptr = std::make_shared<OptCovariance_t>( compute_covariance(A));

  // MAP
  Eigen::VectorXd X_map;
  {
    PROFILE_SCOPE("Xmap = Covariance times information vector");

    X_map = optional_covariance_ptr->value()
            * (A.transpose()
               * b);   // the parenthesis are important for perf (otherwise constly matrix*matrix)
    // OPTIMIZE: use future for At*b
  }


  stats.success       = 1;
  stats.report_str    = "";   //  str(info);
  stats.input_options = options;

  return {X_map, optional_covariance_ptr, stats};
}
