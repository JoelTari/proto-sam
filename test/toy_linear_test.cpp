#include "core/sam-system.h"
#include "factor_impl/anchor.hpp"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/linear-translation.hpp"
#include "utils/tuple_patterns.h"
#include "core/marginal.h"


// Ground truth:
// x0: 0, 0
// x1: 1, 0 
// x2: 1, 1
// x3: 0, 1
//
// x3  ───────x2
//  │          │
//  │          │
//  │          │
// x0 ────────x1
//  ┼

//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  // logger
  std::string result_filename
      = sam_utils::currentDateTime() + "_results_toy_linear_test.json";
  sam_utils::JSONLogger::Instance().beginSession("toy_linear_test.cpp", result_filename);

  // scoped Timer
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  std::cout << "\n\n Declaring a sam system:\n";

  auto syst = ::sam::System::SamSystem<sam::Factor::Anchor2d, sam::Factor::LinearTranslation2d>("A");

  ::sam::Factor::Anchor2d::measure_t z {0, 0};
  ::sam::Factor::Anchor2d::measure_cov_t      Sigma {{0.2, 0}, {0, 0.2}};

  syst.register_new_factor<::sam::Factor::Anchor2d>("f0", z, Sigma, {"x0"});
  syst.register_new_factor<::sam::Factor::LinearTranslation2d>(
      "f1",
      ::sam::Factor::LinearTranslation2d::measure_t {-0.95, 0.1},
      ::sam::Factor::LinearTranslation2d::measure_cov_t {{0.1, 0}, {0, 0.1}},
      {"x0", "x1"});

  syst.register_new_factor<::sam::Factor::LinearTranslation2d>(
      "f2",
      ::sam::Factor::LinearTranslation2d::measure_t {-0.01654, -1.21},
      ::sam::Factor::LinearTranslation2d::measure_cov_t {{0.02, 0}, {0, 0.3}},
      {"x1", "x2"});

  syst.register_new_factor<::sam::Factor::LinearTranslation2d>(
      "f3",
      ::sam::Factor::LinearTranslation2d::measure_t {1.01654, -.11},
      ::sam::Factor::LinearTranslation2d::measure_cov_t {{0.32, 0}, {0, 0.1}},
      {"x2", "x3"});

  // loop-closure
  syst.register_new_factor<::sam::Factor::LinearTranslation2d>(
      "f4",
      ::sam::Factor::LinearTranslation2d::measure_t {0.0, 1},
      ::sam::Factor::LinearTranslation2d::measure_cov_t {{0.002, 0}, {0, 0.002}},
      {"x3", "x0"});
  // syst.register_new_factor<::sam::Factor::LinearTranslation2d>(
  //     "f5",
  //     ::sam::Factor::LinearTranslation2d::measure_t {-1.01654, -0.8},
  //     ::sam::Factor::LinearTranslation2d::measure_cov_t {{0.2, 0}, {0, 0.17}},
  //     {"x0", "x2"});

  // std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << " Pre Optimized points: \n";
  auto sys_marginals = syst.get_marginals();
  std::cout << ::sam::Marginal::stringify_marginal_container(sys_marginals);
  try
  {
    syst.sam_optimize();
  }
  catch (const char* e)
  {
#if ENABLE_DEBUG_TRACE
    std::cerr << "SLAM algorithm failed. Reason: " << e << '\n';
#endif
  }
  std::cout << " After optimization: \n";
  sys_marginals = syst.get_marginals();
  std::cout << ::sam::Marginal::stringify_marginal_container(sys_marginals);

  return 0;
}
