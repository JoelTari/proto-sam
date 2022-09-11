// #define ENABLE_DEBUG_TRACE 1

#include "anchor2d/anchor2d.h"
#include "relative-matcher-2d/relative-matcher-2d.h"
#include "system/sam-system.h"
#include "test_utils.h"

#include <gtest/gtest.h>


//------------------------------------------------------------------//
TEST(ToyLinearSystem, Square)
{
  // logger
  // std::string result_filename
  //     = sam_utils::currentDateTime() + "_results_toy_linear_test.json";
  sam_utils::JSONLogger::Instance().beginSession("gtest_2d", "toy_square_linear_2d");

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

  // scoped Timer
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  std::cout << "\n\n Declaring a sam system:\n";

  auto syst = ::sam::Inference::System<sam::Factor::Anchor2d, sam::Factor::RelativeMatcher2d>("A");

  ::sam::Factor::Anchor2d::measure_t     z {0, 0};
  ::sam::Factor::Anchor2d::measure_cov_t Sigma {{0.002, 0}, {0, 0.002}};

  syst.register_new_factor<::sam::Factor::Anchor2d>("f0", z, Sigma, {"x0"});
  syst.register_new_factor<::sam::Factor::RelativeMatcher2d>(
      "f1",
      ::sam::Factor::RelativeMatcher2d::measure_t {-0.95, 0.1},
      ::sam::Factor::RelativeMatcher2d::measure_cov_t {{0.8, 0}, {0, 0.5}},
      {"x0", "x1"});

  syst.register_new_factor<::sam::Factor::RelativeMatcher2d>(
      "f2",
      ::sam::Factor::RelativeMatcher2d::measure_t {-0.01654, -1.21},
      ::sam::Factor::RelativeMatcher2d::measure_cov_t {{0.5, 0}, {0, 0.8}},
      {"x1", "x2"});

  syst.register_new_factor<::sam::Factor::RelativeMatcher2d>(
      "f3",
      ::sam::Factor::RelativeMatcher2d::measure_t {1.01654, -.11},
      ::sam::Factor::RelativeMatcher2d::measure_cov_t {{0.8, 0}, {0, 0.5}},
      {"x2", "x3"});

  // loop-closure
  syst.register_new_factor<::sam::Factor::RelativeMatcher2d>(
      "f4",
      ::sam::Factor::RelativeMatcher2d::measure_t {0.0, 1},
      ::sam::Factor::RelativeMatcher2d::measure_cov_t {{0.002, 0}, {0, 0.002}},
      {"x3", "x0"});

  // std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << " Pre Optimised points: \n";
  auto sys_marginals = syst.get_marginals();
  std::cout << ::sam::Marginal::stringify_marginal_container_block(sys_marginals);

  try
  {
    syst.sam_optimise();
  }
  catch (const char* e)
  {
#if ENABLE_DEBUG_TRACE
    std::cerr << "SLAM algorithm failed. Reason: " << e << '\n';
#endif
  }
  std::cout << " After optimisation: \n";
  sys_marginals = syst.get_marginals();   // map {keyid : marginal} (tuple of maps, because
                                          // marginals are not all the same type)
  std::cout << ::sam::Marginal::stringify_marginal_container_block(sys_marginals);

  // test: print all marginals, register values, hardcode them, and google test them
  // after optim
  auto expected_x3map = ::sam::Key::Spatial2d_t(-4.757e-05, 1);
  auto expected_x2map = ::sam::Key::Spatial2d_t(0.9975, 0.9513);
  auto expected_x1map = ::sam::Key::Spatial2d_t(0.969, -0.161);
  auto expected_x0map = ::sam::Key::Spatial2d_t(-4.034e-18, 1.862e-16);

  // sys_marginals is a 1-uple, let's simplify
  auto all_position2d = std::get<0>(sys_marginals);


  std::cout << ::sam::Meta::Key::Spatial2d::stringify_key_oneliner(
      *all_position2d.find("x3")->second.shared_mean)
            << '\n';

  EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>("x0",
                                                 expected_x0map,
                                                 *all_position2d.find("x0")->second.shared_mean,1e3); // 1000s precision, since we are close zero (1e-19), and the way the precision works, this is ok
  EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>("x1",
                                                 expected_x1map,
                                                 *all_position2d.find("x1")->second.shared_mean);
  EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>("x2",
                                                 expected_x2map,
                                                 *all_position2d.find("x2")->second.shared_mean);
  EXPECT_KEY_APPROX<::sam::Meta::Key::Spatial2d>("x3",
                                                 expected_x3map,
                                                 *all_position2d.find("x3")->second.shared_mean);

  // additional test: remove f5
  int nbfactors_before = 
    std::apply([](const auto & ...vect)
        { 
          return (vect.size() + ...) ;  
        }
        ,syst.get_all_factors());

  syst.remove_factor("f2");

  int nbfactors_after = 
    std::apply([](const auto & ...vect)
        { 
          return (vect.size() + ...) ;  
        }
        ,syst.get_all_factors());

  EXPECT_EQ(nbfactors_before, nbfactors_after+1);


  // print all the factors
  std::cout << sam::Factor::stringify_wrapped_factor_container_block(syst.get_all_factors());
}
