#include "factor_impl/anchor.hpp"
#include "factor_impl/linear-translation.hpp"

#include <gtest/gtest.h>

TEST(Factors, Anchor2d){
  using TestedFactor_t = ::sam::Factor::Anchor2d;

  TestedFactor_t::criterion_t              m    = {2.0, -1};
  TestedFactor_t::measure_cov_t            cov  = TestedFactor_t::measure_cov_t::Identity()*2; 
  auto F = TestedFactor_t("f0", m, cov, {"x0"}, {});
  auto proposalF = TestedFactor_t::make_composite({1, 0});

  double norm_F = F.factor_norm_at(proposalF);   // expected sqrt( (2-1)^2 + (-1-0)^2 ) = 1.41
  EXPECT_DOUBLE_EQ(norm_F, 1);
  
  F.compute_Ai_bi_linear();
  F.compute_Ai_bi_at(proposalF);
  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  std::cout << " ---- \n";

  F.get_array_keys_id();
}

TEST(Factors, LinearTranslation2d) {
  using TestedFactor_t = ::sam::Factor::LinearTranslation2d;

  TestedFactor_t::criterion_t   m   = {-1.0, 1.0};
  auto cov = TestedFactor_t::measure_cov_t::Identity()/2;

  auto F = TestedFactor_t("f1", m, cov, {"x0", "x1"}, {});   // x0 sighted from x1
  auto proposalF = TestedFactor_t::make_composite({-3, -1}, {-2, -2});

  double norm_F = F.factor_norm_at(proposalF);
  std::cout << " ---- \n";
  EXPECT_NEAR(norm_F, 0, 1e-4);
  std::cout << " ---- \n";

  F.compute_Ai_bi_linear();
  F.compute_Ai_bi_at(proposalF);
  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  std::cout << " ---- \n";

  F.get_array_keys_id();
}

// TODO: do the same for all instanciated factor types
// - pose matcher SE2
// - motion model SE2
// - anchor SE2
// - cartesian landmark observation SE2 -> 2d
//
// workflow : run the test once, note the norms, and use as basis for futur comparaison EXPECT_NEAR
//
// Also see this: 
// testing::internal::CaptureStdout();
// std::cout << "My test";
// std::string output = testing::internal::GetCapturedStdout();
