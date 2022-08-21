#include "factor_impl/anchor.hpp"
#include "factor_impl/linear-translation.hpp"
#include "factor_impl/anchorSE2.hpp"
#include "factor_impl/motion-model-SE2.hpp"
#include "factor_impl/cartesian-landmark-obs-SE2.hpp"
#include "factor_impl/pose-matcher-SE2.hpp"

#include <gtest/gtest.h>

template <typename TUP_Aiks>
void EXPECT_TUPLE_OF_MATRIX_APPROX(const TUP_Aiks & expected, const TUP_Aiks & value, double p)
{
  // tup zip pattern
  std::apply(
      [&](const auto & ...val )
      {
        std::apply(
            [&](const auto & ...exp)
            {
               //  (EXPECT_TRUE(exp.isApprox(val,p)) , ...  ) ;
               EXPECT_TRUE( (exp.isApprox(val,p) && ... ));
            }
            , expected
            );
      }
      , value
      );
}

template <typename VEC_bi, typename TUP_Aiks>
void EXPECT_bi_Aiks(const std::tuple<VEC_bi,TUP_Aiks> & expected, const std::tuple<VEC_bi,TUP_Aiks> & value, double p)
{
  auto expected_bi = std::get<0>(expected);
  auto value_bi = std::get<0>(value);
  auto expected_Aiks =std::get<1>(expected);
  auto value_Aiks =std::get<1>(value);
  EXPECT_TRUE(value_bi.isApprox(expected_bi,p));
  EXPECT_TUPLE_OF_MATRIX_APPROX(expected_Aiks,value_Aiks,p);
}


template <typename VEC_bi, typename TUP_Aiks>
void print(const std::tuple<VEC_bi, TUP_Aiks> & biAiks)
{
  std::cout << "bi : \n" << std::get<0>(biAiks).transpose() << '\n';
  std::apply([](const auto && ... Aik){
        ((std::cout << "Ai_k:\n" << Aik << '\n'), ...);
      },
      std::get<1>(biAiks));
}

TEST(Factors, Anchor2d){
  using TestedFactor_t = ::sam::Factor::Anchor2d;

  TestedFactor_t::measure_t              m    = {2.0, -1};
  TestedFactor_t::measure_cov_t            cov  = TestedFactor_t::measure_cov_t::Identity()*2; 
  auto F = TestedFactor_t("f0", m, cov, {"x0"}, {});
  auto proposalF = TestedFactor_t::make_composite({1, 0});

  double norm_F = F.factor_norm_at(proposalF);   // expected sqrt( (2-1)^2 + (-1-0)^2 ) = 1.41
  EXPECT_DOUBLE_EQ(norm_F, 1);
  
  // TODO:  hard code expected bi, Aiks
  auto [bi, Aiks] = F.compute_Ai_bi_linear();
  // print(F.compute_Ai_bi_linear());

  F.compute_Ai_bi_at(proposalF);
  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  std::cout << " ---- \n";

  F.get_array_keys_id();
  std::cout << " ============================================================= \n";
}

TEST(Factors, LinearTranslation2d) {
  using TestedFactor_t = ::sam::Factor::LinearTranslation2d;

  TestedFactor_t::measure_t   z   = {-1.0, 1.0};
  auto cov = TestedFactor_t::measure_cov_t::Identity()/2;

  auto F = TestedFactor_t("f1", z, cov, {"x0", "x1"}, {});   // x0 sighted from x1
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

  F.get_array_keys_id();
  std::cout << " ============================================================= \n";
}

// WARNING: temporary, remove
TEST(Factors, eigen_matrices_equality) {
  Eigen::Matrix3d M1;
  M1.setIdentity()*2.5;
  Eigen::Matrix3d M2;
  M2.setIdentity()*2.5;

  EXPECT_TRUE(M1.isApprox(M2));
  std::cout << " ============================================================= \n";
}

TEST(Factors, AnchorSE2){
  using TestedFactor_t = ::sam::Factor::AnchorSE2;

  auto  z   = TestedFactor_t::measure_t( -1.0, 1.0, 0 );
  auto cov = TestedFactor_t::measure_cov_t::Identity()/2;

  auto init_point = TestedFactor_t::make_composite({z});
  auto F_tmp = TestedFactor_t("f0", z, cov, {"x0"}, {}); // no init point (rvalue)
  auto F = TestedFactor_t("f0bis", z, cov, {"x0"}, init_point); 
  // proposal :  xyt = -5,5, pi/3
  auto proposalF = TestedFactor_t::make_composite({-5, 5, 3.14159/3});

  double norm_F = F.factor_norm_at(proposalF);
  std::cout << " ---- \n";
  double expected_norm_F = 8.50747;
  EXPECT_NEAR(norm_F, expected_norm_F, 1e-4);
  std::cout << " ---- \n";
  //
  F.compute_Ai_bi_at(proposalF);
  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  // std::cout << " ---- \n";

  F.get_array_keys_id();
  std::cout << " ============================================================= \n";
}

TEST(Factors, MotionModelSE2){
  using TestedFactor_t = ::sam::Factor::MotionModelSE2;

  auto  z   = TestedFactor_t::measure_t( 1.0, 0, 3.14159/4 ); // vx vy w
  auto cov = TestedFactor_t::measure_cov_t::Identity()/2;

  auto proposalF = TestedFactor_t::make_composite({-5, 5, 3.14159/3}, {-6,5, 7*3.14159/12} );
  auto F = TestedFactor_t("f1", z, cov, {"x0", "x1"}, {}); 
  // proposal :  xyt = -5,5, pi/3

  double norm_F = F.factor_norm_at(proposalF);
  std::cout << " ---- \n";
  double expected_norm_F = 2.91635;
  EXPECT_NEAR(norm_F, expected_norm_F, 1e-4);
  std::cout << " ---- \n";
  //
  F.compute_Ai_bi_at(proposalF);
  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  // std::cout << " ---- \n";

  F.get_array_keys_id();
  std::cout << " ============================================================= \n";
}

TEST(Factors, PoseMatcherSE2){
  using TestedFactor_t = ::sam::Factor::PoseMatcherSE2;

  auto  z   = TestedFactor_t::measure_t( -.5, -.5, -3.14159/4 ); // SE2(x,y,t)
  auto cov = TestedFactor_t::measure_cov_t::Identity()/2;

  auto proposalF = TestedFactor_t::make_composite({-5, 5, 3.14159/3}, {-6,5, 7*3.14159/12} );
  auto F = TestedFactor_t("f1", z, cov, {"x0","x1"}, {}); 
  // proposal :  xyt = -5,5, pi/3

  double norm_F = F.factor_norm_at(proposalF);
  std::cout << " ---- \n";
  double expected_norm_F = 0.741965;
  EXPECT_NEAR(norm_F, expected_norm_F, 1e-4);
  std::cout << " ---- \n";
  //
  F.compute_Ai_bi_at(proposalF);
  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  // std::cout << " ---- \n";

  F.get_array_keys_id();
  std::cout << " ============================================================= \n";
}


TEST(Factors, CartesianLandmarkObsSE2){
  using TestedFactor_t = ::sam::Factor::LandmarkCartesianObsSE2;

  auto  z   = TestedFactor_t::measure_t( 8, 0 ); // (dx, dy)
  auto cov = TestedFactor_t::measure_cov_t::Identity()/2;

  auto proposalF = TestedFactor_t::make_composite({0,0}, {-8,0,3.14159/6} );
  auto F = TestedFactor_t("f1", z, cov, {"l0","x1"}, {}); 
  // proposal :  xyt = -5,5, pi/3

  double norm_F = F.factor_norm_at(proposalF);
  std::cout << " ---- \n";
  double expected_norm_F = 5.8564;
  EXPECT_NEAR(norm_F, expected_norm_F, 1e-4);
  std::cout << " ---- \n";
  //
  F.compute_Ai_bi_at(proposalF);
  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  // std::cout << " ---- \n";

  F.get_array_keys_id();
  std::cout << " ============================================================= \n";
}
// Also see this: 
// testing::internal::CaptureStdout();
// std::cout << "My test";
// std::string output = testing::internal::GetCapturedStdout();
