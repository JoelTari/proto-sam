#include "factor_impl/anchor.hpp"
#include "factor_impl/linear-translation.hpp"
#include "factor_impl/anchorSE2.hpp"
#include "factor_impl/motion-model-SE2.hpp"
#include "factor_impl/cartesian-landmark-obs-SE2.hpp"
#include "factor_impl/pose-matcher-SE2.hpp"

#include <gtest/gtest.h>

// TODO: move this to some sort of test utils
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

// TODO: move this to some sort of test utils
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
  std::apply([](const auto & ... Aik){
        ((std::cout << "Ai_k:\n" << Aik << '\n'), ...);
      },
      std::get<1>(biAiks));
}

//------------------------------------------------------------------//
//                            Anchor 2d                             //
//------------------------------------------------------------------//
TEST(Factors, Anchor2d){
  using TestedFactor_t = ::sam::Factor::Anchor2d;

  TestedFactor_t::measure_t              m    = {2.0, -1};
  TestedFactor_t::measure_cov_t            cov  = TestedFactor_t::measure_cov_t::Identity()*2; 
  auto F = TestedFactor_t("f0", m, cov, {"x0"}, {});
  auto proposalF = TestedFactor_t::make_composite({1, 0});

  double norm_F = F.factor_norm_at(proposalF);
  EXPECT_DOUBLE_EQ(norm_F, 1);
  
  {
    // FIX: remove after refactor
    auto [bi, Aiks] = F.compute_Ai_bi_linear();
  }

  // hardcoded test quantities that we expect
  TestedFactor_t::criterion_t expected_bi;
  std::tuple_element_t<0, TestedFactor_t::matrices_Aik_t> expected_Ai_anchored;
  expected_bi << 0.707107, -0.707107;
  expected_Ai_anchored << 0.707107,0,0,0.707107;
  auto expected_Ai_bi = std::make_tuple(expected_bi, std::make_tuple(expected_Ai_anchored));
  auto value_bi_Ai = F.compute_Ai_bi_at(proposalF);

  EXPECT_bi_Aiks(expected_Ai_bi, value_bi_Ai , 1e-4 );

  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  std::cout << " ---- \n";

  F.get_array_keys_id();
}

//------------------------------------------------------------------//
//                      Linear Translation 2d                       //
//------------------------------------------------------------------//
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

  {
    // FIX: remove after refactor
    auto [bi, Aiks] = F.compute_Ai_bi_linear();
  }

  // hardcoded test quantities that we expect
  TestedFactor_t::criterion_t expected_bi;
  std::tuple_element_t<0, TestedFactor_t::matrices_Aik_t> expected_Ai_sighted;
  std::tuple_element_t<1, TestedFactor_t::matrices_Aik_t> expected_Ai_observer;
  expected_bi << 2e-16,-0;
  expected_Ai_sighted << sqrt(2),0,0,sqrt(2);
  expected_Ai_observer << -sqrt(2),0,0,-sqrt(2);
  auto expected_Ai_bi = std::make_tuple(expected_bi, std::make_tuple(expected_Ai_sighted,expected_Ai_observer));
  auto value_bi_Ai = F.compute_Ai_bi_at(proposalF);
  EXPECT_bi_Aiks(expected_Ai_bi, value_bi_Ai , 1 );

  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();

  F.get_array_keys_id();
}

// WARNING: temporary, remove
TEST(Factors, eigen_matrices_equality) {
  Eigen::Matrix3d M1;
  M1.setIdentity()*2.5;
  Eigen::Matrix3d M2;
  M2.setIdentity()*2.5;

  EXPECT_TRUE(M1.isApprox(M2));
}

//------------------------------------------------------------------//
//                            Anchor SE2                            //
//------------------------------------------------------------------//
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
  // hardcoded test quantities that we expect
  TestedFactor_t::criterion_t expected_bi;
  std::tuple_element_t<0, TestedFactor_t::matrices_Aik_t> expected_Ai_anchored;
  expected_bi << -2.16828, 8.09212, 1.48096;
  expected_Ai_anchored <<-1.28255 , 0.74048 ,-3.85329,
                          -0.74048 ,-1.28255 ,-1.80356,
                                -0  ,      0 ,-1.41421;
  auto expected_Ai_bi = std::make_tuple(expected_bi, std::make_tuple(expected_Ai_anchored));
  auto value_bi_Ai = F.compute_Ai_bi_at(proposalF);
  print (value_bi_Ai);

  EXPECT_bi_Aiks(expected_Ai_bi, value_bi_Ai , 1e-3 );

  F.compute_Ai_bi_at(proposalF);
  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  // std::cout << " ---- \n";

  F.get_array_keys_id();
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

  // hardcoded test quantities that we expect
  TestedFactor_t::criterion_t expected_bi;
  std::tuple_element_t<0, TestedFactor_t::matrices_Aik_t> expected_Ai_subsequent;
  std::tuple_element_t<1, TestedFactor_t::matrices_Aik_t> expected_Ai_antecedent;
  expected_bi << -1.22479, -1.43881, -2.22144;
  expected_Ai_subsequent <<
                            -1.34076,     -0.55536,     0.731933,
                             0.55536,     -1.34076, -0.000440045,
                                   0,           -0,     -1.41421;
  expected_Ai_antecedent <<
                                 1.34076, -0.55536, 0.706878,
                                 0.55536,  1.34076, 0.189865,
                                       0,        0,  1.41421;
  auto expected_Ai_bi = std::make_tuple(expected_bi, std::make_tuple(expected_Ai_subsequent,expected_Ai_antecedent));
  auto value_bi_Ai = F.compute_Ai_bi_at(proposalF);
  EXPECT_bi_Aiks(expected_Ai_bi, value_bi_Ai , 1 );

  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  // std::cout << " ---- \n";

  F.get_array_keys_id();
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

  // hardcoded test quantities that we expect
  TestedFactor_t::criterion_t expected_bi;
  std::tuple_element_t<0, TestedFactor_t::matrices_Aik_t> expected_Ai_sighted;
  std::tuple_element_t<1, TestedFactor_t::matrices_Aik_t> expected_Ai_observer;
  expected_bi << sqrt(2)/2, -0.224744, -2.35516e-16;
  expected_Ai_sighted <<  -1.41421, -1.17757e-16,     0.112372,
                       1.17757e-16,     -1.41421,     0.353554,
                                 0,            0,     -1.41421;
  expected_Ai_observer << 1, -0.999999,   1.11237,
                   0.999999,         1,  0.353554,
                          0,         0,   1.41421;
  auto expected_Ai_bi = std::make_tuple(expected_bi, std::make_tuple(expected_Ai_sighted,expected_Ai_observer));
  auto value_bi_Ai = F.compute_Ai_bi_at(proposalF);
  EXPECT_bi_Aiks(expected_Ai_bi, value_bi_Ai , 1e-3 );

  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  // std::cout << " ---- \n";

  F.get_array_keys_id();
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
  
  // hardcoded test quantities that we expect
  TestedFactor_t::criterion_t expected_bi;
  std::tuple_element_t<0, TestedFactor_t::matrices_Aik_t> expected_Ai_sighted;
  std::tuple_element_t<1, TestedFactor_t::matrices_Aik_t> expected_Ai_observer;
  expected_bi << -1.51575, -5.65688;
  expected_Ai_sighted <<  -1.22475, -0.707106,
                          0.707106,  -1.22475;
  expected_Ai_observer << 1.41421,     0, 5.65685,
                            0, 1.41421, 9.79796;

  auto expected_Ai_bi = std::make_tuple(expected_bi, std::make_tuple(expected_Ai_sighted,expected_Ai_observer));
  auto value_bi_Ai = F.compute_Ai_bi_at(proposalF);
  EXPECT_bi_Aiks(expected_Ai_bi, value_bi_Ai , 1e-3 );

  print(F.compute_Ai_bi_at(proposalF));
  std::cout << sam::Factor::stringify_factor_blockliner(F);
  std::cout << " ---- \n";
  std::cout << sam::Factor::stringify_factor_blockliner<TestedFactor_t>();
  // std::cout << " ---- \n";

  F.get_array_keys_id();
}
// Also see this: 
// testing::internal::CaptureStdout();
// std::cout << "My test";
// std::string output = testing::internal::GetCapturedStdout();
