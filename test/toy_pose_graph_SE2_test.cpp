#define ENABLE_DEBUG_TRACE 1


#include "core/sam-system.h"
#include "factor_impl/anchorSE2.hpp"
#include "factor_impl/pose-matcher-SE2.hpp"

#include <gtest/gtest.h>
#include "test_utils.h"

using std::cout;

TEST(ToyPoseGraphSE2System, Square){
  // logger
  sam_utils::JSONLogger::Instance().beginSession("toy_pose_graph_SE2_test.cpp");
  // scoped Timer
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  // random
  std::default_random_engine       generator;
  std::normal_distribution<double> dist_normal_unit_variance {0, 1};

  // ground truth
  std::vector<::sam::Key::PoseSE2_t> true_poses;
  true_poses.emplace_back(0, 0, 0);                // x0 // TODO: vary from 0 0 0 
  true_poses.emplace_back(5, 0, MANIF_PI_2 / 3);   // x1
  true_poses.emplace_back(7.5, 2.5, MANIF_PI_2);   // x2
  true_poses.emplace_back(5, 5, MANIF_PI);         // x3
  true_poses.emplace_back(0, 5, -MANIF_PI_2);      // x4
  true_poses.emplace_back(0, 0, -MANIF_PI_4);      // x5

  cout << "\n\n Declaring a sam system:\n";
  auto sys = ::sam::System::SamSystem<::sam::Factor::AnchorSE2, ::sam::Factor::PoseMatcherSE2>(
      "PoseMatcherSLAM");

  //------------------------------------------------------------------//
  //                        specify the noises                        //
  //------------------------------------------------------------------//
  Eigen::Array<double, 3, 1> prior_sigmas, pose_matcher_sigmas, loop_closure_sigmas;
  prior_sigmas << 0.002, 0.002, 0.0000005;
  pose_matcher_sigmas << 0.2, 0.2, 0.1;
  loop_closure_sigmas << 0.02, 0.02, 0.01;

  Eigen::Matrix<double, 3, 3> cov_prior      = prior_sigmas.square().matrix().asDiagonal();
  Eigen::Matrix<double, 3, 3> chol_cov_prior = cov_prior.llt().matrixL();

  Eigen::Matrix<double, 3, 3> cov_pose_matcher = pose_matcher_sigmas.square().matrix().asDiagonal();
  Eigen::Matrix<double, 3, 3> chol_cov_pose_matcher = cov_pose_matcher.llt().matrixL();

  Eigen::Matrix<double, 3, 3> cov_loop_closure = loop_closure_sigmas.square().matrix().asDiagonal();
  Eigen::Matrix<double, 3, 3> chol_cov_loop_closure = cov_loop_closure.llt().matrixL();

  // anchor
  {
    manif::SE2Tangentd u_noisy_prior = manif::SE2Tangentd(-0.000, -0.002, +0.000);
    auto z     = true_poses[0] + u_noisy_prior;
    auto cov_z = cov_prior;
    // registration
    sys.register_new_factor<::sam::Factor::AnchorSE2>("f0", z, cov_z, {"x0"});
  }

  // odometry pose matcher  x0 to x1
  {
    auto true_Z = true_poses[1].inverse().compose(true_poses[0]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise = manif::SE2Tangentd(+0.093, +0.040, +0.212);
// rd meas pose_matcher :  +0.007, +0.149, +0.003
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    sys.register_new_factor<::sam::Factor::PoseMatcherSE2>("f1", z, cov_z, {"x0", "x1"});
  }

  // odometry pose matcher  x1 to x2
  {
    auto true_Z = true_poses[2].inverse().compose(true_poses[1]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise = manif::SE2Tangentd(+0.204, +0.157, -0.013);
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    sys.register_new_factor<::sam::Factor::PoseMatcherSE2>("f2", z, cov_z, {"x1", "x2"});
  }
  // odometry pose matcher  x2 to x3
  {
    auto true_Z = true_poses[3].inverse().compose(true_poses[2]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise (-0.137, +0.262, -0.094);
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    sys.register_new_factor<::sam::Factor::PoseMatcherSE2>("f3", z, cov_z, {"x2", "x3"});
  }
  // odometry pose matcher  x3 to x4
  {
    auto true_Z = true_poses[4].inverse().compose(true_poses[3]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise ( -0.155, -0.104, -0.202);
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    sys.register_new_factor<::sam::Factor::PoseMatcherSE2>("f4", z, cov_z, {"x3", "x4"});
  }
  // odometry pose matcher  x4 to x5
  {
    auto true_Z = true_poses[5].inverse().compose(true_poses[4]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise ( -0.179, -0.003, -0.018);
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    sys.register_new_factor<::sam::Factor::PoseMatcherSE2>("f5", z, cov_z, {"x4", "x5"});
  }
  // loop closure from x5 to x0
  {
    auto true_Z = true_poses[5].inverse().compose(true_poses[0]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif ::SE2Tangentd loop_closure_matcher_noise (-0.024,-0.217,+0.068);
    auto z     = true_Z + loop_closure_matcher_noise;
    auto cov_z = cov_loop_closure;
    // registration
    sys.register_new_factor<::sam::Factor::PoseMatcherSE2>("f6", z, cov_z, {"x0", "x5"});
  }


  std::cout << "Before Optimization:\n";
  auto sys_marginals = sys.get_marginals();
  auto expected_x5_init = ::sam::Key::PoseSE2_t(1.111, -2.229, -0.6704 );
  auto expected_x4_init = ::sam::Key::PoseSE2_t(0.5155, 2.916, -1.474 );
  auto expected_x3_init = ::sam::Key::PoseSE2_t(5.647, 3.504, 3.037 );
  auto expected_x2_init = ::sam::Key::PoseSE2_t(7.592, 0.6596, 1.372 );
  auto expected_x1_init = ::sam::Key::PoseSE2_t(4.792, -1.084, 0.3116 );
  auto expected_x0_init = ::sam::Key::PoseSE2_t(0, -0.002, 0 );

  auto all_positionSE2 = std::get<0>(sys_marginals);

  EXPECT_KEY_APPROX("x0", expected_x0_init, *all_positionSE2.find("x0")->second->mean_ptr);
  EXPECT_KEY_APPROX("x1", expected_x1_init, *all_positionSE2.find("x1")->second->mean_ptr);
  EXPECT_KEY_APPROX("x2", expected_x2_init, *all_positionSE2.find("x2")->second->mean_ptr);
  EXPECT_KEY_APPROX("x3", expected_x3_init, *all_positionSE2.find("x3")->second->mean_ptr);
  EXPECT_KEY_APPROX("x4", expected_x4_init, *all_positionSE2.find("x4")->second->mean_ptr);
  EXPECT_KEY_APPROX("x5", expected_x5_init, *all_positionSE2.find("x5")->second->mean_ptr);

  // std::cout << ::sam::Marginal::stringify_marginal_container_block(sys_marginals);
  // test map points

  sys.sam_optimize();

  all_positionSE2 = std::get<0>(sys_marginals);

  std::cout << "After Optimization:\n";
  sys_marginals = sys.get_marginals();
  auto expected_x5_map = ::sam::Key::PoseSE2_t(0.03214, 0.2128, -0.8544 );
  auto expected_x4_map = ::sam::Key::PoseSE2_t(0.4438, 5.26, -1.675 );
  auto expected_x3_map = ::sam::Key::PoseSE2_t(5.645, 4.689, 2.966 );
  auto expected_x2_map = ::sam::Key::PoseSE2_t(7.438, 1.594, 1.442 );
  auto expected_x1_map = ::sam::Key::PoseSE2_t(4.822, -0.4609, 0.4156 );
  auto expected_x0_map = ::sam::Key::PoseSE2_t(8.331e-21, -0.002, 5.627e-27 );

  EXPECT_KEY_APPROX("x0", expected_x0_map, *all_positionSE2.find("x0")->second->mean_ptr);
  EXPECT_KEY_APPROX("x1", expected_x1_map, *all_positionSE2.find("x1")->second->mean_ptr);
  EXPECT_KEY_APPROX("x2", expected_x2_map, *all_positionSE2.find("x2")->second->mean_ptr);
  EXPECT_KEY_APPROX("x3", expected_x3_map, *all_positionSE2.find("x3")->second->mean_ptr);
  EXPECT_KEY_APPROX("x4", expected_x4_map, *all_positionSE2.find("x4")->second->mean_ptr);
  EXPECT_KEY_APPROX("x5", expected_x5_map, *all_positionSE2.find("x5")->second->mean_ptr);

  // std::cout << ::sam::Marginal::stringify_marginal_container_block(sys_marginals);
}
