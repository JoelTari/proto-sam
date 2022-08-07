#define ENABLE_DEBUG_TRACE 1

#include "core/sam-system.h"
#include "factor_impl/anchorSE2.hpp"
#include "factor_impl/pose-matcher-SE2.hpp"

#include <memory>
#include <random>
using std::cout;

template <std::size_t N>
Eigen::Vector<double, N> sample_nmv_u_vector(std::default_random_engine& generator)
{
  std::normal_distribution<double> dist {0, 1};
  Eigen::Vector<double, N>         X;
  for (int i = 0; i < N; i++) X[i] = dist(generator);
  return X;
}

int main(int argc, char* argv[])
{
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
  auto samsyst = ::sam::System::SamSystem<::sam::Factor::AnchorSE2, ::sam::Factor::PoseMatcherSE2>(
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
    // TODO: maths check
    manif::SE2Tangentd u_noisy_prior
        = chol_cov_prior * sample_nmv_u_vector<3>(generator);   // + 0, mean is
    auto z     = true_poses[0] + u_noisy_prior;
    auto cov_z = cov_prior;
    // registration
    samsyst.register_new_factor<::sam::Factor::AnchorSE2>("f0", z, cov_z, {"x0"});
  }

  // odometry pose matcher  x0 to x1
  {
    auto true_Z = true_poses[1].inverse().compose(true_poses[0]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise
        = chol_cov_pose_matcher * sample_nmv_u_vector<3>(generator);
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    samsyst.register_new_factor<::sam::Factor::PoseMatcherSE2>("f1", z, cov_z, {"x0", "x1"});
  }

  // odometry pose matcher  x1 to x2
  {
    auto true_Z = true_poses[2].inverse().compose(true_poses[1]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise
        = chol_cov_pose_matcher * sample_nmv_u_vector<3>(generator);
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    samsyst.register_new_factor<::sam::Factor::PoseMatcherSE2>("f2", z, cov_z, {"x1", "x2"});
  }
  // odometry pose matcher  x2 to x3
  {
    auto true_Z = true_poses[3].inverse().compose(true_poses[2]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise
        = chol_cov_pose_matcher * sample_nmv_u_vector<3>(generator);
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    samsyst.register_new_factor<::sam::Factor::PoseMatcherSE2>("f3", z, cov_z, {"x2", "x3"});
  }
  // odometry pose matcher  x3 to x4
  {
    auto true_Z = true_poses[4].inverse().compose(true_poses[3]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise
        = chol_cov_pose_matcher * sample_nmv_u_vector<3>(generator);
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    samsyst.register_new_factor<::sam::Factor::PoseMatcherSE2>("f4", z, cov_z, {"x3", "x4"});
  }
  // odometry pose matcher  x4 to x5
  {
    auto true_Z = true_poses[5].inverse().compose(true_poses[4]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif::SE2Tangentd pose_matcher_noise
        = chol_cov_pose_matcher * sample_nmv_u_vector<3>(generator);
    auto z     = true_Z + pose_matcher_noise;
    auto cov_z = cov_pose_matcher;
    // registration
    samsyst.register_new_factor<::sam::Factor::PoseMatcherSE2>("f5", z, cov_z, {"x4", "x5"});
  }
  // loop closure from x5 to x0
  {
    auto true_Z = true_poses[5].inverse().compose(true_poses[0]);
    // noisy pose matcher: noise applied in the right tangent space (rplus)
    manif ::SE2Tangentd loop_closure_matcher_noise
        = chol_cov_loop_closure * sample_nmv_u_vector<3>(generator);
    auto z     = true_Z + loop_closure_matcher_noise;
    auto cov_z = cov_loop_closure;
    // registration
    samsyst.register_new_factor<::sam::Factor::PoseMatcherSE2>("f6", z, cov_z, {"x0", "x5"});
  }


  samsyst.sam_optimize();

  cout << "\n NOTA: above result doesnt matter, point of this test is : compile, run/print \n";

  return 0;
}
