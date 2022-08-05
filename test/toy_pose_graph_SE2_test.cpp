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
  std::default_random_engine generator;
  std::normal_distribution<double> dist_normal_unit_variance {0, 1};

  // ground truth
  std::vector<::sam::Key::PoseSE2_t> true_poses;
  true_poses.emplace_back(0, 0, 0);   // TODO: vary from 0 0 0
  true_poses.emplace_back(5, 0, MANIF_PI_2 / 3);
  true_poses.emplace_back(7.5, 2.5, MANIF_PI_2);
  true_poses.emplace_back(5, 5, MANIF_PI);
  true_poses.emplace_back(0, 5, -MANIF_PI_2);
  true_poses.emplace_back(0, 0, -MANIF_PI_4);


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

    Eigen::Matrix<double, 3, 3> cov_prior        = prior_sigmas.square().matrix().asDiagonal();
    Eigen::Matrix<double, 3, 3> cov_pose_matcher = pose_matcher_sigmas.square().matrix().asDiagonal();
    Eigen::Matrix<double, 3, 3> cov_loop_closure = loop_closure_sigmas.square().matrix().asDiagonal();


  {
    auto z           = ::sam::Factor::AnchorSE2::measure_t(0, 0, 0);
    auto cov_z       = ::sam::Factor::AnchorSE2::measure_cov_t::Identity()/2.5;
    // auto init_point_ptr = ::sam::Factor::AnchorSE2::make_composite ( ::sam::Key::PoseSE2_t(1,0,0)  );
    // registration
    samsyst.register_new_factor<::sam::Factor::AnchorSE2>( "f0", z, cov_z, {"x0"} );
  }

  {
    auto z           = ::sam::Factor::PoseMatcherSE2::measure_t(-1, 0, 0);
    auto cov_z       = ::sam::Factor::PoseMatcherSE2::measure_cov_t::Identity()/2.5;
    // auto init_point_ptr = ::sam::Factor::PoseMatcherSE2::make_composite ( ::sam::Key::PoseSE2_t(1,0,0)  );
    // registration
    samsyst.register_new_factor<::sam::Factor::PoseMatcherSE2>( "f1", z, cov_z, {"x0","x1"} );
  }

  // ::sam::Factor::AnchorSE2::measure_cov_t::Identity()/2 , {"x0"});
  // // samsyst.register_new_factor<::sam::Factor::LinearTranslation2d>("f1",m2,cov2,{"x0","x1"});
  // samsyst.register_new_factor<::sam::Factor::PoseMatcherSE2>(
  //     "f0",
  //     ::sam::Factor::Anchor2d::criterion_t { 0, 0},
  //     ::sam::Factor::Anchor2d::measure_cov_t {{1, 0}, {0, 1}},
  //     {"x0"});
  // samsyst.register_new_factor<::sam::Factor::LinearTranslation2d>(
  //     "f1",
  //     ::sam::Factor::LinearTranslation2d::criterion_t {0, 0},
  //     ::sam::Factor::LinearTranslation2d::measure_cov_t {{1, 0}, {0, 1}},
  //     {"x0", "x1"});
  // samsyst.register_new_factor<::sam::Factor::LinearTranslation2d>(
  //     "f2",
  //     ::sam::Factor::LinearTranslation2d::criterion_t {0, 0},
  //     ::sam::Factor::LinearTranslation2d::measure_cov_t {{1, 0}, {0, 1}},
  //     {"x1", "x2"});


  samsyst.sam_optimize();

  cout << "\n NOTA: above result doesnt matter, point of this test is : compile, run/print \n";

  return 0;
}
