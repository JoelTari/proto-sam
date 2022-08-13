#define ENABLE_DEBUG_TRACE 1

#include <iomanip> // std::precision
#include <iostream>
#include "core/sam-system.h"
#include "factor_impl/anchorSE2.hpp"
#include "factor_impl/cartesian-landmark-obs-SE2.hpp"
#include "factor_impl/motion-model-SE2.hpp"

using std::cout, std::endl;

int main (int argc, char *argv[])
{
  // logger
  sam_utils::JSONLogger::Instance().beginSession("toy_pose_graph_SE2_test.cpp");
  // scoped Timer
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  // DEBUG INFO
  cout << endl;
  cout << "2D Smoothing and Mapping. 3 poses, 5 landmarks." << endl;
  cout << " This problem setting originates from the manif library examples." << endl;
  cout << "                  ------- b1        " << endl;
  cout << "          b3    /         |         " << endl;
  cout << "          |   /       b4  |         " << endl;
  cout << "          | /       /    \\|         " << endl;
  cout << "          X0 ---- X1 ---- X2        " << endl;
  cout << "          | \\   /   \\   /           " << endl;
  cout << "          |   b0      b2            " << endl;
  cout << "          *                         " << endl;
  cout << "-----------------------------------------------" << endl;
  cout << std::fixed << std::setprecision(4) << std::showpos;
  // cout << "SE2 Dof " << manif::SE2d::DoF << '\n';
  // cout << "SE2 Dim " << manif::SE2d::Dim << '\n';
  
  //------------------------------------------------------------------//
  //                    Ground truth for landmarks                    //
  //------------------------------------------------------------------//
  std::vector< ::sam::Key::Position2d_t > landmarks_simu;
  {
    ::sam::Key::Position2d_t b0, b1, b2, b3, b4;
    b0 << 3.0, 0.0;
    b1 << 2.0, -1.0;
    b2 << 2.0, 1.0;
    b3 << 3.0, -1.0;
    b4 << 3.0, 1.0;
    landmarks_simu.push_back(b0 * 5);
    landmarks_simu.push_back(b1 * 5);
    landmarks_simu.push_back(b2 * 5);
    landmarks_simu.push_back(b3 * 5);
    landmarks_simu.push_back(b4 * 5);
  }

  //------------------------------------------------------------------//
  //                      Ground truth for poses                      //
  //------------------------------------------------------------------//
  int                      NUMPOSES = 3;
  std::vector<::sam::Key::PoseSE2_t> true_poses;
  // applied velocities per time step : vx = 3, omega = -0.4
  ::sam::Meta::Key::PoseSE2::tangent_space_t       u_nom;
  u_nom << 3, 0, -.4;
  // u_nom     << 0.1, 0.0, 0.05;

  // initial pose
  true_poses.emplace_back(10, 1, -MANIF_PI / 6);
  // true_poses.emplace_back(0, 0, 0);
  // applying recursively a control
  for (int i = 1; i < NUMPOSES; i++) true_poses.push_back(true_poses[i - 1] + u_nom);

  
  cout << "\n\n Declaring a sam system:\n";
  auto sys = 
    ::sam::System::SamSystem<
                               ::sam::Factor::MotionModelSE2
                              , ::sam::Factor::AnchorSE2
                              , ::sam::Factor::LandmarkCartesianObsSE2
                            >(
      "slam system");
  
  cout << "\n\n factors registration:\n";
  // covariances
  Eigen::Array<double, 3, 1> prior_sigmas, control_sigmas;
  Eigen::Array<double, 2, 1> landmark_meas_sigmas;
  prior_sigmas << 0.000002, 0.000002, 0.0000005;
  control_sigmas << 0.5, 0.0003, 0.1;
  landmark_meas_sigmas << 0.1, 0.1;
  ::sam::Factor::AnchorSE2::measure_cov_t cov_prior    = prior_sigmas.square().matrix().asDiagonal();
  ::sam::Factor::MotionModelSE2::measure_cov_t  cov_control  = control_sigmas.square().matrix().asDiagonal();
  ::sam::Factor::LandmarkCartesianObsSE2::measure_cov_t cov_landmark = landmark_meas_sigmas.square().matrix().asDiagonal();

  // f0
  {
    auto z = ::sam::Factor::AnchorSE2::measure_t(true_poses[0]);   // or ::sam::Measure::AbsolutePosition2d is the same
    sys.register_new_factor<::sam::Factor::AnchorSE2>("f0",z,cov_prior,{"x0"});
  }
  // f1
  {
    auto z = ::sam::Measure::LinearTranslation2d_t(4.8335, 1.7085);
    sys.register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f1", z , cov_landmark, {"b0","x0"});
  }
  // f2
  {
    auto z = ::sam::Measure::LinearTranslation2d_t(3.0034,-5.2488);
    sys.register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f2", z , cov_landmark, {"b1","x0"});
  }
  // f3
  {
    auto z = ::sam::Measure::LinearTranslation2d_t(7.3764, -2.6761);
    sys.register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f3", z , cov_landmark, {"b3","x0"});
  }
  // f4
  {
    auto z = ::sam::Measure::VelocitySE2(4.0623, 0.0001, -0.2979);
    sys.register_new_factor<::sam::Factor::MotionModelSE2>("f4", z , cov_control, {"x1","x0"});
  }
  // f5
  {
    auto z = ::sam::Measure::LinearTranslation2d_t(0.8791, 2.7602);
    sys.register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f5", z , cov_landmark, {"b0","x1"});
  }
  // f6
  {
    auto z = ::sam::Measure::LinearTranslation2d_t(-6.1805,1.9506);
    sys.register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f6", z , cov_landmark, {"b2","x1"});
  }
  // f7
  {
    auto z = ::sam::Measure::LinearTranslation2d_t(-3.1906,5.5414);
    sys.register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f7", z , cov_landmark, {"b4","x1"});
  }
  // f8
  {
    auto z = ::sam::Measure::VelocitySE2(2.6136, -0.0002, -0.6015); // vx.t , vy.t , w.t
    sys.register_new_factor<::sam::Factor::MotionModelSE2>("f8", z , cov_control, {"x2","x1"});
  }
  // f9
  {
    auto z = ::sam::Measure::LinearTranslation2d_t(0.3480,-3.7441);
    sys.register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f9", z , cov_landmark, {"b1","x2"});
  }
  // f10
  {
    auto z = ::sam::Measure::LinearTranslation2d_t(-9.2770,-1.3639);
    sys.register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f10", z , cov_landmark, {"b2","x2"});
  }
  // f11
  {
    auto z = ::sam::Measure::LinearTranslation2d_t(-8.0425,3.6379);
    sys.register_new_factor<::sam::Factor::LandmarkCartesianObsSE2>("f11", z , cov_landmark, {"b4","x2"});
  }

  sys.sam_optimize();

  return 0;
}
