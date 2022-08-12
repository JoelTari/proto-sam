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
  auto samsyst = 
    ::sam::System::SamSystem<
                               ::sam::Factor::MotionModelSE2
                              , ::sam::Factor::AnchorSE2
                              , ::sam::Factor::LandmarkCartesianObsSE2
                            >(
      "slam system");

  return 0;
}
