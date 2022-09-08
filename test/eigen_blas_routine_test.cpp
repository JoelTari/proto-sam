#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>

// Various calls to Eigen
// Before running this code, set the environment variable
//       export MKL_VERBOSE=1
// if you expect the MKL calls, then they should appear
// in the traces.

TEST(EigenBLASSuite, BasicBLASTest)
{
  std::stringstream ss;

  auto m1 = Eigen::Matrix<double, 10, 10>::Random();
  auto m2 = Eigen::Matrix<double, 10, 10>::Random();
  auto b  = Eigen::Vector<double, 10>::Random();

  ss << m1.adjoint() * b << std::endl;
  ss << m1.selfadjointView<Eigen::Upper>() * b << std::endl;

  auto v1 = m1.lu().solve(b);
  ss << v1 << std::endl;

  Eigen::EigenSolver<Eigen::Matrix<double, 10, 10>> es(m2);

  auto qr = m1.householderQr();
  ss << qr.solve(b) << std::endl;

  ss << m2.selfadjointView<Eigen::Upper>().llt().solve(b) << std::endl;

  // TODO:  PardisoLLT


  // std::cout << ss;
}
