#include <iostream>
#include <Eigen/Dense>

#include <gtest/gtest.h>

// Various calls to Eigen
// Before running this code, set the environment variable
//       export MKL_VERBOSE=1
// if you expect the MKL calls, then they should appear 
// in the traces.

TEST(EigenBLASSuite, BasicBLASTest)
{

  auto m1 = Eigen::Matrix<double, 10, 10>::Random();
  auto m2 = Eigen::Matrix<double, 10, 10>::Random();
  auto b = Eigen::Vector<double,10>::Random();

  std::cout << m1.adjoint()*b 
    << std::endl;
  std::cout << 
    m1.selfadjointView<Eigen::Upper>()*b
    << std::endl;

  auto v1 = m1.lu().solve(b);
  std::cout<< v1 << std::endl;

  Eigen::EigenSolver<Eigen::Matrix<double,10,10>> es(m2);
  
  auto qr = m1.householderQr();
  std::cout << qr.solve(b)
    << std::endl;

  std::cout <<
    m2.selfadjointView<Eigen::Upper>().llt().solve(b)
    << std::endl;

  // TODO:  PardisoLLT
}
