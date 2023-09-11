/* 
 * Copyright 2023 AKKA Technologies and LAAS-CNRS (joel.tari@akka.eu) 
 * 
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by 
 * the European Commission - subsequent versions of the EUPL (the "Licence"); 
 * You may not use this work except in compliance with the Licence. 
 * You may obtain a copy of the Licence at: 
 * 
 * https://joinup.ec.europa.eu/software/page/eupl 
 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the Licence is distributed on an "AS IS" basis, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the Licence for the specific language governing permissions and 
 * limitations under the Licence. 
 */
 
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
