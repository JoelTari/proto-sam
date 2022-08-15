#include "factor_impl/anchor.hpp"
#include "factor_impl/linear-translation.hpp"
#include "core/sam-system.h"

//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  // logger
  sam_utils::JSONLogger::Instance().beginSession("sam-system-factor_test.cpp");
  // scoped Timer
  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  // ::sam::Factor::Anchor2d A;
  ::sam::Factor::Anchor2d::criterion_t m = {0, 0};
  ::sam::Factor::Anchor2d::measure_cov_t  cov;
  cov << 3.15, 0, 0, 1.09;
  ::sam::Factor::LinearTranslation2d::criterion_t m2
      = {-1, 0.1};   // Matrix<2,1> can be init by '='
  ::sam::Factor::LinearTranslation2d::measure_cov_t cov2 {
      {2.1, 0},
      {0, 0.008}};   // Matrix<2,2> can't haha

  auto FA = ::sam::Factor::Anchor2d("f0", m, cov, {"x0"},{});
  auto FB = ::sam::Factor::LinearTranslation2d("f1", m2, cov2, {"x0", "x1"},{});

  std::cout << "Printing runtime infos of a factor : \n\n";

  std::cout << "\nPrinting infos of a factor type (only static infos since it "
               "is just a type) : \n\n";


  std::cout << "\n\n Declaring a sam system:\n";
  auto samsyst = ::sam::System::SamSystem<::sam::Factor::Anchor2d, ::sam::Factor::LinearTranslation2d>("A");

  // samsyst.register_new_factor<::sam::Factor::Anchor2d>("f0", m,cov , {"x0"});
  // samsyst.register_new_factor<::sam::Factor::LinearTranslation2d>("f1",m2,cov2,{"x0","x1"});
  samsyst.register_new_factor<::sam::Factor::Anchor2d>(
      "f0",
      ::sam::Factor::Anchor2d::criterion_t { 0, 0},
      ::sam::Factor::Anchor2d::measure_cov_t {{1, 0}, {0, 1}},
      {"x0"});
  samsyst.register_new_factor<::sam::Factor::LinearTranslation2d>(
      "f1",
      ::sam::Factor::LinearTranslation2d::criterion_t {0, 0},
      ::sam::Factor::LinearTranslation2d::measure_cov_t {{1, 0}, {0, 1}},
      {"x0", "x1"});
  samsyst.register_new_factor<::sam::Factor::LinearTranslation2d>(
      "f2",
      ::sam::Factor::LinearTranslation2d::criterion_t {0, 0},
      ::sam::Factor::LinearTranslation2d::measure_cov_t {{1, 0}, {0, 1}},
      {"x1", "x2"});

    
  // samsyst.sam_optimize();

  std::cout << "\n NOTA: above result doesnt matter, point of this test is : compile, run/print \n";

  return 0;
}
