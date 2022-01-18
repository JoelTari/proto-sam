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

  // AnchorFactor A;
  AnchorFactor::measure_vect_t m = {0, 0};
  AnchorFactor::measure_cov_t  cov;
  cov << 3.15, 0, 0, 1.09;
  LinearTranslationFactor::measure_vect_t m2
      = {-1, 0.1};   // Matrix<2,1> can be init by '='
  LinearTranslationFactor::measure_cov_t cov2 {
      {2.1, 0},
      {0, 0.008}};   // Matrix<2,2> can't haha

  auto FA = AnchorFactor("f0", m, cov, {"x0"},{});
  auto FB = LinearTranslationFactor("f1", m2, cov2, {"x0", "x1"},{});

  std::cout << "Printing runtime infos of a factor : \n\n";
  // factor_print(FA);
  // factor_print(FB);

  std::cout << "\nPrinting infos of a factor type (only static infos since it "
               "is just a type) : \n\n";
  // factor_print<AnchorFactor>();
  // factor_print<LinearTranslationFactor>();


  std::cout << "\n\n Declaring a sam system:\n";
  auto samsyst = SAM::SamSystem<AnchorFactor, LinearTranslationFactor>("A");

  // samsyst.register_new_factor<AnchorFactor>("f0", m,cov , {"x0"});
  // samsyst.register_new_factor<LinearTranslationFactor>("f1",m2,cov2,{"x0","x1"});
  samsyst.register_new_factor<AnchorFactor>(
      "f0",
      AnchorFactor::measure_vect_t { 0, 0},
      AnchorFactor::measure_cov_t {{1, 0}, {0, 1}},
      {"x0"});
  samsyst.register_new_factor<LinearTranslationFactor>(
      "f1",
      LinearTranslationFactor::measure_vect_t {0, 0},
      LinearTranslationFactor::measure_cov_t {{1, 0}, {0, 1}},
      {"x0", "x1"});
  samsyst.register_new_factor<LinearTranslationFactor>(
      "f2",
      LinearTranslationFactor::measure_vect_t {0, 0},
      LinearTranslationFactor::measure_cov_t {{1, 0}, {0, 1}},
      {"x1", "x2"});

    
  // samsyst.sam_optimize();

  std::cout << "\n NOTA: above result doesnt matter, point of this test is : compile, run/print \n";

  return 0;
}
