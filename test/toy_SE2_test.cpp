#include "factor_impl/measure-meta-absolute-pose-SE2.h"
#include <factor_impl/anchorSE2.hpp>

int main (int argc, char *argv[])
{
  // testing some declarations
  ::sam::Meta::Key::PoseSE2 A;
  ::sam::Meta::Measure::AbsolutePoseSE2 M;
  // UniqueSE2KeyConduct_t U("test_kcc", sam::Factor::AnchorSE2::measure_cov_t::Identity());
  ::sam::Factor::AnchorSE2::measure_t z (5,-5,1.57);
  ::sam::Factor::AnchorSE2::measure_cov_t cov_z ( ::sam::Factor::AnchorSE2::measure_cov_t::Identity()/3 );
  ::sam::Factor::AnchorSE2::composite_state_ptr_t init_point_ptr = { std::make_shared<::sam::Key::PoseSE2_t>(0,0,0) }; // WARNING: not the proper way: use make_composite() method
  auto FA = ::sam::Factor::AnchorSE2("f0",z,cov_z,{"x0"},init_point_ptr );
  std::cout << "Printing the factor: \n";
  factor_print(FA) ;
  std::cout << "Printing any anchor SE2 factor: \n";
  factor_print<::sam::Factor::AnchorSE2>();

  auto Xq =   ::sam::Factor::AnchorSE2::make_composite({4,-5,0}) ;
  double norm_value = FA.factor_norm_at(Xq);

  std::cout << "Norm of FA at Xq: " << // Xq << '\n';
            norm_value << "\n";
  
  auto tup_bi_Ai = FA.compute_Ai_bi_at(Xq);

  std::cout << "Computed Ai bi at Xq. \n" << // Xq << '\n';
             "bi : \n"
               << std::get<0>(tup_bi_Ai) << "\n"
               << "Ai: \n"
               << std::get<0>(std::get<1>(tup_bi_Ai))
               << "\n -------- \n";

  return 0;
}
