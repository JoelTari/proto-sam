#include <factor_impl/anchorSE2.hpp>

int main (int argc, char *argv[])
{
  // testing some declarations
  MetaKeyPose_SE2_t A;
  MetaMeasureAbsolutePoseSE2_t M;
  UniqueSE2KeyConduct_t U("test_kcc", AnchorSE2Factor::measure_cov_t::Identity());
  AnchorSE2Factor::measure_t z (5,-5,1.57);
  AnchorSE2Factor::measure_cov_t cov_z ( AnchorSE2Factor::measure_cov_t::Identity()/3 );
  AnchorSE2Factor::composite_state_ptr_t init_point_ptr = { std::make_shared<UniqueSE2KeyConduct_t::Key_t>(0,0,0) };
  auto FA = AnchorSE2Factor("f0",z,cov_z,{"x0"},init_point_ptr );
  std::cout << "Printing the factor: \n";
  factor_print(FA) ;
  std::cout << "Printing any anchor SE2 factor: \n";
  factor_print<AnchorSE2Factor>();

  auto Xq =   AnchorSE2Factor::make_composite({4,-5,0}) ;
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
