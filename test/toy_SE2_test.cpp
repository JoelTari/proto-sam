#include "factor_impl/key-meta-SE2.h"
#include "factor_impl/measure-meta-absolute-pose-SE2.h"
#include <factor_impl/anchorSE2.hpp>
#include <factor_impl/motion-model-SE2.hpp>
#include <factor_impl/pose-matcher-SE2.hpp>

int main (int argc, char *argv[])
{
  // testing some declarations
  ::sam::Meta::Key::PoseSE2 A;
  ::sam::Meta::Measure::AbsolutePoseSE2 M;

  std::cout << "\n ------------------------- \n";
  std::cout << "\n -------Anchor SE2-------- \n";
  std::cout << "\n ------------------------- \n";

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


  std::cout << "\n ------------------------- \n";
  std::cout << "\n ----Motion Model SE2----- \n";
  std::cout << "\n ------------------------- \n";
  // Motion Model SE2
  ::sam::Factor::MotionModelSE2::measure_t u (2, 0, -MANIF_PI/3);
  ::sam::Factor::MotionModelSE2::measure_cov_t cov_u ( ::sam::Factor::MotionModelSE2::measure_cov_t::Identity()/3 );
  auto query_factor_point = ::sam::Factor::MotionModelSE2::make_composite( sam::Key::PoseSE2_t (5,5,0 ), sam::Key::PoseSE2_t(0,0,MANIF_PI/4)  );

  auto factor_motion_model = sam::Factor::MotionModelSE2("f1", u, cov_u,{"x0","x1"}, query_factor_point);

  double norm_value2 = factor_motion_model.factor_norm_at(query_factor_point);
  auto tup_bi_Ai_f1 = factor_motion_model.compute_Ai_bi_at(query_factor_point);

  std::cout << "Printing the factor: \n";
  factor_print(factor_motion_model) ;
  std::cout << "Printing any Motion Model SE2 factor: \n";
  factor_print<::sam::Factor::MotionModelSE2>();

  std::cout << "Norm of motion model factor at Xq: " << // Xq << '\n';
            norm_value2 << "\n";
  

  std::cout << "Computed Ai bi at Xq. \n" << // Xq << '\n';
             "bi : \n"
               << std::get<0>(tup_bi_Ai) << "\n";
               // << "Ai: \n"
               // << std::get<0>(std::get<1>(tup_bi_Ai))
               // << "\n -------- \n";

  std::cout << "\n ------------------------- \n";
  std::cout << "\n ----Pose Matcher SE2----- \n";
  std::cout << "\n ------------------------- \n";

  // ::sam::Factor::

  return 0;
}
