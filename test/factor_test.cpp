#include "factor_impl/anchor.hpp"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/linear-translation.hpp"


//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  //------------------------------------------------------------------//
  //                     TEST Factor Constructors                     //
  //------------------------------------------------------------------//
  // AnchorFactor A;
  AnchorFactor::criterion_t              m    = {2.0, -1};
  AnchorFactor::measure_cov_t            cov  = AnchorFactor::measure_cov_t::Identity()*2; 
  // note the *2 in the measure cov, when the measure cov eigenvalues increases, the factor norm at a given point will decrease
  // as it is proportional to the composite
  LinearTranslationFactor::criterion_t   m2   = {-1.0, 1.0};
  LinearTranslationFactor::measure_cov_t cov2 = LinearTranslationFactor::measure_cov_t::Identity()/2;

  auto FA = AnchorFactor("f0", m, cov, {"x0"}, {});
  auto FB = LinearTranslationFactor("f1", m2, cov2, {"x0", "x1"}, {});   // x0 sighted from x1
  // TODO: FA_NL FB_NL
  // TODO: more factor types

  //------------------------------------------------------------------//
  //           TEST query the factor norm at a given point            //
  //------------------------------------------------------------------//
  // make a tuple of X points (ptr to be precise) for the factors
  AnchorFactor::composite_state_ptr_t proposalFA = AnchorFactor::make_composite({1, 0});
  auto proposalFB = LinearTranslationFactor::make_composite({-3, -1}, {-2, -2});

  // this tests the methods : compute_h_at_x compute_r_at_x
  double norm_FA = FA.factor_norm_at(proposalFA);   // expected sqrt( (2-1)^2 + (-1-0)^2 ) = 1.41
  std::cout << "norm of FA at proposal {1, 0} ( result expected 1 ): " << norm_FA << '\n';
  double norm_FB = FB.factor_norm_at(proposalFB);   // expected 0, here the measure cov have no effect since H.X = z
  std::cout << "norm of FB at proposal (  {-3, -1}, {-2, -2} ), (exp: 0) : " << norm_FB << '\n';
  // TODO: test the other factors
  
  
  //------------------------------------------------------------------//
  //                       TEST compute Ai & bi                       //
  //------------------------------------------------------------------//
  FA.compute_Ai_bi_linear();
  FB.compute_Ai_bi_linear();
  FA.compute_Ai_bi_at(proposalFA);
  FB.compute_Ai_bi_at(proposalFB);

  std::cout << "compute Ai bi completed\n";

  //------------------------------------------------------------------//
  //         TEST query the factor at their current lin point         //
  //                       (stored internally)                        //
  //------------------------------------------------------------------//
  // TODO: - see what happens for linear FA / FB
  // TODO: - test the Factor::get_key_points() method


  //------------------------------------------------------------------//
  //                   TEST Initial guess deduction                   //
  //------------------------------------------------------------------//
  // TODO: test the variations of initial guess deduction

  //------------------------------------------------------------------//
  //                       TEST OF PRETTY PRINT                       //
  //------------------------------------------------------------------//
  FB.get_array_keys_id();

  std::cout << "Printing runtime infos of a factor : \n";
  factor_print(FA);
  factor_print(FB);

  std::cout << "\nPrinting infos of a factor type (only static infos since it "
               "is just a type) : \n\n";
  factor_print<AnchorFactor>();
  factor_print<LinearTranslationFactor>();


  std::cout << "\nAccess the 2nd component of the measurement embedded in the factor \n -> measure "
            << AnchorFactor::kMeasureComponentsName[1] << " = "
            << AnchorFactor::measure_meta_t::get_component<AnchorFactor::kMeasureComponentsName[1]>(
                   m)
            << "\n";

  // this also works (the constexpr is necessary, but then makes the const redundant)
  static constexpr char y[] = "y", x[]= "x";
  std::cout << "Get " << "y" << " component of measurement m : " 
            <<  AnchorFactor::measure_meta_t::get_component<y>(m) << '\n';
  AnchorFactor::measure_meta_t::get_component<x>(m);
  // this wouldnt pass static assertion
  // static constexpr char fake_comp[] = "zzz";
  // std::cout << AnchorFactor::measure_meta_t::get_component<fake_comp>(m) << '\n';

  // TODO: test the runtime versions of get_component
  //------------------------------------------------------------------//
  //                     TEST history management                      //
  //------------------------------------------------------------------//
  auto factors_histories_container
      = FactorsHistoriesContainer<AnchorFactor, LinearTranslationFactor>();
  auto factor_FA_history = FactorHistory<decltype(FA)>(FA.factor_id, {"x0"}, FA.z);
  FactorHistory<decltype(FA)> factor_FA_history_assignment_test = factor_FA_history;
  // auto cc_factor_FA_history = std::move(factor_FA_history); // OK
  factors_histories_container.insert_new_factor_history(FA.factor_id, FA);

  // std::cout << AnchorFactor::kN << '\n';
  // std::cout << LinearTranslationMetaFactor::kN << '\n';

  return 0;
}
