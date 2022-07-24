#include "factor_impl/anchor.hpp"
#include "factor_impl/key-meta-position.h"
#include "factor_impl/linear-translation.hpp"


//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  // AnchorFactor A;
  AnchorFactor::criterion_t            m = {2.0, -1};
  AnchorFactor::measure_cov_t             cov;
  LinearTranslationFactor::criterion_t m2 = {-1, 0.1};
  LinearTranslationFactor::measure_cov_t  cov2;

  auto FA = AnchorFactor("f0", m, cov, {"x0"}, {});
  auto FB = LinearTranslationFactor("f1", m2, cov2, {"x0", "x1"}, {});

  AnchorFactor::composite_state_ptr_t proposalFA;
  std::get<0>(proposalFA) = std::make_shared<MetaKeyPosition_t::key_t>(MetaKeyPosition_t::key_t{1,0}); 
  LinearTranslationFactor::composite_state_ptr_t proposalFB
    = { std::make_shared<MetaKeyPosition_t::key_t>(MetaKeyPosition_t::key_t{0,0}), std::make_shared<MetaKeyPosition_t::key_t>(MetaKeyPosition_t::key_t{-1,3})};

  // this tests the methods : compute_h_at_x compute_r_at_x
  double norm_FA =FA.factor_norm_at(proposalFA);
  // std::cout << 
  double norm_FB = FB.factor_norm_at(proposalFB);

  std::cout << "Printing runtime infos of a factor : \n";
  factor_print(FA);
  factor_print(FB);

  std::cout << "\nPrinting infos of a factor type (only static infos since it "
               "is just a type) : \n\n";
  factor_print<AnchorFactor>();
  factor_print<LinearTranslationFactor>();

  //------------------------------------------------------------------//
  //                     TEST history management                      //
  //------------------------------------------------------------------//
  auto factors_histories_container
      = FactorsHistoriesContainer<AnchorFactor, LinearTranslationFactor>();

  std::cout << "\nAccess the 2nd component of the measurement embedded in the factor \n -> measure "
            << AnchorFactor::kMeasureComponentsName[1] << " = "
            << AnchorFactor::measure_meta_t::get_component<AnchorFactor::kMeasureComponentsName[0]>(
                   m)
            << "\n";

  // this also works
  static constexpr const char ycomp[]  ("y");
  std::cout << AnchorFactor::measure_meta_t::get_component<ycomp>(m);

  auto                        factor_FA_history = FactorHistory<decltype(FA)>(FA.factor_id, {"x0"},FA.z);
  FactorHistory<decltype(FA)> factor_FA_history_assignment_test = factor_FA_history;
  // auto cc_factor_FA_history = std::move(factor_FA_history); // OK
  factors_histories_container.insert_new_factor_history(FA.factor_id, FA);

  // std::cout << AnchorFactor::kN << '\n';
  // std::cout << LinearTranslationMetaFactor::kN << '\n';

  return 0;
}
