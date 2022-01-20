#include "factor_impl/linear-translation.hpp"
#include "factor_impl/anchor.hpp"


//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  // AnchorFactor A;
  AnchorFactor::measure_vect_t            m={0,0};
  AnchorFactor::measure_cov_t             cov;
  LinearTranslationFactor::measure_vect_t m2={-1,0.1};
  LinearTranslationFactor::measure_cov_t  cov2;

  auto FA = AnchorFactor("f0", m, cov, {"x0"},{});
  auto FB = LinearTranslationFactor("f1", m2, cov2, {"x0", "x1"},{});

  std::cout << "Printing runtime infos of a factor : \n";
  factor_print(FA);
  factor_print(FB);

  std::cout << "\nPrinting infos of a factor type (only static infos since it "
               "is just a type) : \n\n";
  factor_print<AnchorFactor>();
  factor_print<LinearTranslationFactor>();

  auto factors_histories_container = FactorsHistoriesContainer<AnchorFactor,LinearTranslationFactor>();
  

  auto factor_FA_history = FactorHistory<decltype(FA)>(FA.factor_id, {"x0"} );
  FactorHistory<decltype(FA)> factor_FA_history_assignment_test = factor_FA_history;
  // auto cc_factor_FA_history = std::move(factor_FA_history); // OK
  factors_histories_container.insert_new_factor_history(FA.factor_id, FA);

  // std::cout << AnchorFactor::kN << '\n';
  // std::cout << LinearTranslationMetaFactor::kN << '\n';

  return 0;
}
