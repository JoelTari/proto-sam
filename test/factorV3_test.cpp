#include "linear-translation.hpp"
#include "anchor.hpp"


//------------------------------------------------------------------//
//                               MAIN                               //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  Eigen::Matrix2d mymat((Eigen::Matrix2d() << 1, 2, 3, 4).finished());
  // AnchorFactor A;
  AnchorFactor::measure_vect_t            m;
  AnchorFactor::measure_cov_t             cov;
  LinearTranslationFactor::measure_vect_t m2;
  LinearTranslationFactor::measure_cov_t  cov2;

  auto FA = AnchorFactor("f0", m, cov, {"x0"});
  auto FB = LinearTranslationFactor("f1", m2, cov2, {"x0", "x1"});

  std::cout << "Printing runtime infos of a factor : \n";
  factor_print(FA);
  factor_print(FB);

  std::cout << "\nPrinting infos of a factor type (only static infos since it "
               "is just a type) : \n\n";
  factor_print<AnchorFactor>();
  factor_print<LinearTranslationFactor>();

  // std::cout << AnchorFactor::kN << '\n';
  // std::cout << LinearTranslationMetaFactor::kN << '\n';

  return 0;
}
