#include "config.h"
#include "factor_impl/anchor.h"
#include "factor_impl/linear-translation.h"
#include "sam-system.h"

//------------------------------------------------------------------//
//                  Linear-translation factor                       //
//------------------------------------------------------------------//
/**
 * @brief linear-translation (2d) factor implementation. Typically used for
 * odometry or observation between 2 2d (R^2) variables
 */
static constexpr char kLinearTranslationCategoryName[] = "linear-translation";
constexpr int         kLinearTranslationNbVar          = 2;
constexpr int         kLinearTranslationXDimTot        = 4;
constexpr std::array<int, kLinearTranslationNbVar> kLinearTranslationVarSizes
    = {2, 2};
constexpr int kLinearTranslationMesDim = 2;

using linTranslMeta_t = FactorMetaInfo<kLinearTranslationNbVar,
                                       kLinearTranslationXDimTot,
                                       kLinearTranslationVarSizes,
                                       kLinearTranslationMesDim>;

/**
 * @brief Linear Translation Factor
 */
class LinearTranslationFactor
    : public BaseFactor<LinearTranslationFactor,
                        linTranslMeta_t,
                        kLinearTranslationCategoryName>
{
  public:
  LinearTranslationFactor(const std::string&                         factor_id,
                          const LinearTranslationFactor::var_keys_t& var_names)
      : BaseFactor<LinearTranslationFactor,
                   linTranslMeta_t,
                   kLinearTranslationCategoryName>(factor_id, var_names)
  {
  }
};


//------------------------------------------------------------------//
//                          Main function                           //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  PROFILE_FUNCTION();
  auto syst = SAM::SamSystem<AnchorFactor, LinearTranslationFactor>();

  // receive the measurement from stdin (as a string that can be converted in a
  // C++ container)

  AnchorFactor test("f0", {"x0"});

  return 0;
}
