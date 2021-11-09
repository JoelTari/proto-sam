#include "config.h"
#include "factor_impl/anchor.h"
#include "factor_impl/linear-translation.h"
#include "sam-system.h"

#include <tuple>

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

  std::tuple<jacobian_matrix_t, measure_vector_t> compute_A_b_impl()
  {
    jacobian_matrix_t A;
    measure_vector_t  b;
    // FIX: add normed jacobian/linear A here
    return {A, b};
  }
};


//------------------------------------------------------------------//
//                        Anchor (2d) factor                        //
//------------------------------------------------------------------//
/**
 * @brief anchor (2d) factor implementation. Typically used for prior or gps on
 * 2d (R^2) variables
 */
static constexpr char                   kAnchorCategoryName[] = "anchor";
constexpr int                           kAnchorNbVar          = 1;
constexpr int                           kAnchorXDimTot        = 2;
constexpr std::array<int, kAnchorNbVar> kAnchorVarSizes       = {2};
constexpr int                           kAnchorMesDim         = 2;
using AnchorMeta_t = FactorMetaInfo<kAnchorNbVar,
                                    kAnchorXDimTot,
                                    kAnchorVarSizes,
                                    kAnchorMesDim>;

class AnchorFactor
    : public BaseFactor<AnchorFactor, AnchorMeta_t, kAnchorCategoryName>
{
  public:
  AnchorFactor(const std::string&              factor_id,
               const AnchorFactor::var_keys_t& var_names)
      : BaseFactor<AnchorFactor, AnchorMeta_t, kAnchorCategoryName>(factor_id,
                                                                    var_names)
  {
  }

  std::tuple<jacobian_matrix_t, measure_vector_t> compute_A_b_impl()
  {
    jacobian_matrix_t A;
    measure_vector_t  b;
    // FIX: add normed jacobian/linear A here
    return {A, b};
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

  auto firstFactor = AnchorFactor("f0", {"x0"});

  syst.smooth_and_map();

  return 0;
}
