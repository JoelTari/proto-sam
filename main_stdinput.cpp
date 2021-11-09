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
  LinearTranslationFactor(
      const std::string&                                          factor_id,
      const LinearTranslationFactor::var_keys_t&                  var_names,
      const LinearTranslationFactor::measure_vector_t&            measure,
      const LinearTranslationFactor::measure_covariance_matrix_t& covariance)
      : BaseFactor<LinearTranslationFactor,
                   linTranslMeta_t,
                   kLinearTranslationCategoryName>(factor_id,
                                                   var_names,
                                                   measure,
                                                   covariance)
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
  AnchorFactor(const std::string&                               factor_id,
               const AnchorFactor::var_keys_t&                  var_names,
               const AnchorFactor::measure_vector_t&            measure,
               const AnchorFactor::measure_covariance_matrix_t& covariance)
      : BaseFactor<AnchorFactor, AnchorMeta_t, kAnchorCategoryName>(factor_id,
                                                                    var_names,
                                                                    measure,
                                                                    covariance)
      , rho_(Eigen::LLT<AnchorFactor::measure_covariance_matrix_t>(
                 covariance.inverse())
                 .matrixU())
      , A_(rho_ * H_)
      , b_(rho_ * measure)
  {
    // (Hx-z)^T Sigma^-1 (Hx-z) = || Ax-b || _Sigma ^2
    // Define rho s.t. Sigma^-1 = rho^T * rho  => LLT decomposition of Sigma^-1
    // and consider the upper triangular matrix to obtain rho (Hx-z)^T Sigma^-1
    // (Hx-z) is (Hx-z)^T rho^T * rho (Hx-z)
    // => A = rho H  and b = rho z
  }

  std::tuple<AnchorFactor::jacobian_matrix_t, AnchorFactor::measure_vector_t>
      compute_A_b_impl()
  {
    return {A_, b_};
  }


  private:
  static const AnchorFactor::jacobian_matrix_t
      H_;   // NOTE: value declared out of line
  const AnchorFactor::jacobian_matrix_t rho_;
  const AnchorFactor::jacobian_matrix_t A_;
  const AnchorFactor::measure_vector_t  b_;
};

AnchorFactor::jacobian_matrix_t const AnchorFactor::H_ {{1, 0}, {0, 1}};

//------------------------------------------------------------------//
//                          Main function                           //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  PROFILE_FUNCTION();
  auto syst = SAM::SamSystem<AnchorFactor, LinearTranslationFactor>();

  // receive the measurement from stdin (as a string that can be converted in a
  // C++ container)

  AnchorFactor::measure_vector_t            z {0, 0};
  AnchorFactor::measure_covariance_matrix_t Omega {{0.2, 0}, {0.2}};
  auto firstFactor = AnchorFactor("f0", {"x0"}, z, Omega);

  syst.register_new_factor<AnchorFactor>("f0", {"x0"}, z, Omega);

  syst.smooth_and_map();

  return 0;
}
