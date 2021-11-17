#include "config.h"
#include "factor.h"
#include "sam-system.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <tuple>

// TODO: move the factors explicit instantiations to factor_impl/{{factor-name}}.hpp

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
      , rho_(Eigen::LLT<LinearTranslationFactor::measure_covariance_matrix_t>(
                 covariance.inverse())
                 .matrixU())
      , A_(rho_ * k_H)
      , b_(rho_ * measure)
  {
  }

  std::tuple<prediction_matrix_t, measure_vector_t> compute_A_b_impl()
  {
    return {A_, b_};
  }

  private:
  static const LinearTranslationFactor::prediction_matrix_t
                                                             k_H;   // NOTE: value declared out of line
  const LinearTranslationFactor::measure_covariance_matrix_t rho_;
  const LinearTranslationFactor::prediction_matrix_t         A_;
  const LinearTranslationFactor::measure_vector_t            b_;
};

LinearTranslationFactor::prediction_matrix_t const
    LinearTranslationFactor::k_H {{1, 0, -1, 0}, {0, 1, 0, -1}};

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
      , A_(rho_ * k_H)
      , b_(rho_ * measure)
  {
    // (Hx-z)^T Sigma^-1 (Hx-z) = || Ax-b || _Sigma ^2
    // Define rho s.t. Sigma^-1 = rho^T * rho  => LLT decomposition of Sigma^-1
    // and consider the upper triangular matrix to obtain rho (Hx-z)^T Sigma^-1
    // (Hx-z) is (Hx-z)^T rho^T * rho (Hx-z)
    // => A = rho H  and b = rho z
  }

  std::tuple<AnchorFactor::prediction_matrix_t, AnchorFactor::measure_vector_t>
      compute_A_b_impl()
  {
    return {A_, b_};
  }


  private:
  static const AnchorFactor::prediction_matrix_t
                                                  k_H;   // NOTE: value declared out of line
  const AnchorFactor::measure_covariance_matrix_t rho_;
  const AnchorFactor::prediction_matrix_t         A_;
  const AnchorFactor::measure_vector_t            b_;
};

AnchorFactor::prediction_matrix_t const AnchorFactor::k_H {{1, 0}, {0, 1}};

//------------------------------------------------------------------//
//                          Main function                           //
//------------------------------------------------------------------//
int main(int argc, char* argv[])
{
  sam_utils::JSONLogger::Instance().beginSession("main_session");

  PROFILE_FUNCTION(sam_utils::JSONLogger::Instance());

  auto syst = SAM::SamSystem<AnchorFactor, LinearTranslationFactor>();

  // receive the measurement from stdin (as a string that can be converted in a
  // C++ container)

  AnchorFactor::measure_vector_t            z {0, 0};
  AnchorFactor::measure_covariance_matrix_t Sigma {{0.2, 0}, {0, 0.2}};

  syst.register_new_factor<AnchorFactor>("f0", {"x0"}, z, Sigma);
  syst.register_new_factor<LinearTranslationFactor>(
      "f1",
      {"x0", "x1"},
      LinearTranslationFactor::measure_vector_t {-0.95, 0.1},
      LinearTranslationFactor::measure_covariance_matrix_t {{0.1, 0},
                                                            {0, 0.1}});

  syst.register_new_factor<LinearTranslationFactor>(
      "f2",
      {"x1", "x2"},
      LinearTranslationFactor::measure_vector_t {-0.01654, -1.21},
      LinearTranslationFactor::measure_covariance_matrix_t {{0.02, 0},
                                                            {0, 0.3}});

  syst.register_new_factor<LinearTranslationFactor>(
      "f3",
      {"x2", "x3"},
      LinearTranslationFactor::measure_vector_t {1.01654, -.11},
      LinearTranslationFactor::measure_covariance_matrix_t {{0.32, 0},
                                                            {0, 0.1}});

  // loop-closure
  syst.register_new_factor<LinearTranslationFactor>(
      "f4",
      {"x3", "x0"},
      LinearTranslationFactor::measure_vector_t {0.01654, 1.181},
      LinearTranslationFactor::measure_covariance_matrix_t {{0.002, 0},
                                                            {0, 0.173}});
  syst.register_new_factor<LinearTranslationFactor>(
      "f5",
      {"x0", "x2"},
      LinearTranslationFactor::measure_vector_t {-1.01654, -0.8},
      LinearTranslationFactor::measure_covariance_matrix_t {{0.2, 0},
                                                            {0, 0.17}});

  std::this_thread::sleep_for(std::chrono::seconds(1));

  try
  {
    syst.smooth_and_map();
  }
  catch (const char* e)
  {
#if ENABLE_DEBUG_TRACE
    std::cerr << "SLAM algorithm failed. Reason: " << e << '\n';
#endif
  }

  return 0;
}
