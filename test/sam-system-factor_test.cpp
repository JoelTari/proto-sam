#include "factor.h"
#include "sam-system.h"

// struct DummyFactor1{
//   static const std::string factor_typename;
//   std::string factor_id;
// };

//------------------------------------------------------------------//
//                  Odom factor type instantiation                  //
//------------------------------------------------------------------//
static constexpr char             kFactorCategory[] = "Odom-Factor";
constexpr int                     kNbVar            = 2;
constexpr int                     kXDimtOt          = 6;
constexpr std::array<int, kNbVar> kVarSizes         = {3, 3};
constexpr int                     kMesDim           = 3;
using metaOdom_t = FactorMetaInfo<kNbVar, kXDimtOt, kVarSizes, kMesDim>;

/**
 * @brief Factor odometry
 */
class OdomFactor : public BaseFactor<OdomFactor, metaOdom_t, kFactorCategory>
{
  public:
  OdomFactor(const std::string&                             factor_id,
             const OdomFactor::var_keys_t&                  var_names,
             const OdomFactor::measure_vector_t&            measure,
             const OdomFactor::measure_covariance_matrix_t& covariance)
      : BaseFactor<OdomFactor, metaOdom_t, kFactorCategory>(factor_id,
                                                            var_names,
                                                            measure,
                                                            covariance)
  {
  }

  std::tuple<prediction_matrix_t, measure_vector_t> compute_A_b_impl()
  {
    prediction_matrix_t A;
    measure_vector_t  b;
    // FIX: add normed jacobian/linear A here
    return {A, b};
  }
};

//------------------------------------------------------------------//
//                Initial pose factor instantiation                 //
//------------------------------------------------------------------//
static constexpr char              kFactorCategory2[] = "SO2-prior";
constexpr int                      kNbVar2            = 1;
constexpr int                      kXDimTot2          = 3;
constexpr std::array<int, kNbVar2> kVarSizes2         = {3};
constexpr int                      kMesDim2           = 3;
using metaIni_t = FactorMetaInfo<kNbVar2, kXDimTot2, kVarSizes2, kMesDim2>;
/**
 * @brief Factor initial pose (2.5d)
 */
class IniFactor : public BaseFactor<IniFactor, metaIni_t, kFactorCategory2>
{
  public:
  IniFactor(const std::string&                            factor_id,
            const IniFactor::var_keys_t&                  var_names,
            const IniFactor::measure_vector_t&            measure,
            const IniFactor::measure_covariance_matrix_t& covariance)
      : BaseFactor<IniFactor, metaIni_t, kFactorCategory2>(factor_id,
                                                           var_names,
                                                           measure,
                                                           covariance)
  {
  }

  std::tuple<prediction_matrix_t, measure_vector_t> compute_A_b_impl()
  {
    prediction_matrix_t A;
    measure_vector_t  b;
    // FIX: add normed jacobian/linear A here
    return {A, b};
  }
};

class BS
{
  public:
  BS(int a, double b) {}
};

// some quick and dirty print helper function
int main(int argc, char* argv[])
{
  // scoped Timer
  PROFILE_FUNCTION();
  // create the sam system
  auto samsyst1 = SAM::SamSystem<IniFactor, OdomFactor>();
  // create some factors
  auto f0 = IniFactor("f0a",
                      {"x0a"},
                      IniFactor::measure_vector_t {0, 0, 0},
                      IniFactor::measure_covariance_matrix_t {{1, 0, 0},
                                                              {0, 1, 0},
                                                              {0, 0, .1}});

  // std::vector<IniFactor> inif;
  // std::vector<BS> vBS;
  // vBS.emplace_back(2, 3.158691);
  // IniFactor::var_keys_t var_names = {"x0"};
  // inif.emplace_back("f0",var_names);
  // inif.emplace_back("f0",{"x0"});

  // auto f1 = OdomFactor("f1", {"x0","x1"});
  // auto f2 = OdomFactor("f2", {"x1","x2"});
  // register the factors
  samsyst1.register_new_factor<IniFactor>(
      "f0",
      IniFactor::var_keys_t {"x0"},
      IniFactor::measure_vector_t {0, 0, 0},
      IniFactor::measure_covariance_matrix_t {{1, 0, 0},
                                              {0, 1, 0},
                                              {0, 0, .1}});
  samsyst1.register_new_factor<OdomFactor>(
      "f1",
      OdomFactor::var_keys_t {"x0", "x1"},
      OdomFactor::measure_vector_t {0, 0, 0},
      OdomFactor::measure_covariance_matrix_t {{1, 0, 0},
                                               {0, 1, 0},
                                               {0, 0, .1}});
  samsyst1.register_new_factor<OdomFactor>(
      "f2",
      OdomFactor::var_keys_t {"x1", "x2"},
      OdomFactor::measure_vector_t {0, 0, 0},
      OdomFactor::measure_covariance_matrix_t {{1, 0, 0},
                                               {0, 1, 0},
                                               {0, 0, .1}});
  // NOTE: the xxFactor::var_keys_t{...}, filling only the initializer_list
  // {...} creates problem with forwarding

  // print all infos about the factors in the sam system
  samsyst1.IterFactorInfos();

  // samsyst1.loop_over_factors<dosmthi>();
  samsyst1.smooth_and_map();

  return 0;
}
