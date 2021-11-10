#include "factor.h"
#include <iostream>


//------------------------------------------------------------------//
//                  Odom factor type instantiation                  //
//------------------------------------------------------------------//
static constexpr char             kFactorCategory[] = "SE2-odometry";
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
  OdomFactor(const std::string& factor_id,
             const std::array<std::string, OdomFactor::Meta_t::kNumberOfVars>&
                                                            var_names,
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
  IniFactor(const std::string& factor_id,
            const std::array<std::string, IniFactor::Meta_t::kNumberOfVars>&
                                                          var_names,
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

//------------------------------------------------------------------//
//              Bearing-only factor type instantiation              //
//------------------------------------------------------------------//
static constexpr char              kFactorCategory3[] = "Bearing-Only";
constexpr int                      kNbVar3            = 2;
constexpr int                      kXDimTot3          = 5;
constexpr std::array<int, kNbVar3> kVarSizes3         = {3, 2};
constexpr int                      kMesDim3           = 1;
using metaBearing_t = FactorMetaInfo<kNbVar3, kXDimTot3, kVarSizes3, kMesDim3>;
/**
 * @brief Factor for bearing only
 */
class bearingFactor
    : public BaseFactor<bearingFactor, metaBearing_t, kFactorCategory3>
{
  public:
  bearingFactor(
      const std::string& factor_id,
      const std::array<std::string, bearingFactor::Meta_t::kNumberOfVars>&
                                                        var_names,
      const bearingFactor::measure_vector_t&            measure,
      const bearingFactor::measure_covariance_matrix_t& covariance)
      : BaseFactor<bearingFactor, metaBearing_t, kFactorCategory3>(factor_id,
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
//             Range-bearing factor type instantiation              //
//------------------------------------------------------------------//
static constexpr char              kFactorCategory4[] = "Range-Bearing";
constexpr int                      kNbVar4            = 2;
constexpr int                      kXDimTot4          = 5;
constexpr std::array<int, kNbVar4> kVarSizes4         = {3, 2};
constexpr int                      kMesDim4           = 2;
using metaRangeBearing_t
    = FactorMetaInfo<kNbVar4, kXDimTot4, kVarSizes4, kMesDim4>;
/**
 * @brief Factor for range-bearing (pose toward position)
 */
class rangeBearingFactor
    : public BaseFactor<rangeBearingFactor,
                        metaRangeBearing_t,
                        kFactorCategory4>
{
  public:
  rangeBearingFactor(
      const std::string& factor_id,
      const std::array<std::string, rangeBearingFactor::Meta_t::kNumberOfVars>&
                                                             var_names,
      const rangeBearingFactor::measure_vector_t&            measure,
      const rangeBearingFactor::measure_covariance_matrix_t& covariance)
      : BaseFactor<rangeBearingFactor, metaRangeBearing_t, kFactorCategory4>(
          factor_id,
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
//           Orientation-fixed range bearing factor type            //
//                     Position toward position                     //
//           Orientation is given wrt the world main axis           //
//------------------------------------------------------------------//
static constexpr char              kFactorCategory5[] = "orFixed-Range-Bearing";
constexpr int                      kNbVar5            = 2;
constexpr int                      kXDimTot5          = 4;
constexpr std::array<int, kNbVar5> kVarSizes5         = {2, 2};
constexpr int                      kMesDim5           = 2;
using metaRangeBearingSliding_t
    = FactorMetaInfo<kNbVar5, kXDimTot5, kVarSizes5, kMesDim5>;
/**
 * @brief Factor for fixed orientation range-bearing
 */
class rangeBearingSlideFactor
    : public BaseFactor<rangeBearingSlideFactor,
                        metaRangeBearingSliding_t,
                        kFactorCategory5>
{
  public:
  rangeBearingSlideFactor(
      const std::string&                                          factor_id,
      const std::array<std::string, Meta_t::kNumberOfVars>&       var_names,
      const rangeBearingSlideFactor::measure_vector_t&            measure,
      const rangeBearingSlideFactor::measure_covariance_matrix_t& covariance)
      : BaseFactor<rangeBearingSlideFactor,
                   metaRangeBearingSliding_t,
                   kFactorCategory5>(factor_id, var_names, measure, covariance)
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

template <typename FACTOR_T>
void printSomeMetaInfo(const FACTOR_T& factor);

// some quick and dirty print helper function
template <typename FACTOR_T>
void printSomeMetaInfo(const FACTOR_T& factor)
{
  std::cout << "\n *****  Printing Meta of "
            << factor.kFactorCategory   // FACTOR_T::kFactorCategory
            << " factor " << factor.factor_id << " of scope ("
            << StringifyArrayOfStrings(factor.variables) << ") ***** \n";

  std::cout << "Automated var idx ranges (in the factor state vector): \n";

  for (auto vrange : FACTOR_T::Meta_t::kVarIdxRanges)
    std::cout << vrange[0] << "," << vrange[1] << "\t";
  std::cout << "\n";

  for (const auto& var : factor.variables)
  {
    std::cout
        << var << " -> ["
        << FACTOR_T::Meta_t::kVarIdxRanges[factor.variable_position.at(var)][0]
        << ":"
        << FACTOR_T::Meta_t::kVarIdxRanges[factor.variable_position.at(var)][1]
        << "]"
        << "\n";
  }
}

int main()
{
  OdomFactor::measure_vector_t            zf1 {0, 0, 0.5};
  OdomFactor::measure_covariance_matrix_t Sigmaf1 {{1, 0, 0},
                                                   {0, 1, 0},
                                                   {0, 0, 1}};
  OdomFactor                  aFactor("f1", {"x1", "x2"}, zf1, Sigmaf1);
  IniFactor::measure_vector_t zf2 {0, 0, 0};
  IniFactor::measure_covariance_matrix_t Sigmaf2 {{1, 0, 0},
                                                  {0, 1, 0},
                                                  {0, 0, 1}};
  IniFactor                              bFactor("f2", {"x0"}, zf2, Sigmaf2);
  // bearingFactor           cFactor("f3", {"x2", "l1"}, {3.14159 / 4}, {0.2});
  rangeBearingFactor dFactor(
      "f4",
      {"x3", "l2"},
      rangeBearingFactor::measure_vector_t {2.01, 0.41},
      rangeBearingFactor::measure_covariance_matrix_t {{0.15, 0}, {0, 0.03}});
  rangeBearingSlideFactor eFactor(
      "f5",
      {"x8", "x9"},
      rangeBearingSlideFactor::measure_vector_t {2.01, 0.41},
      rangeBearingSlideFactor::measure_covariance_matrix_t {{0.15, 0},
                                                            {0, 0.03}});

  printSomeMetaInfo(aFactor);
  printSomeMetaInfo(bFactor);
  // printSomeMetaInfo(cFactor);
  printSomeMetaInfo(dFactor);
  printSomeMetaInfo(eFactor);

  // query test
  try
  {
    std::cout << "Variable ordering of x1 is : "
              << aFactor.variable_position.at("x1") << '\n';
  }
  catch (std::out_of_range e)
  {
    throw e;
  }

  return 0;
}