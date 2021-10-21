#include "factor.h"

#include <array>
#include <iostream>

//------------------------------------------------------------------//
//                  Odom factor type instantiation                  //
//------------------------------------------------------------------//
constexpr int                     kNbVar    = 2;
constexpr int                     kXDimtOt  = 6;
constexpr std::array<int, kNbVar> kVarSizes = {3, 3};
constexpr int                     kMesDim   = 3;
using metaOdom_t = FactorMetaInfo<kNbVar, kXDimtOt, kVarSizes, kMesDim>;

/**
 * @brief Factor odometry
 */
class OdomFactor : public BaseFactor<OdomFactor, metaOdom_t>
{
  public:
  OdomFactor(const std::string& factor_id,
             const std::array<std::string, OdomFactor::Meta_t::kNumberOfVars>&
                 var_names)
      : BaseFactor<OdomFactor, metaOdom_t>(factor_id, var_names)
  {
  }
};

//------------------------------------------------------------------//
//                Initial pose factor instantiation                 //
//------------------------------------------------------------------//
constexpr int                      kNbVar2    = 1;
constexpr int                      kXDimTot2  = 3;
constexpr std::array<int, kNbVar2> kVarSizes2 = {3};
constexpr int                      kMesDim2   = 3;
using metaIni_t = FactorMetaInfo<kNbVar2, kXDimTot2, kVarSizes2, kMesDim2>;
/**
 * @brief Factor initial pose (2.5d)
 */
class IniFactor : public BaseFactor<IniFactor, metaIni_t>
{
  public:
  IniFactor(const std::string& factor_id,
            const std::array<std::string, IniFactor::Meta_t::kNumberOfVars>&
                var_names)
      : BaseFactor<IniFactor, metaIni_t>(factor_id, var_names)
  {
  }
};

//------------------------------------------------------------------//
//              Bearing-only factor type instantiation              //
//------------------------------------------------------------------//
constexpr int                      kNbVar3    = 2;
constexpr int                      kXDimTot3  = 5;
constexpr std::array<int, kNbVar3> kVarSizes3 = {3, 2};
constexpr int                      kMesDim3   = 1;
using metaBearing_t = FactorMetaInfo<kNbVar3, kXDimTot3, kVarSizes3, kMesDim3>;
/**
 * @brief Factor for bearing only
 */
class bearingFactor : public BaseFactor<bearingFactor, metaBearing_t>
{
  public:
  bearingFactor(
      const std::string& factor_id,
      const std::array<std::string, bearingFactor::Meta_t::kNumberOfVars>&
          var_names)
      : BaseFactor<bearingFactor, metaBearing_t>(factor_id, var_names)
  {
  }
};

//------------------------------------------------------------------//
//             Range-bearing factor type instantiation              //
//------------------------------------------------------------------//
constexpr int                      kNbVar4    = 2;
constexpr int                      kXDimTot4  = 5;
constexpr std::array<int, kNbVar4> kVarSizes4 = {3, 2};
constexpr int                      kMesDim4   = 2;
using metaRangeBearing_t
    = FactorMetaInfo<kNbVar4, kXDimTot4, kVarSizes4, kMesDim4>;
/**
 * @brief Factor for range-bearing (pose toward position)
 */
class rangeBearingFactor
    : public BaseFactor<rangeBearingFactor, metaRangeBearing_t>
{
  public:
  rangeBearingFactor(
      const std::string& factor_id,
      const std::array<std::string, rangeBearingFactor::Meta_t::kNumberOfVars>&
          var_names)
      : BaseFactor<rangeBearingFactor, metaRangeBearing_t>(factor_id, var_names)
  {
  }
};

//------------------------------------------------------------------//
//           Orientation-fixed range bearing factor type            //
//                     Position toward position                     //
//           Orientation is given wrt the world main axis           //
//------------------------------------------------------------------//
constexpr int                      kNbVar5    = 2;
constexpr int                      kXDimTot5  = 4;
constexpr std::array<int, kNbVar5> kVarSizes5 = {2, 2};
constexpr int                      kMesDim5   = 2;
using metaRangeBearingSliding_t
    = FactorMetaInfo<kNbVar5, kXDimTot5, kVarSizes5, kMesDim5>;
/**
 * @brief Factor for fixed orientation range-bearing
 */
class rangeBearingSlideFactor
    : public BaseFactor<rangeBearingSlideFactor, metaRangeBearingSliding_t>
{
  public:
  rangeBearingSlideFactor(
      const std::string&                                    factor_id,
      const std::array<std::string, Meta_t::kNumberOfVars>& var_names)
      : BaseFactor<rangeBearingSlideFactor, metaRangeBearingSliding_t>(
          factor_id,
          var_names)
  {
  }
};

// some quick and dirty print helper function
template <typename FACTOR_T>
void printSomeMetaInfo(const FACTOR_T& factor)
{
  std::cout << "\n *****  Printing Meta of factor " << factor.factor_id
            << " of scope (" << StringifyArrayOfStrings(factor.variables)
            << ") ***** \n";

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
  OdomFactor              aFactor("f1", {"x1", "x2"});
  IniFactor               bFactor("f2", {"x0"});
  bearingFactor           cFactor("f3", {"x2", "l1"});
  rangeBearingFactor      dFactor("f4", {"x3", "l2"});
  rangeBearingSlideFactor eFactor("f5", {"x8", "x9"});

  printSomeMetaInfo(aFactor);
  printSomeMetaInfo(bFactor);
  printSomeMetaInfo(cFactor);
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
// for( auto pos: aFactor.variable_position) std::cout << pos.first << ":" <<
// pos.second << " || "; std::cout << "\n";
