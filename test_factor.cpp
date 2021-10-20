#include "factor.h"
#include <array>
#include <iostream>

//------------------------------------------------------------------//
//                  Odom factor type instantiation                  //
//------------------------------------------------------------------//
constexpr int                    nbvar     = 2;
constexpr int                    xdimtot   = 6;
constexpr std::array<int, nbvar> var_sizes = {3, 3};
constexpr int                    mesdim    = 3;
using metaOdom_t = FactorMetaInfo<nbvar, xdimtot, var_sizes, mesdim>;

/**
 * @brief Factor odometry
 */
class OdomFactor : public BaseFactor<OdomFactor, metaOdom_t>
{
public:
  OdomFactor(const std::string & factor_id,
             const std::array<std::string, OdomFactor::meta_t::numberOfVars> &
                 var_names)
      : BaseFactor<OdomFactor, metaOdom_t>(factor_id, var_names)
  {
  }
};

//------------------------------------------------------------------//
//                Initial pose factor instantiation                 //
//------------------------------------------------------------------//
constexpr int                     nbvar2     = 1;
constexpr int                     xdimtot2   = 3;
constexpr std::array<int, nbvar2> var_sizes2 = {3};
constexpr int                     mesdim2    = 3;
using metaIni_t = FactorMetaInfo<nbvar2, xdimtot2, var_sizes2, mesdim2>;
/**
 * @brief Factor initial pose (2.5d)
 */
class IniFactor : public BaseFactor<IniFactor, metaIni_t>
{
public:
  IniFactor(const std::string & factor_id,
            const std::array<std::string, IniFactor::meta_t::numberOfVars> &
                var_names)
      : BaseFactor<IniFactor, metaIni_t>(factor_id, var_names)
  {
  }
};

//------------------------------------------------------------------//
//              Bearing-only factor type instantiation              //
//------------------------------------------------------------------//
constexpr int                     nbvar3     = 2;
constexpr int                     xdimtot3   = 5;
constexpr std::array<int, nbvar3> var_sizes3 = {3, 2};
constexpr int                     mesdim3    = 1;
using metaBearing_t = FactorMetaInfo<nbvar3, xdimtot3, var_sizes3, mesdim3>;
/**
 * @brief Factor for bearing only
 */
class bearingFactor : public BaseFactor<bearingFactor, metaBearing_t>
{
public:
  bearingFactor(
      const std::string & factor_id,
      const std::array<std::string, bearingFactor::meta_t::numberOfVars> &
          var_names)
      : BaseFactor<bearingFactor, metaBearing_t>(factor_id, var_names)
  {
  }
};

//------------------------------------------------------------------//
//             Range-bearing factor type instantiation              //
//------------------------------------------------------------------//
constexpr int                     nbvar4     = 2;
constexpr int                     xdimtot4   = 5;
constexpr std::array<int, nbvar4> var_sizes4 = {3, 2};
constexpr int                     mesdim4    = 2;
using metaRangeBearing_t
    = FactorMetaInfo<nbvar4, xdimtot4, var_sizes4, mesdim4>;
/**
 * @brief Factor for range-bearing (pose toward position)
 */
class rangeBearingFactor
    : public BaseFactor<rangeBearingFactor, metaRangeBearing_t>
{
public:
  rangeBearingFactor(
      const std::string & factor_id,
      const std::array<std::string, rangeBearingFactor::meta_t::numberOfVars> &
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
constexpr int                     nbvar5     = 2;
constexpr int                     xdimtot5   = 4;
constexpr std::array<int, nbvar5> var_sizes5 = {2, 2};
constexpr int                     mesdim5    = 2;
using metaRangeBearingSliding_t
    = FactorMetaInfo<nbvar5, xdimtot5, var_sizes5, mesdim5>;
/**
 * @brief Factor for fixed orientation range-bearing
 */
class rangeBearingSlideFactor
    : public BaseFactor<rangeBearingSlideFactor, metaRangeBearingSliding_t>
{
public:
  rangeBearingSlideFactor(
      const std::string &                                   factor_id,
      const std::array<std::string, meta_t::numberOfVars> & var_names)
      : BaseFactor<rangeBearingSlideFactor, metaRangeBearingSliding_t>(
          factor_id,
          var_names)
  {
  }
};

// some quick and dirty print helper function
template <typename FACTOR_T>
void print_some_meta_info(const FACTOR_T & factor)
{
  std::cout << "\n *****  Printing Meta of factor " << factor.factor_id
            << " of scope (" << stringify_array_of_strings(factor.variables)
            << ") ***** \n";

  std::cout << "Automated var idx ranges (in the factor state vector): \n";

  for (auto vrange : FACTOR_T::meta_t::varIdxRanges)
    std::cout << vrange[0] << "," << vrange[1] << "\t";
  std::cout << "\n";

  for (const auto & var : factor.variables) {
    std::cout
        << var << " -> ["
        << FACTOR_T::meta_t::varIdxRanges[factor.variable_position.at(var)][0]
        << ":"
        << FACTOR_T::meta_t::varIdxRanges[factor.variable_position.at(var)][1]
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

  print_some_meta_info(aFactor);
  print_some_meta_info(bFactor);
  print_some_meta_info(cFactor);
  print_some_meta_info(dFactor);
  print_some_meta_info(eFactor);

  // query test
  try {
    std::cout << "Variable ordering of x1 is : "
              << aFactor.variable_position.at("x1") << '\n';
  } catch (std::out_of_range e) {
    throw e;
  }

  return 0;
}
// for( auto pos: aFactor.variable_position) std::cout << pos.first << ":" <<
// pos.second << " || "; std::cout << "\n";
