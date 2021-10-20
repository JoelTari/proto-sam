#include "factor.h"
#include <array>
#include <iostream>

/// TESTING AN ODOM FACTOR TYPE INSTANTIATION
constexpr int                    nbvar     = 2;
constexpr int                    xdimtot   = 6;
constexpr std::array<int, nbvar> var_sizes = {3, 3};
/* constexpr std::array< std::pair<int, int>, nbvar> var_idx_ranges =
 * {std::make_pair(0, 2), std::make_pair(3, 5)}; */
constexpr int mesdim = 3;
using metaOdom_t     = FactorMetaInfo<nbvar, xdimtot, var_sizes, mesdim>;

class OdomFactor : public BaseFactor<OdomFactor, metaOdom_t>
{
public:
  OdomFactor(const std::array<std::string, OdomFactor::meta_t::numberOfVars> &
                 var_names)
      : BaseFactor<OdomFactor, metaOdom_t>(var_names)
  {
  }
};

/* using OdomFactor_t   = BaseFactor<metaOdom_t>; */

/// TESTING A VAR INI TYPE INSTANTIATION
constexpr int                     nbvar2     = 1;
constexpr int                     xdimtot2   = 3;
constexpr std::array<int, nbvar2> var_sizes2 = {3};
/* constexpr std::array< std::pair<int, int>, nbvar2> var_idx_ranges2 =
 * {std::make_pair(0, 2)}; */
constexpr int mesdim2 = 3;
using metaIni_t       = FactorMetaInfo<nbvar2, xdimtot2, var_sizes2, mesdim2>;
class IniFactor : public BaseFactor<IniFactor, metaIni_t>
{
public:
  IniFactor(const std::array<std::string, IniFactor::meta_t::numberOfVars> &
                var_names)
      : BaseFactor<IniFactor, metaIni_t>(var_names)
  {
  }
};

/// TESTING A BEARING ONLY TYPE INST
constexpr int                     nbvar3     = 2;
constexpr int                     xdimtot3   = 5;
constexpr std::array<int, nbvar3> var_sizes3 = {3, 2};
/* constexpr std::array< std::pair<int, int>, nbvar3> var_idx_ranges3 =
 * {std::make_pair(0, 2), std::make_pair(3, 4)}; */
constexpr int mesdim3 = 1;
using metaBearing_t   = FactorMetaInfo<nbvar3, xdimtot3, var_sizes3, mesdim3>;
class bearingFactor : public BaseFactor<bearingFactor, metaBearing_t>
{
public:
  bearingFactor(
      const std::array<std::string, bearingFactor::meta_t::numberOfVars> &
          var_names)
      : BaseFactor<bearingFactor, metaBearing_t>(var_names)
  {
  }
};

/// TESTING A RANGE BEARING TYPE INST
constexpr int mesdim4 = 2;
using metaRangeBearing_t
    = FactorMetaInfo<nbvar3, xdimtot3, var_sizes3, mesdim4>;
class rangeBearingFactor
    : public BaseFactor<rangeBearingFactor, metaRangeBearing_t>
{
public:
  rangeBearingFactor(
      const std::array<std::string, rangeBearingFactor::meta_t::numberOfVars> &
          var_names)
      : BaseFactor<rangeBearingFactor, metaRangeBearing_t>(var_names)
  {
  }
};


/// TESTING A SLIDING RANGE BEARING TYPE INST
// A Sliding range Bearing is a range-bearing measure, except all states are positions (not pose)
// The angle is assumed to be measured from the world x-axis (not the robot orientation, which doesn't exist)
constexpr int                    nbvar5     = 2;
constexpr int                    xdimtot5   = 4;
constexpr std::array<int, nbvar5> var_sizes5 = {2, 2};
constexpr int mesdim5 = 2;
using metaRangeBearingSliding_t
    = FactorMetaInfo<nbvar5, xdimtot5, var_sizes5, mesdim5>;
class rangeBearingSlideFactor
  : public BaseFactor<rangeBearingSlideFactor, metaRangeBearingSliding_t>
{
  public:
  rangeBearingSlideFactor(const std::array<std::string, meta_t::numberOfVars> & var_names)
    : BaseFactor<rangeBearingSlideFactor, metaRangeBearingSliding_t>(var_names)
{}
};

// some quick and dirty print helper function
template <typename FACTOR_T>
void print_some_meta_info(const FACTOR_T & factor)
{
  std::cout << "\n *****  Printing Meta of factor f("
            << stringify_array_of_strings(factor.variables) << ") ***** \n";

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
  OdomFactor         aFactor({"x1", "x2"});
  IniFactor          bFactor({"x0"});
  bearingFactor      cFactor({"x2", "l1"});
  rangeBearingFactor dFactor({"x3", "l2"});
  rangeBearingSlideFactor eFactor({"x8","x9"});

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
    // for( auto pos: aFactor.variable_position) std::cout << pos.first << ":" << pos.second << " || ";
    // std::cout << "\n";
