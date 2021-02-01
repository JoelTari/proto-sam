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
using OdomFactor_t   = BaseFactor<metaOdom_t>;

/// TESTING A VAR INI TYPE INSTANTIATION
constexpr int                     nbvar2     = 1;
constexpr int                     xdimtot2   = 3;
constexpr std::array<int, nbvar2> var_sizes2 = {3};
/* constexpr std::array< std::pair<int, int>, nbvar2> var_idx_ranges2 =
 * {std::make_pair(0, 2)}; */
constexpr int mesdim2 = 3;
using metaIni_t       = FactorMetaInfo<nbvar2, xdimtot2, var_sizes2, mesdim2>;
using iniFactor_t     = BaseFactor<metaIni_t>;

/// TESTING A BEARING ONLY TYPE INST
constexpr int                     nbvar3     = 2;
constexpr int                     xdimtot3   = 5;
constexpr std::array<int, nbvar3> var_sizes3 = {3, 2};
/* constexpr std::array< std::pair<int, int>, nbvar3> var_idx_ranges3 =
 * {std::make_pair(0, 2), std::make_pair(3, 4)}; */
constexpr int mesdim3 = 1;
using metaBearing_t   = FactorMetaInfo<nbvar3, xdimtot3, var_sizes3, mesdim3>;
using bearingFactor_t = BaseFactor<metaBearing_t>;

/// TESTING A RANGE BEARING TYPE INST
constexpr int mesdim4 = 2;
using metaRangeBearing_t
    = FactorMetaInfo<nbvar3, xdimtot3, var_sizes3, mesdim4>;
using rangeBearingFactor_t = BaseFactor<metaRangeBearing_t>;

// some quick and dirty print helper function
template <typename FACTOR_T>
void print_some_meta_info(const FACTOR_T & factor)
{
  std::cout << "\n *****  Printing Meta of factor f("
            << stringify_array_of_strings(factor.var_names) << ") ***** \n";

  std::cout << "Automated var idx ranges: \n";

  for (auto vrange : FACTOR_T::meta_t::varIdxRanges)
    std::cout << vrange[0] << "," << vrange[1] << "\t";

  std::cout << "\n";
}

int main()
{
  OdomFactor_t         aFactor({"x1", "x2"});
  iniFactor_t          bFactor({"x0"});
  bearingFactor_t      cFactor({"x2", "l1"});
  rangeBearingFactor_t dFactor({"x3", "l2"});

  print_some_meta_info(aFactor);
  print_some_meta_info(bFactor);
  print_some_meta_info(cFactor);
  print_some_meta_info(dFactor);

  return 0;
}
