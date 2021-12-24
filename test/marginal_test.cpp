#include "factor_impl/key-meta-position.h"
#include "factor_impl/anchor.hpp"
#include "core/marginal.h"
#include <iostream>

int main (int argc, char *argv[])
{
    MarginalsContainer<std::tuple<MetaKeyPosition_t>> container;  
    using MarginalPosition_t = Marginal<MetaKeyPosition_t>;
    
    // UniqueKeyConduct::measure_cov_t rho({{1,0},{0,1}});
    auto adfs = UniqueKeyConduct({ "test"}, Eigen::Matrix2d{{1,0},{0,1}} ) ;
    const UniqueKeyConduct & adfss = UniqueKeyConduct({ "test"}, Eigen::Matrix2d{{1,0},{0,1}} ) ;
    auto & dd = adfs;


    std::cout << std::decay_t<decltype(adfss)>::kN << '\n';
    std::cout << std::decay_t<decltype(dd)>::kN << '\n';

    // Marginal<MetaKeyPosition> ;
    Eigen::Vector2d xmap_marg {1,2};
    Eigen::Matrix2d cov; cov << 1,0,0,3.1;

    MarginalPosition_t marginal(xmap_marg,cov);

    container.insertt<MarginalPosition_t>("k",marginal);

    auto opt = container.findt<MetaKeyPosition_t>("k");
    std::cout << "has value? : " << opt.has_value();
    if (opt.has_value())
      std::cout << "  value (mean) of k is : " << opt.value().mean << '\n';

  return 0;
}
