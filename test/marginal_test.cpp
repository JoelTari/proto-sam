#include "factor_impl/key-meta-position.h"
#include "factor_impl/anchor.hpp"
#include "core/marginal.h"
#include <iostream>

int main (int argc, char *argv[])
{
    MarginalsContainer<std::tuple<MetaKeyPosition_t>> container;  
    using MarginalPosition_t = Marginal<MetaKeyPosition_t>;
    
    // UniqueKeyConduct::measure_cov_t rho({{1,0},{0,1}});
    auto adfs = UniqueKeyConduct_t({ "test"}, Eigen::Matrix2d{{1,0},{0,1}} ) ;
    const UniqueKeyConduct_t & adfss = UniqueKeyConduct_t({ "test"}, Eigen::Matrix2d{{1,0},{0,1}} ) ;
    auto & dd = adfs;


    std::cout << std::decay_t<decltype(adfss)>::kN << '\n';
    std::cout << std::decay_t<decltype(dd)>::kN << '\n';

    // Marginal<MetaKeyPosition> ;
    Eigen::Vector2d xmap_marg {1,2};
    Eigen::Matrix2d cov; cov << 1,0,0,3.1;

    auto marg_ptr = std::make_shared<MarginalPosition_t>(  std::make_shared<typename MarginalPosition_t::Mean_t>(xmap_marg),cov);

    container.insert_in_marginal_container<MarginalPosition_t>("k",marg_ptr);

    auto opt = container.find_mean_ptr<MetaKeyPosition_t>("k");
    std::cout << "has value? : " << opt.has_value();
    if (opt.has_value())
      std::cout << "  value (mean) of k is : " << *opt.value() << '\n';

  return 0;
}
