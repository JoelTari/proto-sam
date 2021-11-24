#include "factor_impl/key-meta-position.h"
#include "core/marginal.h"
#include <iostream>

int main (int argc, char *argv[])
{
  MarginalsContainer<std::tuple<MetaKeyPosition_t>> container;  
  //  static_assert( std::is_same_v
  // <
  // std::unordered_map<std::string,MetaKeyPosition_t>
  // ,typename std::tuple_element_t<0, typename MarginalsContainer<std::tuple<MetaKeyPosition_t>>::marginals_containers_t>>
  // );


    // Marginal<MetaKeyPosition> ;
    Eigen::Vector2d xmap_marg {1,2};
    Eigen::Matrix2d cov; cov << 1,0,0,3.1;

    container.insert<MetaKeyPosition_t>("k",xmap_marg,cov);

    auto opt = container.find<MetaKeyPosition_t>("k");
    std::cout << "has value? : " << opt.has_value();
    if (opt.has_value())
      std::cout << "  value (mean) of k is : " << opt.value().mean << '\n';

  return 0;
}
