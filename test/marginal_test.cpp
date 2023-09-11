/* 
 * Copyright 2023 AKKA Technologies and LAAS-CNRS (joel.tari@akka.eu) 
 * 
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by 
 * the European Commission - subsequent versions of the EUPL (the "Licence"); 
 * You may not use this work except in compliance with the Licence. 
 * You may obtain a copy of the Licence at: 
 * 
 * https://joinup.ec.europa.eu/software/page/eupl 
 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the Licence is distributed on an "AS IS" basis, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the Licence for the specific language governing permissions and 
 * limitations under the Licence. 
 */
 
#include "key-spatial-2d/key-spatial-2d.h"
#include "marginal/marginal.h"

#include <iostream>

int main(int argc, char* argv[])
{
  // some compile time tests
  using My_Container_t =
      typename sam::Marginal::MarginalsCollection<std::tuple<sam::Meta::Key::Spatial2d>>::type;
  using My_Container2_t =
      typename sam::Marginal::MarginalsCollection<sam::Meta::Key::Spatial2d>::type;
  // without tuple
  static_assert(std::is_same_v<My_Container2_t,
                               My_Container_t>);   // not the same actually if ::type is forgotten
  My_Container_t container;
  using Marginal_Position_t = sam::Marginal::BaseMarginal<sam::Meta::Key::Spatial2d>;
  using Wrapped_Marginal_Position_t
      = ::sam::Marginal::WrapperPersistentMarginal<Marginal_Position_t>;

  // Marginal<MetaKeyPosition> ;
  Eigen::Vector2d xmap {1, 2};
  Eigen::Matrix2d cov;
  cov << 1, 0, 0, 3.1;

  auto my_wrapped_marginal = sam::Marginal::WrapperPersistentMarginal<Marginal_Position_t>(
      "x1",
      std::make_shared<sam::Key::Spatial2d_t>(xmap),
      std::optional<Eigen::Matrix2d>(cov));

  container.insert_in_marginal_container(my_wrapped_marginal);

  auto opt = container.find_wrapped_marginal<sam::Meta::Key::Spatial2d>("x1");
  std::cout << "has value? : " << opt.has_value();
  // if (opt.has_value())
  //   std::cout << " value (mean) of k is :\n" << *opt.value() << '\n';

  auto my_copied_wmarg = sam::Marginal::WrapperPersistentMarginal<Marginal_Position_t>(
      "x2",
      std::make_shared<sam::Key::Spatial2d_t>(xmap),
      std::optional<Eigen::Matrix2d>(cov));

  // copy assignment
  my_copied_wmarg = my_wrapped_marginal;

  // TODO: marginal SE2

  return 0;
}
