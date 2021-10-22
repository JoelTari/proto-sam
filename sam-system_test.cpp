#include "sam-system.h"
#include <string>
#include <iostream>

struct DummyFactor1{
  static const std::string factor_typename;
  std::string factor_id;
};

int main()
{
    auto samsyst1 = SAM::SamSystem<DummyFactor1>();
    return 0;
}
