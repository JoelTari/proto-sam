#pragma once

#include "meta/meta.h"

#include <Eigen/Dense>
#include <stdexcept>
#include <string_view>


// forward declarations, completed in cpp
namespace details_sam::Meta::Key::Position2dImpl::exports
{
  struct Position2d;
}

// expose the key meta position structure
namespace sam::Meta::Key{
  // for users
  using namespace details_sam::Meta::Key::Position2dImpl::exports;
  // exported : Position2d, Position2d_t (the key_t)
}
