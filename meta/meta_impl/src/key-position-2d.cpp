#include "key-position-2d/key-position-2d.h"


// using namespace ::sam::Meta::Key;
//
// constexpr std::size_t Position2d::compute_kN_impl()
// {
//   return Position2d_t::RowsAtCompileTime;
// }
//
// // remove
// double Position2d::get_component_impl(const char*          component,
//     const Position2d_t& key_position_element)
// {
//   if (std::string_view(component) == x)
//     return key_position_element(0, 0);
//   else if (std::string_view(component) == y)
//     return key_position_element(1, 0);
//   else
//     throw std::runtime_error("component requested doesnt exist in key position meta");
// }