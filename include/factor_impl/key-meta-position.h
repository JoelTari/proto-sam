#ifndef KEY_META_POSITION_H_
#define KEY_META_POSITION_H_

#include "core/meta.h"

#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <string_view>

namespace __MetaKeyPosition
{
  inline static constexpr const char position[] = "position";
  inline static constexpr const char x[]        = "x";
  inline static constexpr const char y[]        = "y";
  using KeyPosition_t                           = Eigen::Vector<double, 2>;
  // using MetaKeyPosition_t                = KeyMeta<position, 2, x, y>;
  struct MetaKeyPosition_t : EuclidKeyMeta<MetaKeyPosition_t, position, KeyPosition_t, x, y>
  {
    template <const char* COMPONENT>
    static auto get_component_impl(const KeyPosition_t& key_position_element)
    {
      if constexpr (std::string_view(COMPONENT) == x)
        return key_position_element(0, 0);
      else
      {
        static_assert(std::string_view(COMPONENT) == y);
        return key_position_element(1, 0);
      }
    }

    // remove
    static double get_component_impl(const char*          component,
                                     const KeyPosition_t& key_position_element)
    {
      if (std::string_view(component) == x)
        return key_position_element(0, 0);
      else if (std::string_view(component) == y)
        return key_position_element(1, 0);
      else
        throw std::runtime_error("component requested doesnt exist in key position meta");
    }

  };
}   // namespace __MetaKeyPosition
using MetaKeyPosition_t = __MetaKeyPosition::MetaKeyPosition_t;
#endif
