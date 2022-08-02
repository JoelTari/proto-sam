#ifndef KEY_META_POSITION_H_
#define KEY_META_POSITION_H_

#include "core/meta.h"

#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <string_view>


namespace details_sam::Meta::Key
{
  namespace Position2dImpl{

    inline static constexpr const char position_str[] = "position 2d";
    inline static constexpr const char x[]        = "x";
    inline static constexpr const char y[]        = "y";
    using Position2d_t                           = Eigen::Vector<double, 2>;
    using namespace ::sam::Meta::Key;

    namespace exports{
      struct Position2d : 
        ::sam::Meta::Key::EuclidBase <Position2d, position_str, Position2d_t,x,y>
      {
        constexpr static std::size_t compute_kN_impl()
        {
          return Position2d_t::RowsAtCompileTime;
        }

        template <const char* COMPONENT>
          static auto get_component_impl(const Position2d_t& key_position_element)
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
            const Position2d_t& key_position_element)
        {
          if (std::string_view(component) == x)
            return key_position_element(0, 0);
          else if (std::string_view(component) == y)
            return key_position_element(1, 0);
          else
            throw std::runtime_error("component requested doesnt exist in key position meta");
        }

      };
    }

  }
}

// expose the key meta position structure
namespace sam::Meta::Key{
  using namespace details_sam::Meta::Key::Position2dImpl::exports;
}
// expose the key type
namespace sam::Key{
  using Position2d_t = typename sam::Meta::Key::Position2d::type;
}

#endif
