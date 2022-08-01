#ifndef KEY_META_SE2_H_
#define KEY_META_SE2_H_

#include "core/meta.h"
#include <manif/manif.h>

namespace details_sam::Meta::Key
{
  namespace PoseSE2Impl {
    inline static constexpr const char pose_SE2_label[] = "pose SE2";
    inline static constexpr const char x[]   = "x";
    inline static constexpr const char y[]   = "y";
    inline static constexpr const char t[]   = "theta";
    using PoseSE2_t = typename manif::SE2d;

    using namespace ::sam::Meta::Key;

    inline namespace exports{
      struct PoseSE2 : ::sam::Meta::Key::Base<PoseSE2, pose_SE2_label, PoseSE2_t, manif::SE2Tangentd ,x,y,t>
      {
        constexpr static std::size_t compute_kN_impl()
        {
            return manif::SE2Tangentd::DoF;
        }
        
        template<const char* COMPONENT>
        static auto get_component_impl(const PoseSE2_t & key_SE2)
        {
          if constexpr(std::string_view(COMPONENT) ==x)
            return key_SE2.x();
          else
          {
            if constexpr(std::string_view(COMPONENT)==y)
            {
              return key_SE2.y();
            }
            else
            {
              static_assert(std::string_view(COMPONENT)==t);
              return key_SE2.angle();
              // return key_SE2.rotation();
            }
          }
        }

        static double get_component_impl(const char* component, const PoseSE2_t & key_SE2)
        {
          if (std::string_view(component) == x)
            return key_SE2.x();
          else if (std::string_view(component) == y)
            return key_SE2.y();
          else if (std::string_view(component) == t)
            return key_SE2.angle();
          else
            throw std::runtime_error("component requested doesnt exist in key position meta");
        }

      };
    }

  }
}

namespace sam::Meta::Key{
  using namespace details_sam::Meta::Key::PoseSE2Impl::exports;
}

namespace sam::Key{
  using PoseSE2_t = typename sam::Meta::Key::PoseSE2::type;
}

#endif
