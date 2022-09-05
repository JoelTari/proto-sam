#include "measure-position-2d/measure-position-2d.h"

namespace details_sam::Meta::Measure::Position2dImpl::exports
{
    double Position2d::get_component_impl(const char*                       component,
                                     const Position2d_t & measure)
    {
      if (std::string_view(component) == x)
        return measure(0, 0);
      else if (std::string_view(component) == y)
        return measure(1, 0);
      else
        throw std::runtime_error("component requested doesnt exist in measure  position meta");
    }
}
