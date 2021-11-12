#include "sam-system.h"

struct DummyFactor1{
  static const std::string factor_typename;
  std::string factor_id;
};

int main()
{
    // auto test_timer = Timer<std::chrono::duration<float, std::micro> >("main"); // std::micro is same as std::ratio<1,1000>
    // auto main_timer = sam_utils::ScopedTimer<std::chrono::microseconds>(__FUNCTION__); // microseconds is same as duration<int64_t, ratio<1,1000> >
    // PROFILE_FUNCTION();
    auto samsyst1 = SAM::SamSystem<DummyFactor1>();
    return 0;
}
