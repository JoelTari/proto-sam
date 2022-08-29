#pragma once

#include "core/config.h"

// a wrapper for factor, with storing informations about a lin point
template <typename FACTOR_T>
class WrapperPersistentFactor
{
  // using composite
  // using matrices_Aik

  public:
    

    void new_point(const composite_state_ptr_t & composite_state_point, bool push_current_in_history)
    {
      // Aj, bj
      // factor norm
      // push former point in history (if arg true)
      // r(X) ?
    }

    //const getters of bj, Aj, norm, state etc...

    std::vector<composite_state_ptr_t> history;
  private:


};


// specialize for euclidian, linear ?
