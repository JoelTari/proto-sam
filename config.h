#ifndef SAM_CONFIG_H_
#define SAM_CONFIG_H_

#define ENABLE_DEBUG_TRACE 1
#define ENABLE_RUNTIME_CONSISTENCY_CHECKS 1
#define ENABLE_TIMER 1

#if ENABLE_DEBUG_TRACE
#include <iostream>
#endif

#if ENABLE_TIMER > 0
#include<chrono>
    #define PROFILE_SCOPE(name) sam_utils::ScopedTimer timer##__LINE__(name)
    #define PROFILE_FUNCTION()  PROFILE_SCOPE(__FUNCTION__)
#else
    #define PROFILE_SCOPE(name)  // nada
    #define PROFILE_FUNCTION()   // nada
#endif

#endif
