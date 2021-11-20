#ifndef SAM_CONFIG_H_
#define SAM_CONFIG_H_

#ifndef ENABLE_DEBUG_TRACE
#define ENABLE_DEBUG_TRACE 1
#endif
#ifndef ENABLE_RUNTIME_CONSISTENCY_CHECKS
#define ENABLE_RUNTIME_CONSISTENCY_CHECKS 1
#endif
#ifndef ENABLE_TIMER
#define ENABLE_TIMER 1
#endif

#if ENABLE_DEBUG_TRACE
#include <iostream>
#endif

#if ENABLE_TIMER > 0
#include<chrono>
    #define PROFILE_SCOPE(name,logger) sam_utils::ScopedTimer timer##__LINE__(name,logger)
    #define PROFILE_FUNCTION(logger)  PROFILE_SCOPE(__FUNCTION__,logger)
#else
    #define PROFILE_SCOPE(name)  // nada
    #define PROFILE_FUNCTION()   // nada
#endif

#endif
