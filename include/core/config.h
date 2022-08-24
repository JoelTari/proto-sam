#ifndef SAM_CONFIG_H_
#define SAM_CONFIG_H_

// compile definitions and their respective default values
#ifndef ENABLE_DEBUG_TRACE
#define ENABLE_DEBUG_TRACE 1
#endif
#ifndef ENABLE_RUNTIME_CONSISTENCY_CHECKS
#define ENABLE_RUNTIME_CONSISTENCY_CHECKS 0
#endif
#ifndef ENABLE_JSON_OUTPUT
#define ENABLE_JSON_OUTPUT 1
#endif
// enable timer -- only makes sense if ENABLE_JSON_OUTPUT is enabled
#ifndef ENABLE_TIMER
static_assert(ENABLE_JSON_OUTPUT,"JSON output must be enabled for timer to work");
#define ENABLE_TIMER 1
#else
#if ENABLE_TIMER
  static_assert(ENABLE_JSON_OUTPUT,"JSON output must be enabled for timer to work");
#endif
#endif

// #if ENABLE_DEBUG_TRACE // no let's include them anyway, no big deal
#include <iostream>
#include <iomanip>
// #endif

#if ENABLE_TIMER > 0
#include<chrono>
    #define PROFILE_SCOPE(name,logger) sam_utils::ScopedTimer timer##__LINE__( (name) ,logger)
    #define PROFILE_FUNCTION(logger)  PROFILE_SCOPE(__FUNCTION__,logger)
#else
    #define PROFILE_SCOPE(name,logger)  // nada
    #define PROFILE_FUNCTION(logger)   // nada
#endif

#endif
