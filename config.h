#ifndef SAM_CONFIG_H_
#define SAM_CONFIG_H_

#define ENABLE_DEBUG_TRACE 1
#define ENABLE_RUNTIME_CONSISTENCY_CHECKS 1
#define ENABLE_TIMER 1

#if ENABLE_DEBUG_TRACE
#include <iostream>
#endif

#if ENABLE_TIMER
#include <chrono>
#endif

#endif
