#pragma once

// compile definitions and their respective default values
#ifndef ENABLE_DEBUG_TRACE
#define ENABLE_DEBUG_TRACE 0
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
#define ENABLE_TIMER ENABLE_JSON_OUTPUT // enabled if JSON OUPUT if itself enabled, disabled othws
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

//------------------------------------------------------------------//
//                         Compile options                          //
//------------------------------------------------------------------//
// # BLAS 
// # OpenMP
// # BLA_VENDOR_MKL / BLA_VENDOR_GENERIC
// # AGGRESSIVE_OPTIMISATION
// # CUDA (later)
#ifndef COMPILED_WITH_BLAS
#define COMPILED_WITH_BLAS 0
#endif
#ifndef COMPILED_WITH_OPEN_MP
#define COMPILED_WITH_OPEN_MP 0
#endif
#ifndef COMPILED_WITH_BLAS
#define COMPILED_WITH_BLAS 0
#endif
#ifndef COMPILED_WITH_BLA_VENDOR_MKL
#define COMPILED_WITH_BLA_VENDOR_MKL 0
#endif
// #ifndef BLA_VENDOR_GENERIC
// #define BLA_VENDOR_GENERIC 0
// #endif
#ifndef COMPILED_WITH_AGGRESSIVE_OPTIMISATION
#define COMPILED_WITH_AGGRESSIVE_OPTIMISATION 0
#endif
// later: CUDA

static_assert( (COMPILED_WITH_BLA_VENDOR_MKL && COMPILED_WITH_BLAS ) || (!COMPILED_WITH_BLAS && !COMPILED_WITH_BLA_VENDOR_MKL), 
    "Compile definitions are wrong : can't be compiled with MKL blas if not compiled with blas"
    );

namespace sam::definitions
{
struct CompiledDefinitions
{
  static constexpr bool blas = COMPILED_WITH_BLAS;
  static constexpr bool openmp = COMPILED_WITH_OPEN_MP;
  static constexpr bool bla_vendor_mkl = COMPILED_WITH_BLA_VENDOR_MKL;
  static constexpr bool optimised = COMPILED_WITH_AGGRESSIVE_OPTIMISATION;
  static constexpr bool timer = ENABLE_TIMER;
  static constexpr bool json_output = ENABLE_JSON_OUTPUT;
  static constexpr bool runtime_checks = ENABLE_RUNTIME_CONSISTENCY_CHECKS;
  static constexpr bool debug_trace= ENABLE_DEBUG_TRACE;
  // later : CUDA
};
}
// since these doesn't implies any branching in this library (only at the lower level for ext dependencies)
// Lets put that in a structure
