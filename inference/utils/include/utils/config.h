/* 
 * Copyright 2023 AKKA Technologies and LAAS-CNRS (joel.tari@akka.eu) 
 * 
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by 
 * the European Commission - subsequent versions of the EUPL (the "Licence"); 
 * You may not use this work except in compliance with the Licence. 
 * You may obtain a copy of the Licence at: 
 * 
 * https://joinup.ec.europa.eu/software/page/eupl 
 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the Licence is distributed on an "AS IS" basis, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the Licence for the specific language governing permissions and 
 * limitations under the Licence. 
 */
 
#pragma once

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
static_assert(ENABLE_JSON_OUTPUT, "JSON output must be enabled for timer to work");
#define ENABLE_TIMER ENABLE_JSON_OUTPUT   // enabled if JSON OUPUT if itself enabled, disabled othws
#else
#if ENABLE_TIMER
static_assert(ENABLE_JSON_OUTPUT, "JSON output must be enabled for timer to work");
#endif
#endif

#define ENABLE_DEBUG_TRACE_TMP 1 // convenience

// #if ENABLE_DEBUG_TRACE // no let's include them anyway, no big deal
#include <iomanip>
#include <iostream>
// #endif

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

static_assert(
    (COMPILED_WITH_BLA_VENDOR_MKL && COMPILED_WITH_BLAS)
        || (!COMPILED_WITH_BLAS && !COMPILED_WITH_BLA_VENDOR_MKL),
    "Compile definitions are wrong : can't be compiled with MKL blas if not compiled with blas");

namespace sam::definitions
{
  struct CompiledDefinitions
  {
    static constexpr bool blas           = COMPILED_WITH_BLAS;
    static constexpr bool openmp         = COMPILED_WITH_OPEN_MP;
    static constexpr bool bla_vendor_mkl = COMPILED_WITH_BLA_VENDOR_MKL;
    static constexpr bool optimised      = COMPILED_WITH_AGGRESSIVE_OPTIMISATION;
    static constexpr bool timer          = ENABLE_TIMER;
    static constexpr bool json_output    = ENABLE_JSON_OUTPUT;
    static constexpr bool runtime_checks = ENABLE_RUNTIME_CONSISTENCY_CHECKS;
    static constexpr bool debug_trace    = ENABLE_DEBUG_TRACE;
    // later : CUDA
  };
}   // namespace sam::definitions
// since these doesn't implies any branching in this library (only at the lower level for ext
// dependencies) Lets put that in a structure
