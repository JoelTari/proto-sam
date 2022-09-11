cmake_minimum_required(VERSION 3.16)

# Current Project compile definitions and default
# ENABLE_DEBUG_TRACE 0   (some cout debug messages and results such as small matrices etc...)
# ENABLE_JSON_OUTPUT 1   (results are put in json)
# ENABLE_TIMER 1 (must have ENABLE_JSON_OUTPUT at 1, this is enforced by static assertion)
# ENABLE_RUNTIME_CONSISTENCY_CHECKS 0

project(proto-sam)

if ("${CMAKE_BUILD_TYPE}" MATCHES "Release")
  list(APPEND "COMPILED_WITH" "COMPILED_WITH_AGGRESSIVE_OPTIMISATION")
endif()

# set(CMAKE_CXX_STANDARD 20)

# set(CMAKE_BUILD_TYPE Release) # defer to command line
# add_definitions("-Wfatal-errors")
include(CMakePrintHelpers)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(manif REQUIRED)
find_package(jsoncpp 1.9.4 REQUIRED CONFIG)  # this sets 'jsoncpp_lib' and 'jsoncpp_static'

# attemp to find low level libraries to improve performance
find_package(OpenMP)
# set(BLA_STATIC on)
set(BLA_VENDOR Intel10_64lp)
# set(BLA_VENDOR Generic)
find_package(BLAS)
find_package(LAPACK)
find_package(TBB) # link error with intel if BLAS and LAPACK called but not tbb (on execution policy)
# BUG: link error with lapacke_dgetrf with generic blas vendor

# setting some variables depending on whats avaible on the system
if (BLAS_FOUND)
  list(APPEND "COMPILED_WITH" "COMPILED_WITH_BLAS")
  if ("${BLAS_LIBRARIES}" MATCHES ".*intel.*") # if BLAS vendor is intel's mkl
    list(APPEND "COMPILED_WITH" "COMPILED_WITH_BLA_VENDOR_MKL")
    set(LOW_LEVEL_DEFINITIONS "EIGEN_USE_BLAS;EIGEN_USE_LAPACKE;EIGEN_USE_MKL_VML")# equivalent to EIGEN_USE_MKL_ALL
    set(LOW_LEVEL_LIBRARIES "BLAS::BLAS;LAPACK::LAPACK;TBB::tbb") # no openMP with intel, see if another BLAS lib
  else()
    set(LOW_LEVEL_DEFINITIONS "EIGEN_USE_BLAS;EIGEN_USE_LAPACKE")
    if (OpenMP_CXX_FOUND)
      list(APPEND "COMPILED_WITH" "COMPILED_WITH_OPEN_MP")
      set(LOW_LEVEL_LIBRARIES "BLAS::BLAS;LAPACK::LAPACK;OpenMP::OpenMP_CXX")
    else()
      set(LOW_LEVEL_LIBRARIES "BLAS::BLAS;LAPACK::LAPACK")
    endif()
  endif()
else()
  set(LOW_LEVEL_DEFINITIONS "")
  if (OpenMP_CXX_FOUND)
    list(APPEND "COMPILED_WITH" "COMPILED_WITH_OPEN_MP")
    set(LOW_LEVEL_LIBRARIES "OpenMP::OpenMP_CXX")
  else()
    set(LOW_LEVEL_LIBRARIES "")
  endif()
endif()

# letting the user know the compilation profile wrt low level libraries
# ExIGraM : Exact Inference on Graphical Models
message(STATUS "\n[  ] Low level definitions :\n\t ${LOW_LEVEL_DEFINITIONS}")
message(STATUS "\n[  ] Low level libraries :\n\t ${LOW_LEVEL_LIBRARIES}")
message(STATUS "\n[  ] Compiled with the following definitions:\n\t ${COMPILED_WITH}")

add_subdirectory(meta)
list(APPEND META_STATIC_LIBRARIES key_spatial_2d key_spatial_SE2)  # add measures
add_subdirectory(factor)
# bundle of all factors 
list(APPEND FACTOR_STATIC_LIBRARIES
  factor_anchor2d factor_anchorSE2 factor_motion_model_SE2 factor_relative_matcher_2d factor_relative_matcher_SE2 factor_motion_model_SE2 factor_cartesian_landmark_obs_SE2
  )
add_subdirectory(inference)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)


add_executable(mainstdin main/mainstdin.cpp)
target_link_libraries(mainstdin 
  PUBLIC 
  jsoncpp_static 
  factor_anchor2d 
  factor_relative_matcher_2d 
  sam_system_interface
  ${LOW_LEVEL_LIBRARIES})
target_compile_features(mainstdin PUBLIC cxx_std_20)
target_compile_definitions(mainstdin PUBLIC ${COMPILED_WITH})


#--------------------------------------------------------------------#
#                                test                                #
#--------------------------------------------------------------------#
# test: move in test subdirectory with dedicated cmake
add_executable(marginal_test test/marginal_test.cpp)
target_link_libraries(marginal_test PUBLIC sam_system_interface ${META_STATIC_LIBRARIES})
target_include_directories(marginal_test PUBLIC ${EIGEN_INCLUDE_DIRS})
target_compile_features(marginal_test PUBLIC cxx_std_20)

include(FetchContent)

find_package(GTest REQUIRED)

if (NOT GTest_FOUND)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
  )
  FetchContent_MakeAvailable(googletest)
endif()

enable_testing()

message(STATUS "\n[ExIGraM] TEST LIBRARIES :\n\t ${GTEST_BOTH_LIBRARIES}")

add_executable(aggregate_tests 
  test/factor_test.cpp 
  test/toy_pose_graph_SE2_test.cpp 
  test/toy_slam_SE2.cpp 
  test/toy_linear_test.cpp 
  test/eigen_blas_routine_test.cpp 
  test/system_fail_test.cpp)
target_link_libraries(aggregate_tests sam_system_interface ${FACTOR_STATIC_LIBRARIES} ${LOW_LEVEL_LIBRARIES} ${GTEST_BOTH_LIBRARIES})
target_include_directories(aggregate_tests PUBLIC ${GTEST_INCLUDE_DIRS})
target_compile_definitions(aggregate_tests PUBLIC ENABLE_DEBUG_TRACE=0 ENABLE_TIMER=0 ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(aggregate_tests PUBLIC cxx_std_20)

include(GoogleTest)
gtest_discover_tests(aggregate_tests)
