cmake_minimum_required(VERSION 3.16)

# add those target compile definitionns: 
# BLAS 
# OMP (without the rest of BLAS)
# AGGRESSIVELY_OPTIMIZED

# Current Project compile definitions and default
# ENABLE_DEBUG_TRACE 0   (some cout debug messages and results such as small matrices etc...)
# ENABLE_JSON_OUTPUT 1   (results are put in json)
# ENABLE_TIMER 1 (must have ENABLE_JSON_OUTPUT at 1, this is enforced by static assertion)
# ENABLE_RUNTIME_CONSISTENCY_CHECKS 0

project(proto-sam)

# set(CMAKE_BUILD_TYPE Release) # defer to command line
# add_definitions("-Wfatal-errors")
include(CMakePrintHelpers)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(manif REQUIRED)
find_package(jsoncpp 1.9.4 REQUIRED CONFIG)  # this sets 'jsoncpp_lib' and 'jsoncpp_static'

# attemp to find low level libraries to improve performance
find_package(OpenMP)
set(BLA_STATIC on)
set(BLA_VENDOR Intel10_64lp) # TODO: vendor should be flexible
find_package(BLAS)
find_package(LAPACK)

# setting some variables depending on whats avaible on the system
if (BLAS_FOUND)
  set(LOW_LEVEL_DEFINITIONS "EIGEN_USE_MKL_ALL") # EIGEN_USE_BLAS and EIGEN_USE_LAPACKE and EIGEN_USE_MKL_VML
  # define only EIGEN_USE_BLAS and EIGEN_USE_LAPACKE if blas vendor is not intel
  if (OpenMP_CXX_FOUND)
    set(LOW_LEVEL_LIBRARIES "BLAS::BLAS OpenMP::OpenMP_CXX")
  else()
    set(LOW_LEVEL_LIBRARIES "BLAS::BLAS")
  endif()
else()
  set(LOW_LEVEL_DEFINITIONS "")
  if (OpenMP_CXX_FOUND)
    set(LOW_LEVEL_LIBRARIES "OpenMP::OpenMP_CXX")
  else()
    set(LOW_LEVEL_LIBRARIES "")
  endif()
endif()

# letting the user know the compilation profile wrt low level libraries
message(STATUS "Low level definitions: ${LOW_LEVEL_DEFINITIONS}")
message(STATUS "Low level libraries: ${LOW_LEVEL_LIBRARIES}")

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

add_executable(marginal_test test/marginal_test.cpp)
add_executable(mainstdin main/mainstdin.cpp)

target_include_directories(marginal_test PUBLIC include)
target_include_directories(mainstdin PUBLIC include)

target_compile_definitions(mainstdin PUBLIC ${LOW_LEVEL_DEFINITIONS})
target_link_libraries(mainstdin PUBLIC jsoncpp_lib ${LOW_LEVEL_LIBRARIES})

# NOTE: jsoncpp_static vs jsoncpp_lib (.so)  => 200 kB difference on binary size

target_compile_features(marginal_test PUBLIC cxx_std_20)
target_compile_features(mainstdin PUBLIC cxx_std_20)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(aggregate_tests test/factor_test.cpp test/toy_pose_graph_SE2_test.cpp test/toy_slam_SE2.cpp test/toy_linear_test.cpp test/eigen_blas_routine_test.cpp)
target_include_directories(aggregate_tests PUBLIC include ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
# target_link_libraries(aggregate_tests jsoncpp_lib OpenMP::OpenMP_CXX BLAS::BLAS gtest gmock gtest_main)
target_link_libraries(aggregate_tests jsoncpp_lib OpenMP::OpenMP_CXX gtest gmock gtest_main)
target_compile_definitions(aggregate_tests PUBLIC ENABLE_DEBUG_TRACE=0 ENABLE_TIMER=0)
# target_compile_definitions(aggregate_tests PUBLIC EIGEN_USE_MKL_ALL ENABLE_DEBUG_TRACE=0 ENABLE_TIMER=0)
target_compile_features(aggregate_tests PUBLIC cxx_std_20)

include(GoogleTest)
gtest_discover_tests(aggregate_tests)
