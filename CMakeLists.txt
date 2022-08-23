cmake_minimum_required(VERSION 3.16)

# add those target compile definitionns: 
# BLAS 
# OMP (without the rest of BLAS)
# AGGRESSIVELY_OPTIMIZED

project(proto-sam)

# set(CMAKE_BUILD_TYPE Release) # defer to command line
# add_definitions("-Wfatal-errors")
include(CMakePrintHelpers)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(manif REQUIRED)
find_package(jsoncpp 1.9.4 REQUIRED CONFIG)  # this sets 'jsoncpp_lib'
find_package(OpenMP)

set(BLA_STATIC on)
set(BLA_VENDOR Intel10_64lp) # TODO: vendor should be flexible
find_package(BLAS)
find_package(LAPACK)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

add_executable(marginal_test test/marginal_test.cpp)
add_executable(mainstdin main/mainstdin.cpp)

target_include_directories(marginal_test PUBLIC include)
target_include_directories(mainstdin PUBLIC include)

# NOTE: jsoncpp_static vs jsoncpp_lib (.so)  => 200 kB difference on binary size
if(BLAS_FOUND)
  # check that iomp5 goes with it
  message(STATUS "Linking targets to BLAS libraries...")
  target_compile_definitions(mainstdin PUBLIC EIGEN_USE_MKL_ALL)
  target_link_libraries(mainstdin PUBLIC jsoncpp_lib BLAS::BLAS)
elseif(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found !")
    target_link_libraries(mainstdin PUBLIC OpenMP::OpenMP_CXX jsoncpp_lib)
else()
    message(WARNING "No blas found, no openMP found, might not be ")
    target_link_libraries(mainstdin PUBLIC jsoncpp_lib)
endif()

target_compile_features(marginal_test PUBLIC cxx_std_20)
target_compile_features(mainstdin PUBLIC cxx_std_20)

# NOTE: to manage some definitions
# target_compile_definitions(mainstdin PUBLIC ENABLE_DEBUG_TRACE=1 ENABLE_TIMER=1)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)

enable_testing()

add_executable(aggregate_tests test/factor_test.cpp test/toy_pose_graph_SE2_test.cpp test/toy_slam_SE2.cpp test/toy_linear_test.cpp test/eigen_blas_routine_test.cpp)
target_include_directories(aggregate_tests PUBLIC include ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(aggregate_tests jsoncpp_lib OpenMP::OpenMP_CXX BLAS::BLAS gtest gmock gtest_main)
target_compile_definitions(aggregate_tests PUBLIC EIGEN_USE_MKL_ALL ENABLE_DEBUG_TRACE=0 ENABLE_TIMER=0)
target_compile_features(aggregate_tests PUBLIC cxx_std_20)

include(GoogleTest)
gtest_discover_tests(aggregate_tests)
