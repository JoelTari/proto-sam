cmake_minimum_required(VERSION 3.16)

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

# add_executable(factor_test test/factor_test.cpp)
add_executable(marginal_test test/marginal_test.cpp)
add_executable(sam-system-factor_test test/sam-system-factor_test.cpp)
add_executable(toy_linear_test test/toy_linear_test.cpp)
add_executable(toy_SE2_test test/toy_SE2_test.cpp)
add_executable(toy_pose_graph_SE2_test test/toy_pose_graph_SE2_test.cpp)
add_executable(toy_slam_SE2 test/toy_slam_SE2.cpp)
add_executable(mainstdin main/mainstdin.cpp)

target_include_directories(marginal_test PUBLIC include)
target_include_directories(sam-system-factor_test PUBLIC include)
target_include_directories(toy_linear_test PUBLIC include)
target_include_directories(toy_SE2_test PUBLIC include ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_include_directories(toy_pose_graph_SE2_test PUBLIC include ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_include_directories(toy_slam_SE2 PUBLIC include ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_include_directories(mainstdin PUBLIC include)

# TODO: try jsoncpp_static and compare size of binaries
if(BLAS_FOUND)
  # check that iomp5 goes with it
  message(STATUS "Linking targets to BLAS libraries...")
  # add some test executable
  add_executable(eigen_blas_routine_test test/eigen_blas_routine_test.cpp)
  target_compile_definitions(eigen_blas_routine_test PUBLIC EIGEN_USE_MKL_ALL)
  target_link_libraries(eigen_blas_routine_test BLAS::BLAS)
  target_include_directories(eigen_blas_routine_test SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
  target_compile_features(eigen_blas_routine_test PUBLIC cxx_std_20)
  # definitions
  target_compile_definitions(sam-system-factor_test PUBLIC EIGEN_USE_MKL_ALL)
  # target_compile_definitions(factor_test PUBLIC EIGEN_USE_MKL_ALL)
  target_compile_definitions(toy_linear_test PUBLIC EIGEN_USE_MKL_ALL)
  target_compile_definitions(toy_pose_graph_SE2_test PUBLIC EIGEN_USE_MKL_ALL)
  target_compile_definitions(toy_slam_SE2 PUBLIC EIGEN_USE_MKL_ALL)
  target_compile_definitions(mainstdin PUBLIC EIGEN_USE_MKL_ALL)
  # links
  target_link_libraries(sam-system-factor_test jsoncpp_lib BLAS::BLAS)
  # target_link_libraries(factor_test jsoncpp_lib BLAS::BLAS)
  target_link_libraries(toy_linear_test jsoncpp_lib BLAS::BLAS)
  target_link_libraries(toy_pose_graph_SE2_test jsoncpp_lib BLAS::BLAS)
  target_link_libraries(toy_slam_SE2 jsoncpp_lib BLAS::BLAS)
  target_link_libraries(mainstdin PUBLIC jsoncpp_lib BLAS::BLAS)
elseif(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found !")
    target_link_libraries(sam-system-factor_test jsoncpp_lib OpenMP::OpenMP_CXX)
    # target_link_libraries(factor_test jsoncpp_lib OpenMP::OpenMP_CXX)
    target_link_libraries(toy_linear_test jsoncpp_lib OpenMP::OpenMP_CXX)
    target_link_libraries(toy_pose_graph_SE2_test jsoncpp_lib OpenMP::OpenMP_CXX)
    target_link_libraries(toy_slam_SE2 jsoncpp_lib OpenMP::OpenMP_CXX)
    target_link_libraries(mainstdin PUBLIC jsoncpp_lib OpenMP::OpenMP_CXX)
    target_link_libraries(mainstdin PUBLIC OpenMP::OpenMP_CXX jsoncpp_lib)
else()
    message(WARNING "No blas found, no openMP found, might not be ")
    target_link_libraries(sam-system-factor_test jsoncpp_lib )
    # target_link_libraries(factor_test jsoncpp_lib )
    target_link_libraries(toy_linear_test jsoncpp_lib )
    target_link_libraries(toy_pose_graph_SE2_test jsoncpp_lib )
    target_link_libraries(toy_slam_SE2 jsoncpp_lib )
    target_link_libraries(mainstdin PUBLIC jsoncpp_lib )
    target_link_libraries(mainstdin PUBLIC jsoncpp_lib)
endif()

target_compile_features(marginal_test PUBLIC cxx_std_20)
target_compile_features(sam-system-factor_test PUBLIC cxx_std_20)
target_compile_features(toy_linear_test PUBLIC cxx_std_20)
target_compile_features(toy_SE2_test PUBLIC cxx_std_20)
target_compile_features(toy_pose_graph_SE2_test PUBLIC cxx_std_20)
target_compile_features(toy_slam_SE2 PUBLIC cxx_std_20)
# mainstding
target_compile_features(mainstdin PUBLIC cxx_std_20)

# NOTE: to manage some definitions
# target_compile_definitions(mainstdin PUBLIC ENABLE_DEBUG_TRACE=1 ENABLE_TIMER=1)

# cmake_print_properties(
#     TARGETS factor_test
#     PROPERTIES POSITION_INDEPENDENT_CODE
# )

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)

enable_testing()

add_executable(factor_test test/factor_test.cpp)
target_include_directories(factor_test PUBLIC include ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(factor_test gtest gmock gtest_main)
target_compile_features(factor_test PUBLIC cxx_std_20)

include(GoogleTest)
gtest_discover_tests(factor_test)
