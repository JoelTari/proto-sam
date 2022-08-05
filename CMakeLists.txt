cmake_minimum_required(VERSION 3.10)

project(proto-sam)

set(CMAKE_BUILD_TYPE Debug)
# add_definitions("-Wfatal-errors")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(manif REQUIRED)
find_package(jsoncpp 1.9.4 REQUIRED CONFIG)  # this sets 'jsoncpp_lib'
find_package(OpenMP REQUIRED)

set(CMAKE_CXX_STANDARD 20) # ISSUE: not reflected in compile_commands.json, which makes clangd still thinks its c++14 (it does when it's clang actually)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

add_executable(factor_test test/factor_test.cpp)
add_executable(marginal_test test/marginal_test.cpp)
add_executable(sam-system-factor_test test/sam-system-factor_test.cpp)
add_executable(toy_linear_test test/toy_linear_test.cpp)
add_executable(toy_SE2_test test/toy_SE2_test.cpp)
add_executable(toy_pose_graph_SE2_test test/toy_pose_graph_SE2_test.cpp)
add_executable(mainstdin main/mainstdin.cpp)

target_include_directories(factor_test PUBLIC include)
target_include_directories(marginal_test PUBLIC include)
target_include_directories(sam-system-factor_test PUBLIC include)
target_include_directories(toy_linear_test PUBLIC include)
target_include_directories(toy_SE2_test PUBLIC include ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_include_directories(toy_pose_graph_SE2_test PUBLIC include ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_include_directories(mainstdin PUBLIC include)

# TODO: try jsoncpp_static and compare size of binaries
target_link_libraries(sam-system-factor_test jsoncpp_lib)
target_link_libraries(factor_test jsoncpp_lib)
target_link_libraries(toy_linear_test jsoncpp_lib)
target_link_libraries(toy_pose_graph_SE2_test jsoncpp_lib)
if(OpenMP_CXX_FOUND)
    message("openMP found !")
    target_link_libraries(mainstdin PUBLIC OpenMP::OpenMP_CXX jsoncpp_lib)
else()
    target_link_libraries(mainstdin PUBLIC jsoncpp_lib)
endif()

# BUG: cxx_std_20 is not reflected afterwards in compile commands (It does when compiler is clang actually)
target_compile_features(factor_test PUBLIC cxx_std_20)
target_compile_features(marginal_test PUBLIC cxx_std_20)
target_compile_features(sam-system-factor_test PUBLIC cxx_std_20)
target_compile_features(toy_linear_test PUBLIC cxx_std_20)
target_compile_features(toy_SE2_test PUBLIC cxx_std_20)
target_compile_features(toy_pose_graph_SE2_test PUBLIC cxx_std_20)
# mainstding
target_compile_features(mainstdin PUBLIC cxx_std_20)
# target_compile_options(mainstdin PUBLIC "-O3" "-std=c++20") # "-fopt-info")

# NOTE: to manage some definitions
# target_compile_definitions(mainstdin PUBLIC ENABLE_DEBUG_TRACE=1 ENABLE_TIMER=1)


# TODO: For gtest 
# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   GIT_REPOSITORY https://github.com/google/googletest.git
#   GIT_TAG release-1.12.1
# )
