cmake_minimum_required(VERSION 3.10)

project(proto-sam)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_CXX_STANDARD 17) # this will be reflected in compile commands

add_executable(factor_test test/factor_test.cpp)
add_executable(sam-system_test test/sam-system_test.cpp)
add_executable(sam-system-factor_test test/sam-system-factor_test.cpp)
add_executable(main_stdinput main_stdinput.cpp)

target_include_directories(factor_test PUBLIC core)
target_include_directories(sam-system_test PUBLIC core)
target_include_directories(sam-system-factor_test PUBLIC core)
target_include_directories(main_stdinput PUBLIC core)

# BUG: cxx_std_17 is not reflected afterwards in compile commands
target_compile_features(factor_test PUBLIC cxx_std_17)
target_compile_features(sam-system_test PUBLIC cxx_std_17)
target_compile_features(sam-system-factor_test PUBLIC cxx_std_17)
target_compile_features(main_stdinput PUBLIC cxx_std_17)

# target_compile_definitions(main_stdinput PUBLIC ENABLE_DEBUG_TRACE=1 ENABLE_TIMER=1)
