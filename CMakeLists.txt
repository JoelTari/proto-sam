cmake_minimum_required(VERSION 3.10)

project(proto-sam)

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

target_compile_features(factor_test PUBLIC cxx_std_17) # BUG: cxx_std_17 is not reflected afterwards in compile commands
target_compile_features(sam-system_test PUBLIC cxx_std_17) # BUG: is not reflected in compile commands
target_compile_features(sam-system-factor_test PUBLIC cxx_std_17) # BUG: is not reflected in compile commands
