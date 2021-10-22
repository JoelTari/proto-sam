cmake_minimum_required(VERSION 3.10)

project(proto-sam)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_CXX_STANDARD 17) # this will be reflected in compile commands 

add_executable(factor_test factor_test.cpp)
add_executable(sam-system_test sam-system_test.cpp)
add_executable(sam-system-factor_test sam-system-factor_test.cpp)
# add_library(test_factor test_factor.cpp)

target_compile_features(factor_test PUBLIC cxx_std_17) # BUG: is not reflected in compile commands
target_compile_features(sam-system_test PUBLIC cxx_std_17) # BUG: is not reflected in compile commands
target_compile_features(sam-system-factor_test PUBLIC cxx_std_17) # BUG: is not reflected in compile commands
