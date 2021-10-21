cmake_minimum_required(VERSION 3.10)

project(proto-sam)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_CXX_STANDARD 17) # this will be reflected in compile commands 

add_executable(test_factor test_factor.cpp)
# add_library(test_factor test_factor.cpp)

target_compile_features(test_factor PUBLIC cxx_std_17) # BUG: is not reflected in compile commands
