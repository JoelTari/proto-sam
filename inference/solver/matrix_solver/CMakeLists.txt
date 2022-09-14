set(CMAKE_EXPORT_COMPILE_COMMANDS on)

add_library(solver STATIC src/solver.cpp)
target_include_directories(solver PUBLIC ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(solver PUBLIC jsoncpp_static utils ${LOW_LEVEL_LIBRARIES})
target_compile_definitions(solver PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(solver PUBLIC cxx_std_20)


