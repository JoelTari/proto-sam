set(CMAKE_EXPORT_COMPILE_COMMANDS on)
add_library(sam_system STATIC src/MatrixConverter.cpp)
target_include_directories(sam_system PUBLIC include/ ${EIGEN_INCLUDE_DIRS})
target_compile_definitions(sam_system PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(sam_system PUBLIC cxx_std_20)
# TODO: only link jsoncpp_static if ENABLE_JSON_OUTPUT
target_link_libraries(sam_system PUBLIC jsoncpp_static utils solver ${LOW_LEVEL_LIBRARIES})

# doesnt link against factor ('merge' happens at next level)
