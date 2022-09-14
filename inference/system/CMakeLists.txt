set(CMAKE_EXPORT_COMPILE_COMMANDS on)
add_library(sam_system_interface INTERFACE)
target_include_directories(sam_system_interface INTERFACE include/ ${EIGEN_INCLUDE_DIRS})
target_compile_definitions(sam_system_interface INTERFACE ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(sam_system_interface INTERFACE cxx_std_20)
# TODO: only link jsoncpp_static if ENABLE_JSON_OUTPUT
target_link_libraries(sam_system_interface INTERFACE jsoncpp_static utils solver ${LOW_LEVEL_LIBRARIES})

# doesnt link against factor ('merge' happens at next level)
