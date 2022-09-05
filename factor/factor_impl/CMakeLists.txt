
add_library(factor_anchor2d STATIC src/anchor2d.cpp)

target_include_directories(factor_anchor2d PUBLIC ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(factor_anchor2d PRIVATE key_position_2d PUBLIC factor ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(factor_anchor2d PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(factor_anchor2d PUBLIC cxx_std_20) # perhaps unecessary
