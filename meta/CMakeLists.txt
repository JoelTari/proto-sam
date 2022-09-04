
# header-only templates
add_library(meta INTERFACE)
target_include_directories(meta INTERFACE include/)
target_compile_definitions(meta INTERFACE ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})

#--------------------------------------------------------------------#
#                      key meta implementations                      #
#--------------------------------------------------------------------#
add_library(key_position_2d STATIC src/key-position-2d.cpp)
target_include_directories(key_position_2d PUBLIC ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(key_position_2d PRIVATE meta PUBLIC ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(key_position_2d PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(key_position_2d PUBLIC cxx_std_20)
# # PRIVATE means users (such as factors) will have neither the stuff included nor defined by meta


# pose SE2
add_library(key_pose_SE2 STATIC src/key-pose-SE2.cpp)
target_include_directories(key_pose_SE2 PUBLIC ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(key_pose_SE2 PRIVATE meta PUBLIC ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(key_pose_SE2 PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(key_pose_SE2 PUBLIC cxx_std_20)
