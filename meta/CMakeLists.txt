
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



#--------------------------------------------------------------------#
#                    measure meta implementations                    #
#--------------------------------------------------------------------#

# pose SE2 (used for anchorSE2 etc..)
add_library(measure_pose_SE2 STATIC src/measure-pose-SE2.cpp)
target_include_directories(measure_pose_SE2 PUBLIC ${manif_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(measure_pose_SE2 PRIVATE meta PUBLIC ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(measure_pose_SE2 PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(measure_pose_SE2 PUBLIC cxx_std_20)

# position 2d (used for anchor2d etc..)
add_library(measure_position_2d STATIC src/measure-position-2d.cpp)
target_include_directories(measure_position_2d PUBLIC ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(measure_position_2d PRIVATE meta PUBLIC ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(measure_position_2d PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(measure_position_2d PUBLIC cxx_std_20)

# rigid body motion 2d (used for 2d odometry etc..)
add_library(measure_motion_2d STATIC src/measure-motion-2d.cpp)
target_include_directories(measure_motion_2d PUBLIC ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(measure_motion_2d PRIVATE meta PUBLIC ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(measure_motion_2d PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(measure_motion_2d PUBLIC cxx_std_20)

# rigid body motion SE2
add_library(measure_motion_SE2 STATIC src/measure-motion-SE2.cpp)
target_include_directories(measure_motion_SE2 PUBLIC ${EIGEN3_INCLUDE_DIRS} ${manif_INCLUDE_DIRS} include)
target_link_libraries(measure_motion_SE2 PRIVATE meta PUBLIC ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(measure_motion_SE2 PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(measure_motion_SE2 PUBLIC cxx_std_20)

# velocity SE2
add_library(measure_velocity_SE2 STATIC src/measure-velocity-SE2.cpp)
target_include_directories(measure_velocity_SE2 PUBLIC ${EIGEN3_INCLUDE_DIRS} ${manif_INCLUDE_DIRS} include)
target_link_libraries(measure_velocity_SE2 PRIVATE meta PUBLIC ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(measure_velocity_SE2 PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(measure_velocity_SE2 PUBLIC cxx_std_20)
