set(CMAKE_EXPORT_COMPILE_COMMANDS on)

add_library(factor_anchor2d STATIC src/anchor2d.cpp)
target_include_directories(factor_anchor2d PUBLIC ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(factor_anchor2d PUBLIC factor key_position_2d ${LOW_LEVEL_LIBRAIRIES}) #FIX: URGENT ultra tmp
target_link_libraries(factor_anchor2d PUBLIC factor meta ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(factor_anchor2d PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(factor_anchor2d PUBLIC cxx_std_20) # perhaps unecessary

add_library(factor_anchorSE2 STATIC src/anchorSE2.cpp)
target_include_directories(factor_anchorSE2 PUBLIC ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(factor_anchorSE2 PUBLIC factor key_pose_SE2 measure_pose_SE2 ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(factor_anchorSE2 PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(factor_anchorSE2 PUBLIC cxx_std_20) # perhaps unecessary

add_library(factor_cartesian_landmark_obs_SE2 STATIC src/cartesian-landmark-obs-SE2.cpp)
target_include_directories(factor_cartesian_landmark_obs_SE2 PUBLIC ${EIGEN3_INCLUDE_DIRS} ${MANIF_INCLUDE_DIRS} include)
target_link_libraries(factor_cartesian_landmark_obs_SE2 PUBLIC factor key_pose_SE2 key_position_2d measure_motion_2d ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(factor_cartesian_landmark_obs_SE2 PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(factor_cartesian_landmark_obs_SE2 PUBLIC cxx_std_20) # perhaps unecessary

add_library(factor_motion_model_SE2 STATIC src/motion-model-SE2.cpp)
target_include_directories(factor_motion_model_SE2 PUBLIC ${EIGEN3_INCLUDE_DIRS} ${MANIF_INCLUDE_DIRS} include)
target_link_libraries(factor_motion_model_SE2 PUBLIC factor key_pose_SE2 measure_velocity_SE2 ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(factor_motion_model_SE2 PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(factor_motion_model_SE2 PUBLIC cxx_std_20) # perhaps unecessary

add_library(factor_relative_matcher_2d STATIC src/relative-matcher-2d.cpp)
target_include_directories(factor_relative_matcher_2d PUBLIC ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(factor_relative_matcher_2d PUBLIC factor key_position_2d measure_motion_2d ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(factor_relative_matcher_2d PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(factor_relative_matcher_2d PUBLIC cxx_std_20) # perhaps unecessary

add_library(factor_relative_matcher_SE2 STATIC src/relative-matcher-2d.cpp)
target_include_directories(factor_relative_matcher_SE2 PUBLIC ${EIGEN3_INCLUDE_DIRS} ${MANIF_INCLUDE_DIRS} include)
target_link_libraries(factor_relative_matcher_SE2 PUBLIC factor key_pose_SE2 measure_motion_SE2 ${LOW_LEVEL_LIBRAIRIES})
target_compile_definitions(factor_relative_matcher_SE2 PUBLIC ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})
target_compile_features(factor_relative_matcher_SE2 PUBLIC cxx_std_20) # perhaps unecessary
