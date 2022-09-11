# tuple patterns and config are header only
add_library(utils INTERFACE)
target_include_directories(utils INTERFACE include/)
target_compile_definitions(utils INTERFACE ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})


# # utils (json + timer) is header only for now
# add_library(utils INTERFACE)
# target_include_directories(utils INTERFACE include/)
