# header-only templates
add_library(factor INTERFACE)
target_include_directories(factor INTERFACE include/)
target_compile_definitions(factor INTERFACE ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})




