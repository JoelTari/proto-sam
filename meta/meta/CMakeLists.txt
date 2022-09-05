# header-only templates
add_library(meta INTERFACE)
target_include_directories(meta INTERFACE include/)
target_compile_definitions(meta INTERFACE ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})

