# header-only templates
add_library(meta_interface INTERFACE)
target_include_directories(meta_interface INTERFACE include/)
target_compile_definitions(meta_interface INTERFACE ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})

