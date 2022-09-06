# header-only templates
add_library(factor_interface INTERFACE)
target_include_directories(factor_interface INTERFACE include/)
target_compile_definitions(factor_interface INTERFACE ${LOW_LEVEL_DEFINITIONS} ${COMPILED_WITH})




