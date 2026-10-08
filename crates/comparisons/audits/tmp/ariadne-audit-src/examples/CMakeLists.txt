add_custom_target(examples)

add_dependencies(everything examples)

add_subdirectory(discrete)
add_subdirectory(continuous)
add_subdirectory(hybrid)
add_subdirectory(pdes)
