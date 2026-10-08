set(EXAMPLES_DISCRETE
    henon_map
)

foreach(example ${EXAMPLES_DISCRETE})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()

add_dependencies(examples ${EXAMPLES_DISCRETE})

