set(EXAMPLES_PDES
    dirichlet
    acoustic_first_order_pde
)

foreach(example ${EXAMPLES_PDES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()

add_custom_target(examples-pdes)
add_dependencies(examples-pdes ${EXAMPLES_PDES})
add_dependencies(examples ${EXAMPLES_PDES})
