set(TUTORIALS_DEMONSTRATION
    numeric_demonstration
    algebra_demonstration
    function_demonstration
    geometry_demonstration
    solver_demonstration
)

add_custom_target(demonstrations)

foreach(demonstration ${TUTORIALS_DEMONSTRATION})
    add_executable(${demonstration} ${demonstration}.cpp)
    add_dependencies(demonstrations ${demonstration})
    target_link_libraries(${demonstration} ariadne)
endforeach()

add_dependencies(tutorials demonstrations)
