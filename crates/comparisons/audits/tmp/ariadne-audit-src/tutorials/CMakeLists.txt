set(TUTORIALS
    rigorous_numerics
    hybrid_evolution
)

add_custom_target(tutorials)

add_dependencies(everything tutorials)

foreach(tutorial ${TUTORIALS})
    add_executable(${tutorial}_tutorial ${tutorial}/${tutorial}_tutorial.cpp)
    target_link_libraries(${tutorial}_tutorial ariadne)
    add_dependencies(tutorials ${tutorial}_tutorial)
endforeach()

add_subdirectory(demonstrations)

