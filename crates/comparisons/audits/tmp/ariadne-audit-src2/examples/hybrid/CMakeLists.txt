set(EXAMPLES_HYBRID
    bouncingball
    heating
    rectifier
)

foreach(example ${EXAMPLES_HYBRID})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()

add_dependencies(examples ${EXAMPLES_HYBRID})

add_subdirectory(watertank)
