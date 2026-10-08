set(EXAMPLES_WATERTANK
    watertank-hysteresis
    watertank-proportional
)

foreach(example ${EXAMPLES_WATERTANK})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()

add_dependencies(examples ${EXAMPLES_WATERTANK})
