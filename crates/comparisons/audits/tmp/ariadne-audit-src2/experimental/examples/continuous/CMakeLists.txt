set(EXAMPLES_EXPERIMENTAL_CONTINUOUS
    laub-loomis
)

foreach(example ${EXAMPLES_EXPERIMENTAL_CONTINUOUS})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()

add_subdirectory(noisy)