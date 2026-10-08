set(EXAMPLES_TWOWATERTANKS
    twowatertanks
)

foreach(example ${EXAMPLES_TWOWATERTANKS})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()

add_dependencies(examples ${EXAMPLES_TWOWATERTANKS})
