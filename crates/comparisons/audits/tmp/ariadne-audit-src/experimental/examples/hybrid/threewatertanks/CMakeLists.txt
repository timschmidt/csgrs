set(EXAMPLES_THREEWATERTANKS
    threewatertanks
)

foreach(example ${EXAMPLES_THREEWATERTANKS})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()

add_dependencies(examples ${EXAMPLES_THREEWATERTANKS})
