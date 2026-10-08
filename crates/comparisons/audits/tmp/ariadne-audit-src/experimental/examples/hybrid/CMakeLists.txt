set(EXAMPLES_EXPERIMENTAL_HYBRID
	laser
)

foreach(example ${EXAMPLES_EXPERIMENTAL_HYBRID})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()

add_subdirectory(twowatertanks)
add_subdirectory(threewatertanks)