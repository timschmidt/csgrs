set(EXAMPLES_EXPERIMENTAL_ARCH
    ROBE25
    CVDP23
	LALO20
    LOVO25
    SPRE22
    SUTR21
    arch_suite
)

foreach(example ${EXAMPLES_EXPERIMENTAL_ARCH})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()
