set(EXAMPLES_CONTINUOUS_NOISY
	higgins-selkov
	chemical-reactor
	lotka-volterra
	jet-engine
	pi-controller
	jerk21
	lorenz-attractor
	rossler-attractor
	jerk16
	dc-dc
	
	noisy-benchmark
)

foreach(example ${EXAMPLES_CONTINUOUS_NOISY})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} ariadne)
endforeach()

add_custom_target(examples-continuous-noisy)
add_dependencies(examples-continuous-noisy ${EXAMPLES_CONTINUOUS_NOISY})
add_dependencies(examples ${EXAMPLES_CONTINUOUS_NOISY})
