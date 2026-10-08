set(UNIT_TESTS
    test_array
    test_container
    test_lru_cache
    test_stopwatch
    test_stack_trace
)

foreach(TEST ${UNIT_TESTS})
    add_executable(${TEST} ${TEST}.cpp)
    target_link_libraries(${TEST} ariadne-kernel)  
    add_test(${TEST} ${TEST})
    set_property(TEST ${TEST} PROPERTY LABELS "utility;kernel")
endforeach()

add_dependencies(tests-kernel ${UNIT_TESTS})
add_dependencies(tests-cpp ${UNIT_TESTS})
add_dependencies(tests ${UNIT_TESTS})
