include(CTest)

set(UNIT_TESTS
    test_array
    test_container
    test_lazy
    test_lru_cache
    test_stack_trace
    test_randomiser
    test_stopwatch
)

foreach(TEST ${UNIT_TESTS})
    add_executable(${TEST} ${TEST}.cpp)
    target_link_libraries(${TEST} helper)
    add_test(${TEST} ${TEST})
endforeach()

add_custom_target(tests)
add_dependencies(tests ${UNIT_TESTS})
