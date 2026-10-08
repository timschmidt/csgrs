set(UNIT_TESTS
    test_buffer
    test_buffered_thread
    test_thread
    test_thread_pool
    test_thread_manager
    test_workload_advancement
    test_workload
)

foreach(TEST ${UNIT_TESTS})
    add_executable(${TEST} ${TEST}.cpp)
    target_link_libraries(${TEST} betterthreads)
    add_test(${TEST} ${TEST})
endforeach()

add_custom_target(tests)
add_dependencies(tests ${UNIT_TESTS})
