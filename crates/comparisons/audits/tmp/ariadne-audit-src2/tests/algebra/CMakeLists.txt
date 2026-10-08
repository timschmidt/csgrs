set(UNIT_TESTS
    test_differential
    test_algebra
    test_expansion
    test_graded
    test_matrix
    test_multi_index
    test_series
    test_vector
)

foreach(TEST ${UNIT_TESTS})
    add_executable(${TEST} ${TEST}.cpp)
    target_link_libraries(${TEST} ariadne-core)
    add_test(${TEST} ${TEST})
    set_property(TEST ${TEST} PROPERTY LABELS "algebra;core;kernel")
endforeach()

add_dependencies(tests-core ${UNIT_TESTS})
add_dependencies(tests-kernel ${UNIT_TESTS})
add_dependencies(tests-cpp ${UNIT_TESTS})
add_dependencies(tests ${UNIT_TESTS})

#add_library(check_algebra OBJECT check_algebra.cpp)
#add_dependencies(tests-core check_algebra)
#add_dependencies(tests-kernel check_algebra)
#add_dependencies(tests check_algebra)
