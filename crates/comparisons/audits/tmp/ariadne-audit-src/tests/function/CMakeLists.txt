set(UNIT_TESTS
    test_function
    test_measurable_function
    test_multifunction
    test_polynomial
    test_chebyshev_polynomial
    test_formula
    test_procedure
    test_taylor_function
    test_taylor_model
)

foreach(TEST ${UNIT_TESTS})
    add_executable(${TEST} ${TEST}.cpp)
    target_link_libraries(${TEST} ariadne-kernel)
    add_test(${TEST} ${TEST})
    set_property(TEST ${TEST} PROPERTY LABELS "function;kernel")
endforeach()

add_dependencies(tests-kernel ${UNIT_TESTS})
add_dependencies(tests-cpp ${UNIT_TESTS})
add_dependencies(tests ${UNIT_TESTS})

#add_library(check_function OBJECT check_function.cpp)
#add_dependencies(tests-kernel check_function)
#add_dependencies(tests check_function)
