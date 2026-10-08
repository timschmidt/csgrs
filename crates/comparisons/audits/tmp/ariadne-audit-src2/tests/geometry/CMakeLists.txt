set(UNIT_TESTS
    test_affine_sets
    test_binary_tree
    test_box
    test_point_curve
    test_constrained_image_set
    test_interval
    test_grid_paving
    test_paving
    test_measurable_set
)

foreach(TEST ${UNIT_TESTS})
    add_executable(${TEST} ${TEST}.cpp)
    target_link_libraries(${TEST} ariadne-kernel)
    add_test(${TEST} ${TEST})
    set_property(TEST ${TEST} PROPERTY LABELS "geometry;kernel")
endforeach()

add_dependencies(tests-kernel ${UNIT_TESTS})
add_dependencies(tests-cpp ${UNIT_TESTS})
add_dependencies(tests ${UNIT_TESTS})
