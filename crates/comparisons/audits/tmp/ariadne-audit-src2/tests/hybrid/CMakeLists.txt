set(UNIT_TESTS
    test_hybrid_automaton
    test_hybrid_evolution    
    test_hybrid_evolver
    test_hybrid_graphics
    test_hybrid_enclosure
    test_hybrid_set
    test_hybrid_simulator
    test_hybrid_reachability
)

foreach(TEST ${UNIT_TESTS})
    add_executable(${TEST} ${TEST}.cpp)
    target_link_libraries(${TEST} ariadne)  
    add_test(${TEST} ${TEST})    
    set_property(TEST ${TEST} PROPERTY LABELS "hybrid")
endforeach()

add_dependencies(tests-cpp ${UNIT_TESTS})
add_dependencies(tests ${UNIT_TESTS})