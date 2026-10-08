set(LIBRARY_NAME ariadne-hybrid)

add_library(${LIBRARY_NAME} OBJECT
    discrete_location.cpp
    hybrid_set.cpp
    hybrid_paving.cpp
    hybrid_enclosure.cpp
    hybrid_automaton.cpp
    hybrid_automaton-composite.cpp
        hybrid_simulator.cpp
    hybrid_evolver.cpp
    hybrid_reachability_analyser.cpp
    hybrid_graphics.cpp
)

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
    target_compile_options(${LIBRARY_NAME} PUBLIC -W${WARN})
endforeach()

if(COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()
