set(LIBRARY_NAME ariadne-solvers)

add_library(${LIBRARY_NAME} OBJECT
    solver.cpp
    integrator.cpp
    inclusion_integrator.cpp
    bounder.cpp
    constraint_solver.cpp
    simplex_algorithm.cpp
    linear_programming.cpp
    nonlinear_programming.cpp
    runge_kutta_integrator.cpp
)

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
    target_compile_options(${LIBRARY_NAME} PUBLIC -W${WARN})
endforeach()

if(COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()