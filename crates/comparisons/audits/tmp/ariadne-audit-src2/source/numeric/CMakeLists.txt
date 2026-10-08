set(LIBRARY_NAME ariadne-numeric)

add_library(${LIBRARY_NAME} OBJECT
    operators.cpp
    int.cpp
    double.cpp
    integer.cpp
    dyadic.cpp
    decimal.cpp
    rational.cpp
    real.cpp
    complex.cpp
    number.cpp
    floatdp.cpp
    floatmp.cpp
    float_approximation.cpp
    float_lower_bound.cpp
    float_upper_bound.cpp
    float_bounds.cpp
    float_ball.cpp
    float_error.cpp
    float_literals.cpp

    concepts.cpp
)

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
     target_compile_options(${LIBRARY_NAME} PUBLIC -W${WARN})
endforeach()

if(COVERAGE)
     include(CodeCoverage)
     append_coverage_compiler_flags()
endif()

