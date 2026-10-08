set(LIBRARY_NAME ariadne-algebra)

add_library(${LIBRARY_NAME} OBJECT
    vector.cpp
    matrix.cpp
    series.cpp
    multi_index.cpp
    expansion.cpp
    differential.cpp
    algebra.cpp
)

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
    target_compile_options(${LIBRARY_NAME} PUBLIC -W${WARN})
endforeach()

if(COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()