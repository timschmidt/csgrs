set(LIBRARY_NAME ariadne-symbolic)

add_library(${LIBRARY_NAME} OBJECT
    space.cpp
    expression.cpp
    expression_set.cpp
)

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
    target_compile_options(${LIBRARY_NAME} PUBLIC -W${WARN})
endforeach()

if(COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()
