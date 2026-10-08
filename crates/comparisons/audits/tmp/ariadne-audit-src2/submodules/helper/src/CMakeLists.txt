set(LIBRARY_NAME HELPER_SRC)

add_library(${LIBRARY_NAME} OBJECT
        stack_trace.cpp
        )

if(COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
    target_compile_options(${LIBRARY_NAME} PUBLIC -W${WARN})
endforeach()
