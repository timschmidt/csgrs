set(LIBRARY_NAME ariadne-foundations)

add_library(${LIBRARY_NAME} OBJECT
    logical.cpp
)

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
     target_compile_options(${LIBRARY_NAME} PUBLIC -W${WARN})
endforeach()

if(COVERAGE)
     include(CodeCoverage)
     append_coverage_compiler_flags()
endif()

