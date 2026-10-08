add_library(CONCLOG_SRC OBJECT
   logging.cpp
)

if(COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()