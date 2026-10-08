set(LIBRARY_NAME BETTERTHREADS_SRC)

add_library(${LIBRARY_NAME} OBJECT
        thread.cpp
        buffered_thread.cpp
        thread_pool.cpp
        workload_advancement.cpp
        thread_manager.cpp
        )

if(COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
    target_compile_options(${LIBRARY_NAME} PUBLIC -W${WARN})
endforeach()
