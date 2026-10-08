set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/experimental/features/kirk")

find_package(Kirk)

if(${KIRK_FOUND})
    set(EXTERNAL_SRC ${EXTERNAL_SRC} kirk_real.cpp)
endif()

if (${EXTERNAL_SRC})
    add_library(ariadne-external SHARED ${EXTERNAL_SRC})
    foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
        target_compile_options(ariadne-external PUBLIC -W${WARN})
    endforeach()
    target_link_libraries(ariadne-external ariadne)
    if(COVERAGE)
        include(CodeCoverage)
        append_coverage_compiler_flags()
    endif()
endif()

if(${KIRK_FOUND})
    target_link_libraries(ariadne-external kirk)
endif()

add_subdirectory(examples)
