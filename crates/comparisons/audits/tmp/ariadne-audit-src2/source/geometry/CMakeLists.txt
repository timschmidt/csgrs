set(LIBRARY_NAME ariadne-geometry)

add_library(${LIBRARY_NAME} OBJECT
    point.cpp
    interval.cpp
    box.cpp
    curve.cpp
    set.cpp
    function_set.cpp
    binary_tree.cpp
    grid.cpp
    grid_paving.cpp
    paver.cpp
    affine_set.cpp

    geometry.cpp
)

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
    target_compile_options(${LIBRARY_NAME} PUBLIC -W${WARN})
endforeach()

if(COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()
