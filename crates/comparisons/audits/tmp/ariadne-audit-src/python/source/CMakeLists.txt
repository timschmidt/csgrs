include_directories(${Python3_INCLUDE_DIRS})
include_directories(SYSTEM ${PYBIND11_INCLUDE_DIR})

message(STATUS "Please note that you need to use the Python 3 interpreter to import pyariadne correctly.")

add_library(pyariadne-numeric-submodule-obj OBJECT
    foundations_submodule.cpp
    numeric_submodule.cpp
)

add_library(pyariadne-numeric-module-obj OBJECT
    numeric_module.cpp
)

add_library(pyariadne-algebra-submodule-obj OBJECT
    utilities.cpp
    linear_algebra_submodule.cpp
    differentiation_submodule.cpp
    calculus_submodule.cpp
)

add_library(pyariadne-algebra-module-obj OBJECT
    algebra_module.cpp
)

add_library(pyariadne-extra-submodule-obj OBJECT
    symbolic_submodule.cpp
    optimization_submodule.cpp
    function_submodule.cpp
    geometry_submodule.cpp
    io_submodule.cpp
    concurrency_submodule.cpp
    solver_submodule.cpp
    storage_submodule.cpp
    system_submodule.cpp
    evolution_submodule.cpp
    graphics_submodule.cpp
    hybrid_submodule.cpp
)

add_library(pyariadne-module-obj OBJECT
    ariadne_module.cpp
)

foreach(WARN ${LIBRARY_EXCLUSIVE_WARN})
    target_compile_options(pyariadne-numeric-submodule-obj PUBLIC -W${WARN})
    target_compile_options(pyariadne-numeric-module-obj PUBLIC -W${WARN})
    target_compile_options(pyariadne-algebra-submodule-obj PUBLIC -W${WARN})
    target_compile_options(pyariadne-algebra-module-obj PUBLIC -W${WARN})
    target_compile_options(pyariadne-extra-submodule-obj PUBLIC -W${WARN})
    target_compile_options(pyariadne-module-obj PUBLIC -W${WARN})
endforeach()
