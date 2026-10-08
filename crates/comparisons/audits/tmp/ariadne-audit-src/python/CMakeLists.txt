find_package(Python3 3.12 COMPONENTS Interpreter Development)

option(PYTHON_ENFORCE_TESTS "Enforce Python tests" OFF)

if(PYTHON3_FOUND)

    add_subdirectory(pybind11)
    add_subdirectory(source)

    add_library(pyariadne MODULE
            $<TARGET_OBJECTS:pyariadne-module-obj>
            $<TARGET_OBJECTS:pyariadne-numeric-submodule-obj>
            $<TARGET_OBJECTS:pyariadne-algebra-submodule-obj>
            $<TARGET_OBJECTS:pyariadne-extra-submodule-obj>
            )
    set_target_properties(pyariadne PROPERTIES PREFIX "" OUTPUT_NAME "pyariadne" SUFFIX ".so")
    target_link_libraries (pyariadne PUBLIC ariadne PRIVATE pybind11::module)

    add_library(pyariadne-numeric MODULE
            $<TARGET_OBJECTS:pyariadne-numeric-module-obj>
            $<TARGET_OBJECTS:pyariadne-numeric-submodule-obj>
            $<TARGET_OBJECTS:ariadne-utility>
            $<TARGET_OBJECTS:ariadne-foundations>
            $<TARGET_OBJECTS:ariadne-numeric>
            )
    set_target_properties(pyariadne-numeric PROPERTIES PREFIX "" OUTPUT_NAME "pyariadne_numeric" SUFFIX ".so")
    target_link_libraries (pyariadne-numeric PUBLIC ${MPFR_LIBRARIES} ${GMP_LIBRARIES} PRIVATE pybind11::module)

    add_library(pyariadne-algebra MODULE
            $<TARGET_OBJECTS:pyariadne-algebra-module-obj>
            $<TARGET_OBJECTS:pyariadne-numeric-submodule-obj>
            $<TARGET_OBJECTS:pyariadne-algebra-submodule-obj>
            )
    set_target_properties(pyariadne-algebra PROPERTIES PREFIX "" OUTPUT_NAME "pyariadne_algebra" SUFFIX ".so")
    target_link_libraries (pyariadne-algebra PUBLIC ariadne PRIVATE pybind11::module)

    set(PYTHON_BINDINGS_INSTALL_DIR "${Python3_SITEARCH}"
            CACHE STRING "Install destination for Python targets")
    install(TARGETS pyariadne DESTINATION "${PYTHON_BINDINGS_INSTALL_DIR}")

    add_subdirectory(tests)

else()
    message(STATUS "Python3 not found: Python bindings module 'pyariadne' will not be built.")
endif()
