cmake_minimum_required(VERSION 3.15)
message(STATUS "CMake version ${CMAKE_VERSION}")

project(Ariadne VERSION 2.5.3)

if(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
    message(WARNING "Intel compiler not supported.")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    message(WARNING "Microsoft compiler not supported.")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 12.0)
        message(FATAL_ERROR "GCC version must be at least 12!")
    endif()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if ((NOT APPLE) AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 16.0))
        message(FATAL_ERROR "Clang version must be at least 16!")
    endif()
else()
    message(WARNING "You are using an unidentified compiler! Compilation has only been tested with Clang and GCC.")
endif()

set(ANY_TARGET_WARN all extra pedantic sign-conversion cast-qual disabled-optimization
                  init-self missing-include-dirs sign-promo switch-default undef redundant-decls)
set(ANY_TARGET_DONT_WARN strict-aliasing unused-parameter)
set(LIBRARY_EXCLUSIVE_WARN shadow)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(CMAKE_MACOSX_RPATH 1)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

foreach(WARN ${ANY_TARGET_WARN})
    add_compile_options(-W${WARN})
endforeach()
foreach(WARN ${ANY_TARGET_DONT_WARN})
    add_compile_options(-Wno-${WARN})
endforeach()
if(WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-Wno-non-template-friend)
endif()
if((CMAKE_CXX_COMPILER_ID MATCHES "GNU") OR (NOT APPLE))
    add_compile_options(-frounding-math)
endif()
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-fdiagnostics-color=always)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-fcolor-diagnostics)
endif ()

option(COVERAGE "Enable coverage reporting" OFF)

enable_testing()
include(CTest)

include(UseDoxygen OPTIONAL)

include(CheckTypeSize)
include(CheckIncludeFile)
include(CheckIncludeFiles)
include(CheckIncludeFileCXX)
include(CheckLibraryExists)

check_type_size("char" SIZEOF_CHAR)
check_type_size("short int" SIZEOF_SHORT_INT)
check_type_size("int" SIZEOF_INT)
check_type_size("long int" SIZEOF_LONG_INT)
check_type_size("long long int" SIZEOF_LONG_LONG_INT)
check_type_size("float" SIZEOF_FLOAT)
check_type_size("double" SIZEOF_DOUBLE)

check_include_file("dlfcn.h" HAVE_DLFCN_H)
check_include_file("execinfo.h" HAVE_EXECINFO_H)
check_include_file_cxx("cxxabi.h" HAVE_CXXABI_H)
if((${HAVE_DLFCN_H}) AND (${HAVE_EXECINFO_H}) AND (${HAVE_CXXABI_H}))
    set(ARIADNE_ENABLE_STACK_TRACE ON)
endif()

option(PACKAGED_SUBMODULES "Notify that the submodules are already available in their subdirectories; for release purposes" OFF)

if(NOT PACKAGED_SUBMODULES)
find_package(Git)
if(GIT_FOUND)
    if (NOT EXISTS ${PROJECT_SOURCE_DIR}/.git) # Manages the case when an archive is used
        execute_process(COMMAND git init WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_QUIET ERROR_QUIET)
        execute_process(COMMAND rm -Rf ${PROJECT_SOURCE_DIR}/python/pybind11 OUTPUT_QUIET ERROR_QUIET)
        execute_process(COMMAND git submodule add -f https://github.com/pybind/pybind11 python/pybind11 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_QUIET ERROR_QUIET)
        execute_process(COMMAND rm -Rf ${PROJECT_SOURCE_DIR}/submodules/helper OUTPUT_QUIET ERROR_QUIET)
        execute_process(COMMAND git submodule add -f https://github.com/ariadne-cps/helper submodules/helper WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_QUIET ERROR_QUIET)
        execute_process(COMMAND rm -Rf ${PROJECT_SOURCE_DIR}/submodules/conclog OUTPUT_QUIET ERROR_QUIET)
        execute_process(COMMAND git submodule add -f https://github.com/ariadne-cps/conclog submodules/conclog WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_QUIET ERROR_QUIET)
        execute_process(COMMAND rm -Rf ${PROJECT_SOURCE_DIR}/submodules/betterthreads OUTPUT_QUIET ERROR_QUIET)
        execute_process(COMMAND git submodule add -f https://github.com/ariadne-cps/betterthreads submodules/betterthreads WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_QUIET ERROR_QUIET)
    endif()
    message(STATUS "Getting all Ariadne submodules if not downloaded already...")
    execute_process(COMMAND git submodule update --init --recursive WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_QUIET ERROR_QUIET)
    if(NOT EXISTS ${PROJECT_SOURCE_DIR}/python/pybind11/CMakeLists.txt)
        message(ERROR "pybind11 download failure")
    else()
        message(STATUS "pybind11 submodule found")
    endif()
    if(NOT EXISTS ${PROJECT_SOURCE_DIR}/submodules/helper/CMakeLists.txt)
        message(ERROR "Helper download failure")
    else()
        message(STATUS "Helper submodule found")
    endif()
    if(NOT EXISTS ${PROJECT_SOURCE_DIR}/submodules/conclog/CMakeLists.txt)
        message(ERROR "ConcLog download failure")
    else()
        message(STATUS "ConcLog submodule found")
    endif()
    if(NOT EXISTS ${PROJECT_SOURCE_DIR}/submodules/betterthreads/CMakeLists.txt)
        message(ERROR "BetterThreads download failure")
    else()
        message(STATUS "BetterThreads submodule found")
    endif()
else()
    message(FATAL_ERROR "Git not found, no submodules (including pybind11 for Python bindings) could be downloaded")
endif()
endif()

if (NOT EXISTS ${PROJECT_SOURCE_DIR}/submodules/.symbolic)
    execute_process(COMMAND mkdir ${PROJECT_SOURCE_DIR}/submodules/.symbolic)
    message(STATUS "Created symbolic links directory for submodules")
endif()

if (NOT EXISTS ${PROJECT_SOURCE_DIR}/submodules/.symbolic/helper)
    execute_process(COMMAND ln -s ${PROJECT_SOURCE_DIR}/submodules/helper/include ${PROJECT_SOURCE_DIR}/submodules/.symbolic/helper)
    message(STATUS "Created symbolic link to ConcLog submodule")
endif()

if (NOT EXISTS ${PROJECT_SOURCE_DIR}/submodules/.symbolic/conclog)
    execute_process(COMMAND ln -s ${PROJECT_SOURCE_DIR}/submodules/conclog/include ${PROJECT_SOURCE_DIR}/submodules/.symbolic/conclog)
    message(STATUS "Created symbolic link to ConcLog submodule")
    endif()

if (NOT EXISTS ${PROJECT_SOURCE_DIR}/submodules/.symbolic/betterthreads)
    execute_process(COMMAND ln -s ${PROJECT_SOURCE_DIR}/submodules/betterthreads/include ${PROJECT_SOURCE_DIR}/submodules/.symbolic/betterthreads)
    message(STATUS "Created symbolic link to BetterThreads submodule")
endif()

find_package(Threads)
if(NOT Threads_FOUND)
    message(FATAL_ERROR "Unable to find Threads library")
endif()

find_package(GMP)
if(GMP_FOUND)
    include_directories(SYSTEM ${GMP_INCLUDES})
else()
    message(FATAL_ERROR "Unable to find GMP")
endif()

find_package(MPFR)
if(MPFR_FOUND)
    include_directories(SYSTEM ${MPFR_INCLUDES})
else()
    message(FATAL_ERROR "Unable to find MPFR")
endif()

find_package(Cairo)
if(CAIRO_FOUND)
    set(HAVE_CAIRO_H ON)
    include_directories(SYSTEM ${CAIRO_INCLUDE_DIRS})
endif()

find_package(Gnuplot 5.2.8)
if(GNUPLOT_FOUND)
    set(HAVE_GNUPLOT_H ON)
endif()

if(NOT CAIRO_FOUND AND NOT GNUPLOT_FOUND)
    message(STATUS "No Cairo or Gnuplot backend support found: figures can't be written to file")
endif()

add_custom_target(everything)
add_custom_target(tests)

configure_file(${PROJECT_SOURCE_DIR}/cmake/config.hpp.cmake ${PROJECT_SOURCE_DIR}/source/config.hpp)

include_directories(${PROJECT_SOURCE_DIR}/source ${PROJECT_SOURCE_DIR}/submodules/.symbolic)

add_subdirectory(submodules)

add_subdirectory(source)
add_subdirectory(python)

add_subdirectory(tests EXCLUDE_FROM_ALL)
add_subdirectory(examples EXCLUDE_FROM_ALL)
add_subdirectory(tutorials EXCLUDE_FROM_ALL)
add_subdirectory(experimental EXCLUDE_FROM_ALL)

add_library(ariadne-core SHARED
    $<TARGET_OBJECTS:ariadne-utility>
    $<TARGET_OBJECTS:ariadne-foundations>
    $<TARGET_OBJECTS:ariadne-numeric>
    $<TARGET_OBJECTS:ariadne-algebra>
    $<TARGET_OBJECTS:BETTERTHREADS_SRC>
    $<TARGET_OBJECTS:CONCLOG_SRC>
    $<TARGET_OBJECTS:HELPER_SRC>
    source/geometry/interval.cpp
)

add_library(ariadne-kernel SHARED
    $<TARGET_OBJECTS:ariadne-utility>
    $<TARGET_OBJECTS:ariadne-foundations>
    $<TARGET_OBJECTS:ariadne-numeric>
    $<TARGET_OBJECTS:ariadne-algebra>
    $<TARGET_OBJECTS:ariadne-function>
    $<TARGET_OBJECTS:ariadne-geometry>
    $<TARGET_OBJECTS:ariadne-solvers>
    $<TARGET_OBJECTS:ariadne-io>
    $<TARGET_OBJECTS:ariadne-symbolic>
    $<TARGET_OBJECTS:BETTERTHREADS_SRC>
    $<TARGET_OBJECTS:CONCLOG_SRC>
    $<TARGET_OBJECTS:HELPER_SRC>
)

add_library(ariadne SHARED
    $<TARGET_OBJECTS:ariadne-utility>
    $<TARGET_OBJECTS:ariadne-foundations>
    $<TARGET_OBJECTS:ariadne-numeric>
    $<TARGET_OBJECTS:ariadne-algebra>
    $<TARGET_OBJECTS:ariadne-function>
    $<TARGET_OBJECTS:ariadne-geometry>
    $<TARGET_OBJECTS:ariadne-solvers>
    $<TARGET_OBJECTS:ariadne-io>
    $<TARGET_OBJECTS:ariadne-symbolic>
    $<TARGET_OBJECTS:ariadne-dynamics>
    $<TARGET_OBJECTS:ariadne-hybrid>
    $<TARGET_OBJECTS:BETTERTHREADS_SRC>
    $<TARGET_OBJECTS:CONCLOG_SRC>
    $<TARGET_OBJECTS:HELPER_SRC>
)

target_link_libraries(ariadne dl ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(ariadne-kernel dl ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(ariadne-core dl ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

if(COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()

if(CAIRO_FOUND)
    target_link_libraries(ariadne ${CAIRO_LIBRARIES})
    target_link_libraries(ariadne-kernel ${CAIRO_LIBRARIES})
endif()

install(TARGETS ariadne DESTINATION lib)
install(DIRECTORY source/ DESTINATION include/ariadne FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY submodules/.symbolic/helper DESTINATION include FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY submodules/.symbolic/conclog DESTINATION include FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY submodules/.symbolic/betterthreads DESTINATION include FILES_MATCHING PATTERN "*.hpp")
