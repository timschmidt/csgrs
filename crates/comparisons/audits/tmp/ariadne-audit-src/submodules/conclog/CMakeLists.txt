cmake_minimum_required(VERSION 3.15)

project(ConcLog VERSION 1.0)

option(COVERAGE "Enable coverage reporting" OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.20)
        message(FATAL_ERROR "MSVC version must be at least 19.20!")
    endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    if (NOT EXISTS "/etc/nv_tegra_release")
        if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10.2)
            message(FATAL_ERROR "GCC version must be at least 10.2!")
        endif()
    endif()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 11.0)
        message(FATAL_ERROR "Clang version must be at least 11.0!")
    endif()
else()
    message(WARNING "You are using an unsupported compiler! MSVC, GCC and Clang are supported.")
endif()

if(WIN32)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    set(LIBRARY_KIND STATIC)
elseif(COVERAGE)
    set(LIBRARY_KIND STATIC)
else()
    set(LIBRARY_KIND SHARED)	
endif()

set(CMAKE_MACOSX_RPATH 1)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(WIN32)
    set(CMAKE_CXX_STANDARD 20)
else()
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT WIN32)
    set(ANY_TARGET_WARN all extra pedantic sign-conversion cast-qual disabled-optimization
        init-self missing-include-dirs sign-promo switch-default undef redundant-decls
        strict-aliasing unused-parameter shadow error)
    foreach(WARN ${ANY_TARGET_WARN})
        add_compile_options(-W${WARN})
    endforeach()
else()
    add_compile_options(/WX)
endif()

find_package(Threads REQUIRED)

include_directories(SYSTEM ${PROJECT_SOURCE_DIR}/include)

if(NOT TARGET conclog)
    add_subdirectory(src)

    add_library(conclog ${LIBRARY_KIND} $<TARGET_OBJECTS:CONCLOG_SRC>)
    target_link_libraries(conclog ${CMAKE_THREAD_LIBS_INIT})

    if(NOT TARGET tests)

        if(COVERAGE)
            include(CodeCoverage)
            append_coverage_compiler_flags()
        endif()

        enable_testing()
        add_subdirectory(test)
    endif()

endif()
