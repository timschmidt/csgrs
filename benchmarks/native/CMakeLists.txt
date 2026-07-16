cmake_minimum_required(VERSION 3.20)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
endif()

project(csgrs_native_benchmarks LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CSGRS_BENCH_REQUIRE_ALL "Fail unless both comparison kernels are found" ON)

find_package(CGAL QUIET)
if(CGAL_FOUND)
  add_executable(csgrs_bench_cgal cgal.cpp)
  target_link_libraries(csgrs_bench_cgal PRIVATE CGAL::CGAL)
  target_compile_options(csgrs_bench_cgal PRIVATE
    -O3
    -DNDEBUG
    -Wno-stringop-overflow
  )
elseif(CSGRS_BENCH_REQUIRE_ALL)
  message(FATAL_ERROR "CGAL was not found")
else()
  message(WARNING "CGAL was not found; skipping its benchmark runner")
endif()

find_package(OpenCASCADE QUIET COMPONENTS
  FoundationClasses
  ModelingData
  ModelingAlgorithms
)
if(OpenCASCADE_FOUND)
  find_library(CSGRS_OCCT_STL_LIBRARY
    NAMES TKDESTL TKSTL
    HINTS ${OpenCASCADE_LIBRARY_DIR}
  )
  if(NOT CSGRS_OCCT_STL_LIBRARY)
    message(FATAL_ERROR "OpenCascade STL writer library was not found")
  endif()
  add_executable(csgrs_bench_occt opencascade.cpp)
  target_include_directories(csgrs_bench_occt PRIVATE ${OpenCASCADE_INCLUDE_DIR})
  target_link_libraries(csgrs_bench_occt PRIVATE
    ${OpenCASCADE_FoundationClasses_LIBRARIES}
    ${OpenCASCADE_ModelingData_LIBRARIES}
    ${OpenCASCADE_ModelingAlgorithms_LIBRARIES}
    ${CSGRS_OCCT_STL_LIBRARY}
  )
  target_compile_options(csgrs_bench_occt PRIVATE -O3 -DNDEBUG)
elseif(CSGRS_BENCH_REQUIRE_ALL)
  message(FATAL_ERROR "OpenCascade/OCCT was not found")
else()
  message(WARNING "OpenCascade/OCCT was not found; skipping its benchmark runner")
endif()
