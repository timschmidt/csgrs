cmake_minimum_required(VERSION 3.20)
project(hypermesh_cgal_epeck LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(CGAL 6.0.3 EXACT REQUIRED)

add_executable(hypermesh_cgal_epeck main.cpp)
target_link_libraries(hypermesh_cgal_epeck PRIVATE CGAL::CGAL)
target_compile_options(
  hypermesh_cgal_epeck
  PRIVATE
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-DNDEBUG>
)
