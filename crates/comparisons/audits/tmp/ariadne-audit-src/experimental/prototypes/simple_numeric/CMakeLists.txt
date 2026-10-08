cmake_minimum_required (VERSION 3.15)

set(CMAKE_CXX_STANDARD 20)

add_executable(check_simple_numeric check_simple_numeric.cpp)
target_link_libraries(check_simple_numeric)