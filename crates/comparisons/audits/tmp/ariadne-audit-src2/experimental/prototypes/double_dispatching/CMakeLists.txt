cmake_minimum_required (VERSION 3.15)

set(CMAKE_CXX_STANDARD 20)

add_executable(double_dispatching double_dispatching.cpp)
target_link_libraries(double_dispatching ${GMP_LIBRARIES} ${GMPXX_LIBRARIES})