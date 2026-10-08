# Preferable to avoid warnings during execution
cmake_minimum_required(VERSION 3.15)

project(HybridEvolutionTutorial VERSION 2.5.3)

# Set to search the root source directory for the FindAriadne.cmake module
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

# Set the C++ standard to C++20, as used by Ariadne
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Ariadne
find_package(Ariadne REQUIRED)

# Add the headers to the includes
include_directories(SYSTEM ${ARIADNE_INCLUDE_DIRS})

# Set the executable along with the required source files
add_executable(hybrid_evolution_tutorial hybrid_evolution_tutorial.cpp)

# Instruct to link against the ariadne library
target_link_libraries(hybrid_evolution_tutorial ${ARIADNE_LIBRARIES})
