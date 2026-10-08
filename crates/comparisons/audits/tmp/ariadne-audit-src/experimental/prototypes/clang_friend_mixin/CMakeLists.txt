cmake_minimum_required (VERSION 3.15)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_COMPILER clang++)

add_library(clang_friend_mixin SHARED clang_friend_mixin.cpp)

add_executable(test_clang_friend_mixin test_clang_friend_mixin.cpp)
target_link_libraries(test_clang_friend_mixin clang_friend_mixin)