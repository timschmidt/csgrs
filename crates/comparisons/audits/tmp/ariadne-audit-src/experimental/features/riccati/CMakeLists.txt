add_executable(riccati riccati.cpp)
target_include_directories(riccati PUBLIC ../../../source/dynamics)
target_link_libraries(riccati ariadne)