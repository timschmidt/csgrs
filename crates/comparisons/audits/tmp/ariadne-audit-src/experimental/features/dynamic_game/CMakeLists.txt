add_executable(dynamic_game dynamic_game.cpp)
target_include_directories(dynamic_game PUBLIC ../../../source/symbolic)
target_link_libraries(dynamic_game ariadne)