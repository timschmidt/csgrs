add_executable(control_validator control_validator.cpp)
target_include_directories(control_validator PUBLIC ../../../source/function)
target_link_libraries(control_validator ariadne)