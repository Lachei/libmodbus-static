cmake_minimum_required(VERSION 3.16)

project(libmodbus-static-test CXX)

add_executable(${PROJECT_NAME} 
	main.cpp
)
target_link_libraries(${PROJECT_NAME} libmodbus-static)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 23)

