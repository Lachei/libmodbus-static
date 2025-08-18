cmake_minimum_required(VERSION 3.16)

project(modbus-tcp-linux-server CXX)

add_executable(${PROJECT_NAME} 
	modbus-tcp-linux-server.cpp
)
target_link_libraries(${PROJECT_NAME} libmodbus-static)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 23)

