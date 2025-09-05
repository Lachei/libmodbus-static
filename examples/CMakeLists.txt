cmake_minimum_required(VERSION 3.16)

project(examples CXX)

add_executable(raw-modbus-tcp-linux-server 
	raw-modbus-tcp-linux-server.cpp
)
target_link_libraries(raw-modbus-tcp-linux-server libmodbus-static)
set_property(TARGET raw-modbus-tcp-linux-server PROPERTY CXX_STANDARD 23)


add_executable(modbus-tcp-linux-client 
	modbus-tcp-linux-client.cpp
)
target_link_libraries(modbus-tcp-linux-client libmodbus-static)
set_property(TARGET modbus-tcp-linux-client PROPERTY CXX_STANDARD 23)

