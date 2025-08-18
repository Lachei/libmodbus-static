cmake_minimum_required(VERSION 3.16)

project(libmodbus-static CXX)

add_library(${PROJECT_NAME} STATIC
	include/modbus-register.h
	include/common.h
	src/common.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)

option(BUILD_TESTS "Build unit tests" ON)
if (${BUILD_TESTS})
	add_subdirectory(test)
endif ()

option(BUILD_EXAMPLES "Build examples" ON)
if (${BUILD_EXAMPLES})
	add_subdirectory(examples)
endif ()
