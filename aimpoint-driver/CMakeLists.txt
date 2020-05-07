# Setup
cmake_minimum_required(VERSION 3.2)
project(aimpoint-driver)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package (Threads REQUIRED)

# Add odvd messages
include_directories(SYSTEM ${CMAKE_BINARY_DIR})

# Main program stuff
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/main)
target_link_libraries(${PROJECT_NAME} Threads::Threads)