# Setup
cmake_minimum_required(VERSION 3.2)
project(cone-detection)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package (Threads REQUIRED)
find_package( OpenCV REQUIRED )

# Add odvd messages
include_directories(SYSTEM ${CMAKE_BINARY_DIR})

# Main program stuff
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
${CMAKE_CURRENT_SOURCE_DIR}/main)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} )

# # Test program stuff
# enable_testing()
# add_executable(${PROJECT_NAME}-test tests.cpp path_finder.cpp
# 	${CMAKE_CURRENT_SOURCE_DIR}/tests.cpp)
# add_test(NAME ${PROJECT_NAME}-test COMMAND ${PROJECT_NAME}-test)