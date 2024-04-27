cmake_minimum_required(VERSION 3.0)

get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId} C CXX)

set (CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )

add_executable( rrt rrt.cpp )
target_link_libraries( rrt ${OpenCV_LIBS} )