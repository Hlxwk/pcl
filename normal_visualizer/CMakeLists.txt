cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal_visualizer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (normal_visualizer normal_visualizer.cpp)
target_link_libraries (normal_visualizer ${PCL_LIBRARIES})
