cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(concave_hull_2d)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (concave_hull_2d concave_hull_2d.cpp)
target_link_libraries (concave_hull_2d ${PCL_LIBRARIES})
