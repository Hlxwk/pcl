cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(vfh)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (vfh vfh.cpp)
target_link_libraries (vfh ${PCL_LIBRARIES})
