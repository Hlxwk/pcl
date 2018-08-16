cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(statistical_removal)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (statistical_removal statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})
