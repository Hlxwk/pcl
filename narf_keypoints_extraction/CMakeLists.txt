cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(narf_keypoints_extraction)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (narf_keypoints_extraction narf_keypoints_extraction.cpp)
target_link_libraries (narf_keypoints_extraction ${PCL_LIBRARIES})
