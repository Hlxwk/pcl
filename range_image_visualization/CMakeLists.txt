cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(range_image_visualization)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (range_image_visualization range_image_visualization.cpp)
target_link_libraries (range_image_visualization ${PCL_LIBRARIES})
