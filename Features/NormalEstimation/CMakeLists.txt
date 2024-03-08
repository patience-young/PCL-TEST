cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(NormalEstimation)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (estimate_all_points estimate_all_points.cpp)
target_link_libraries (estimate_all_points ${PCL_LIBRARIES})

add_executable (estimate_indices_points estimate_indices_points.cpp)
target_link_libraries (estimate_indices_points ${PCL_LIBRARIES})

add_executable (estimate_subset_points estimate_subset_points.cpp)
target_link_libraries (estimate_subset_points ${PCL_LIBRARIES})