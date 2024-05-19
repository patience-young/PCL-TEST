cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(Segmentation)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_segmentation src/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

add_executable (cluster_extraction src/cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

add_executable (model_outlier_removal src/model_outlier_removal.cpp)
target_link_libraries (model_outlier_removal ${PCL_LIBRARIES})