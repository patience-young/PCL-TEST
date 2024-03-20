cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(Registration)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iterative_closest_point src/iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

add_executable (pairwise_incremental_registration src/pairwise_incremental_registration.cpp)
target_link_libraries (pairwise_incremental_registration ${PCL_LIBRARIES})

add_executable (interactive_icp src/interactive_icp.cpp)
target_link_libraries (interactive_icp ${PCL_LIBRARIES})

add_executable(normal_distributions_transform src/normal_distributions_transform.cpp)
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})