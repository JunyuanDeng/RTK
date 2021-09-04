cmake_minimum_required(VERSION 3.10)

project(RTK)

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})

#add_executable(transform transform.cpp)

add_executable(RTK main.cpp pointcloud.cpp transform.cpp readFileName.cpp)
target_link_libraries(RTK ${PCL_LIBRARIES})