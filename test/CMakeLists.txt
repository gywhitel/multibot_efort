cmake_minimum_required(VERSION 3.1)

project(filter)
set(TARGET1 preprocess)
set(TARGET2 path_generation)
set(TARGET3 kinematics_test)
set(TARGET4 make_pointcloud)

set(SRC pointcloud_preprocess.cpp)

add_executable(${TARGET1} ${SRC})
add_executable(${TARGET2} path_generate.cpp
                            arithmatic.cpp)
add_executable(${TARGET3}   kinematicTest.cpp
                            ../src/kinematic.cpp)
add_executable(${TARGET4} generatePointCloud.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(/usr/include/eigen3/)
include_directories(/usr/include/pcl-1.10)
include_directories(${PROJECT_SOURCE_DIR}/../include)
include_directories(/opt/ros/noetic/include)

find_package(PCL REQUIRED QUIET)    #do not use COMPONENTS here, otherwise only components would be imported
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${TARGET1} ${PCL_LIBRARIES})
target_link_libraries(${TARGET4} ${PCL_LIBRARIES})
target_link_libraries(${TARGET2} ${PCL_LIBRARIES})