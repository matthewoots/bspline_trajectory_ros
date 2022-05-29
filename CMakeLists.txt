cmake_minimum_required(VERSION 3.0.2)
project(bs_trajectory_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

## Find catkin macros and libraries
find_package(catkin 
  COMPONENTS
    roscpp
    common_msgs
    message_generation
    pcl_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS  
    roscpp 
    common_msgs
    pcl_ros

  DEPENDS
    Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
    bs-traj-server/libbspline/cpp/include
)

add_subdirectory(bs-traj-server)

add_executable(${PROJECT_NAME}_node 
    src/main.cpp
    src/trajectory_server_ros.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  bs_traj_server
)
