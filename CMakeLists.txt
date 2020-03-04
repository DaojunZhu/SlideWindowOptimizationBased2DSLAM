cmake_minimum_required(VERSION 2.8.3)
project(slam_2d_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
  slam_data_sim
)

find_package(Ceres REQUIRED)
find_package(Eigen3)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slam_2d_sim
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


add_library(slam_2d_lib
  src/estimator.cpp
  src/feature_manager.cpp
  src/parameters.cpp
  src/utility.cpp
  src/factor/marginalization_factor.cpp
  src/visualization.cpp
)

target_link_libraries(slam_2d_lib ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(slam_2d src/rosNodeTest.cpp)
target_link_libraries(slam_2d slam_2d_lib) 