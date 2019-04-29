cmake_minimum_required(VERSION 2.8.3)
project(pclvis)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES read_poing
  CATKIN_DEPENDS roscpp std_msgs pcl_msgs pcl_ros pcl_conversions
  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_IBCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include_directories(/usr/local/include/pcl-1.8/pcl)
include_directories(/usr/include/eigen3)
include_directories(/usr/include/boost)
include_directories(/usr/include/x86_64-linux-gnu)


add_executable(visuliza ./src/visuliza.cpp)
target_link_libraries(visuliza ${catkin_LIBRARIES} ${PCL_LIBRAIES})

