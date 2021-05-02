cmake_minimum_required(VERSION 2.8.3)
project(esaul_software)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf
  sensor_msgs
  )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    roscpp
    gazebo_ros
)



