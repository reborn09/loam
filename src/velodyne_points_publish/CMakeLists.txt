cmake_minimum_required(VERSION 2.8.3)
project(velodyne_points_publish)

add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "./include"
)

add_executable(velodynePointsPublish src/velodyne_points_publish.cpp src/velodyne_points_parser.cpp src/parameterReader.cpp)

target_link_libraries(velodynePointsPublish ${catkin_LIBRARIES} ${OpenCV_LIBS})
