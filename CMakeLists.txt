cmake_minimum_required(VERSION 2.8.3)
project(gazebo_f150)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  message_generation
  gazebo_msgs
  roscpp
  rospy
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  driver_base
  std_msgs
  mav_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#message(${GAZEBO_INCLUDE_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)


add_message_files(
  FILES
  DriveCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  angles
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  driver_base
  std_msgs

  DEPENDS
    roscpp
    gazebo_ros
    gazebo
    SDF
)


add_library(gazebo_f150 src/CartDemoPlugin.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(f150_follower src/f150_follower.cc)
target_link_libraries(f150_follower ${catkin_LIBRARIES})
add_dependencies(f150_follower ${catkin_EXPORTED_TARGETS})
