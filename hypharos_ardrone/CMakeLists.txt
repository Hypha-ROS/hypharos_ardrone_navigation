cmake_minimum_required(VERSION 2.8.3)
project(hypharos_ardrone)

# For Kinetic support
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs tum_ardrone moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)
find_package(Eigen REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hypharos_ardrone
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_executable(conversion src/conversion.cpp)
target_link_libraries(conversion ${catkin_LIBRARIES})

add_executable(ardrone_controller src/ardrone_controller.cpp)
target_link_libraries(ardrone_controller ${catkin_LIBRARIES})
