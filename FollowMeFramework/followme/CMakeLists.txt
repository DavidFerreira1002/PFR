cmake_minimum_required(VERSION 3.0.2)
project(followme)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  sensor_msgs
  rospy
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include/followme
  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# create the transform handler library
add_library(transform_handler src/TransformHandler.cpp)
target_link_libraries(transform_handler ${catkin_LIBRARIES})

# create the gesture handler library
add_library(gesture_handler src/GestureHandler.cpp)
target_link_libraries(gesture_handler ${catkin_LIBRARIES})

add_library(general_control_handler src/GeneralControlHandler.cpp)
target_link_libraries(general_control_handler ${catkin_LIBRARIES})

add_library(search_nearby_handler src/SearchNearbyHandler.cpp)
target_link_libraries(search_nearby_handler ${catkin_LIBRARIES})

# move base navigation controller
add_executable(followme_node src/followme_node.cpp)
target_link_libraries(followme_node transform_handler gesture_handler general_control_handler search_nearby_handler  ${catkin_LIBRARIES})

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
target_link_libraries(obstacle_avoidance_node ${catkin_LIBRARIES})


# python scripts
catkin_install_python(PROGRAMS
  scripts/track_person_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)