cmake_minimum_required(VERSION 3.5.1)
project(deeporange13_control)

find_package(catkin REQUIRED COMPONENTS 
  roslaunch
  can_dbc_parser
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  deeporange13_msgs
  novatel_oem7_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
  deeporange13_msgs
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE
    include/
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/nodelet.cpp
  src/DeepOrangeDbwCan.cpp
  src/VehicleModel.cpp
  src/DbwSupervisor.cpp
  src/RosHealthMonitor.cpp
  src/StackVelPub.cpp
  src/DataLogger.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# add_executable(publisherC src/publisher.cpp)
# target_link_libraries(publisherC ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_dbwnode src/node.cpp

)
# add more executables ^^ ...

target_link_libraries(${PROJECT_NAME}_dbwnode ${PROJECT_NAME}
)
# add more ^^ ...

# roslaunch_add_file_check(launch/control.launch IGNORE_UNSET_ARGS)
# roslaunch_add_file_check(launch/teleop.launch)

install(DIRECTORY config launch src dbc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
