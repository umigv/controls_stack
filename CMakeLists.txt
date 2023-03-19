# %Tag(FULLTEXT)%


cmake_minimum_required(VERSION 2.8.3)
project(gps_transform)

find_package(catkin REQUIRED COMPONENTS 
  roscpp std_msgs 
  global_planner 
  move_base 
  nav_core 
  # Added cartographer_ros_msgs and geometry_msgs
 # cartographer_ros_msgs 
  geometry_msgs
  message_generation
  tf2_msgs
  std_srvs
  )

#moved service files before catkin package
add_service_files(
  FILES
  servicemsg.srv
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs global_planner move_base nav_core geometry_msgs message_runtime tf2_msgs std_srvs) 

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/gps_listener.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

# %EndTag(FULLTEXT)%
