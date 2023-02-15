# %Tag(FULLTEXT)%


cmake_minimum_required(VERSION 2.8.3)
project(gps_transform)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs global_planner move_base nav_core)

catkin_package(CATKIN_DEPENDS roscpp std_msgs global_planner move_base nav_core)

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
 
 add_service_files(
  FILES
  servicemsg.srv
)


# %EndTag(FULLTEXT)%
