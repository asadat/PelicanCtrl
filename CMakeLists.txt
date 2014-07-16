cmake_minimum_required(VERSION 2.8.3)

project(pelican_ctrl)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall") # For G++ <4.7; also appears to be supported in G++ 4.7+ as alias to c++11

#set(UEYE_INCLUDE_PATH /usr/include)
#set(UEYE_LIBRARY_PATH /usr/lib)
#set(UEYE_LIBRARY ueye_api)
#set(UEYECAM_NODELET_NAME ueye_cam_nodelet)

set(POSITIONVIS_LIBRARIES GL GLU glut)

set(PELICANCTRL_SOURCES
  src/PelicanPosCtrl.cpp
  src/LoggingService.cpp
  include/PelicanPosCtrl.h
)

set(POSITIONVIS_SOURCES
  include/PositionVis.h
  src/PositionVis.cpp
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  nav_msgs
  dynamic_reconfigure
  tf
  asctec_hl_comm
  control_toolbox
  message_generation
#  rosconsole
  )

#generate_dynamic_reconfigure_options(
#  cfg/UEyeCam.cfg
#  )

add_service_files(FILES
   gotoPos.srv  
   hover.srv
)

generate_messages()


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp  rospy  geometry_msgs  std_msgs  nav_msgs  dynamic_reconfigure  tf  asctec_hl_comm  control_toolbox message_runtime)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

  
add_executable(PositionViz ${POSITIONVIS_SOURCES})
target_link_libraries(PositionViz  ${POSITIONVIS_LIBRARIES} ${catkin_LIBRARIES})

add_executable(PelicanCtrl ${PELICANCTRL_SOURCES})
target_link_libraries(PelicanCtrl ${catkin_LIBRARIES})

add_dependencies(PelicanCtrl  ${PROJECT_NAME}_gencpp)

#install(TARGETS ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )

#install(FILES nodelet_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  )

