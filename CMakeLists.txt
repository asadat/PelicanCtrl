cmake_minimum_required(VERSION 2.8.3)

project(pelican_ctrl)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall") # For G++ <4.7; also appears to be supported in G++ 4.7+ as alias to c++11

include(ExternalProject)
ExternalProject_Add(libtoon
  URL ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/TooN-2.2.tar.gz
  PREFIX ${CATKIN_DEVEL_PREFIX}
  CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND make
#  TEST_COMMAND make test
  INSTALL_COMMAND make install
  BUILD_IN_SOURCE 1
)

#set(UEYE_INCLUDE_PATH /usr/include)
#set(UEYE_LIBRARY_PATH /usr/lib)
#set(UEYE_LIBRARY ueye_api)
#set(UEYECAM_NODELET_NAME ueye_cam_nodelet)

set(POSITIONVIS_LIBRARIES GL GLU glut)

set(PELICANCTRL_SOURCES
  src/PelicanPosCtrl.cpp
  include/PelicanPosCtrl.h
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
  nav_msgs
  )

#generate_dynamic_reconfigure_options(
#  cfg/UEyeCam.cfg
#  )

add_service_files(FILES
   gotoPos.srv  
   gotoPos_body.srv
   hover.srv
   gotoPosGPS.srv
)

generate_messages()


catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include
  LIBRARIES 
  CATKIN_DEPENDS roscpp  rospy  geometry_msgs  std_msgs  nav_msgs  dynamic_reconfigure  tf  asctec_hl_comm  control_toolbox message_runtime)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
  )

add_executable(PelicanCtrl ${PELICANCTRL_SOURCES})
target_link_libraries(PelicanCtrl ${catkin_LIBRARIES})
add_dependencies(PelicanCtrl  ${PROJECT_NAME}_gencpp libtoon)

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

