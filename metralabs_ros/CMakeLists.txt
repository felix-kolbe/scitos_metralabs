cmake_minimum_required(VERSION 2.8.3)
project(metralabs_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rospy
  
  tf
  urdf
  actionlib
  dynamic_reconfigure
  
  metralabs_msgs
  diagnostic_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  control_msgs
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## include dynamic reconfigure script
generate_dynamic_reconfigure_options(cfg/ScitosG5.cfg)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES metralabs_ros
  CATKIN_DEPENDS
  dynamic_reconfigure
  metralabs_msgs
  diagnostic_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  control_msgs
  trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# flags DM_LINUX and D_REENTRANT are needed/suggested by MetraLabs
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_REENTRANT -DM_LINUX -Wall")

# set up MetraLabs libs
set (ML_ROOT_DIR /opt/MetraLabs)
link_directories("${ML_ROOT_DIR}/lib")


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ML_ROOT_DIR}/MetraLabsBase/include
  ${ML_ROOT_DIR}/MLRobotic/include
)

## Declare a cpp library
# add_library(metralabs_ros
#   src/${PROJECT_NAME}/metralabs_ros.cpp
# )

## Declare a cpp executable
# add_executable(metralabs_ros_node src/metralabs_ros_node.cpp)
add_executable(ScitosServer
  src/ScitosServer.cpp
  src/ScitosBase.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(metralabs_ros_node metralabs_ros_generate_messages_cpp)
add_dependencies(ScitosServer
  ${PROJECT_NAME}_gen_cfg
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
# target_link_libraries(metralabs_ros_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(ScitosServer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  MetraLabsBase
  MLRoboticBase
  MLRoboticSystem
  MLRoboticCAN
  MLRoboticRobot
  MLRoboticSCITOS
  MLRoboticHardware
  MLRoboticHardwareSCHUNK
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS metralabs_ros metralabs_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_metralabs_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
