cmake_minimum_required(VERSION 2.8.3)
project(depth_image_processing)

## CLION - specific options
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../../build/{PROJECT_NAME}")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "/home/vprooks/catkin_indigo_ws/build/{PROJECT_NAME}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_generation
  message_runtime
  sensor_msgs
  pcl_conversions
  image_transport
  message_filters
  depth_image_proc
  image_geometry
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## PCL
set(PCL_DIR "/usr/local/share/pcl-1.8/PCLConfig.cmake")
find_package(PCL 1.8 REQUIRED)
if (NOT PCL_FOUND)
    MESSAGE(FATAL_ERROR "PCL not found.\n")
endif (NOT PCL_FOUND)
link_directories(${PCL_LIBRARY_DIRS})
include_directories(BEFORE ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

## OpenCV
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

## ZeroMQ
# - Try to find ZMQ
# Once done this will define
# ZMQ_FOUND - System has ZMQ
# ZMQ_INCLUDE_DIRS - The ZMQ include directories
# ZMQ_LIBRARIES - The libraries needed to use ZMQ
# ZMQ_DEFINITIONS - Compiler switches required for using ZMQ

# find_library(ZMQ_LIB libzmq)
find_path ( ZMQ_INCLUDE_DIR zmq.h )
find_library ( ZMQ_LIBRARY NAMES zmq )
set ( ZMQ_LIBRARIES ${ZMQ_LIBRARY} )
set ( ZMQ_INCLUDE_DIRS ${ZMQ_INCLUDE_DIR} )
include ( FindPackageHandleStandardArgs )
# handle the QUIETLY and REQUIRED arguments and set ZMQ_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args ( ZMQ DEFAULT_MSG ZMQ_LIBRARY ZMQ_INCLUDE_DIR )

## RapidJSON
find_package(RapidJSON)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
 INCLUDE_DIRS include
#  LIBRARIES depth_image_processing
 CATKIN_DEPENDS roscpp std_msgs cv_bridge message_generation message_runtime sensor_msgs std_msgs image_transport message_filters depth_image_proc image_geometry
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZMQ_INCLUE_DIRS}
  ${RapidJSON_INCLUDE_DIRS}
)

# add_library(server src/${PROJECT_NAME}/server.cpp)
# target_link_libraries(server
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# add_library(acceptor src/${PROJECT_NAME}/acceptor.cpp)
# target_link_libraries(acceptor
#   server
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# add_library(connection src/${PROJECT_NAME}/connection.cpp)
# target_link_libraries(connection
#   acceptor
#   server
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# add_library(point_cloud_streamer src/${PROJECT_NAME}/point_cloud_streamer.cpp)
# target_link_libraries(point_cloud_streamer
#   server
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

add_library(json_interop src/${PROJECT_NAME}/json_interop.cpp)
target_link_libraries(json_interop
  ${RapidJSON_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(utilities src/${PROJECT_NAME}/utilities.cpp)
target_link_libraries(utilities
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(processing src/${PROJECT_NAME}/processing.cpp)
target_link_libraries(processing
  utilities
  json_interop
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZMQ_LIBRARIES}
)

add_executable(node src/node.cpp)
target_link_libraries(node
  processing
  utilities
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)




## Declare a cpp library
# add_library(depth_image_processing
#   src/${PROJECT_NAME}/depth_image_processing.cpp
# )

## Declare a cpp executable
# add_executable(depth_image_processing_node src/depth_image_processing_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(depth_image_processing_node depth_image_processing_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(node
   #libdhd.a
   #libdrd.a
   #libdhd.so.3.5.3
   #libdrd.so.3.5.3
   ${catkin_LIBRARIES}
   /home/naveed/haptics/sdk-3.5.2/lib/release/lin-x86_64-gcc/libdhd.a
   /home/naveed/haptics/sdk-3.5.2/lib/release/lin-x86_64-gcc/libdrd.a
   /home/naveed/haptics/sdk-3.5.2/lib/release/lin-x86_64-gcc/libdhd.so.3.5.2
   /home/naveed/haptics/sdk-3.5.2/lib/release/lin-x86_64-gcc/libdrd.so.3.5.2
   -lusb-1.0
   -lrt
   -ldl
   -lpthread
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
# install(TARGETS depth_image_processing depth_image_processing_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_depth_image_processing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
