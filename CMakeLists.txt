cmake_minimum_required(VERSION 2.8.3)
project(ros-wrappers)

# any dependencies here
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roslaunch
  nodelet
  behaviors)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   # DIRECTORY msg
#    FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   service_file.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    std_msgs sensor_msgs # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros-wrappers
  CATKIN_DEPENDS roscpp
)

###################################
## BUILD ##
###################################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# add any .cpp file paths here
set(SRCS 
)

if ( NOT ("${SRCS}" STREQUAL "") )
  set(LIBS 
    ${catkin_LIBRARIES}
  )
  message(STATUS "${PROJECT_NAME} library compiling: ${SRCS}")
  add_library(ros-wrappers ${SRCS})
  target_link_libraries(ros-wrappers 
      PUBLIC
      ${LIBS})

  #update master CMakeLists.txt
  list(APPEND COMPILATION_TARGETS ros-wrappers)

  #############
  ## Install ##
  #############
  message(WARNING "ros-wrappers files found")
  install(TARGETS ros-wrappers
    # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  ## Mark cpp header files for installation
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
  )
  install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    FILES_MATCHING PATTERN "*.launch"
    )

  install(DIRECTORY plugins/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
    FILES_MATCHING PATTERN "*.xml"
    )
endif()