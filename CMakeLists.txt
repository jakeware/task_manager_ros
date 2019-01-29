cmake_minimum_required(VERSION 2.8.3)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

project(task_manager_ros)

set(CMAKE_VERBOSE_MAKEFILE FALSE)

add_definitions("-std=c++11")
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# find catkin dependencies
set(REQ_CATKIN_PKGS   
  rospy
  std_msgs
  message_generation
  )
find_package(catkin REQUIRED COMPONENTS ${REQ_CATKIN_PKGS})

catkin_python_setup()

add_message_files(FILES
  TaskInfo.msg
  TaskConfig.msg
  TaskCommand.msg
  TaskConfigList.msg
  TaskInfoList.msg
  )

add_service_files(FILES
  RegisterTask.srv
  )

generate_messages(DEPENDENCIES
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS ${REQ_CATKIN_PKGS}
  DEPENDS message_runtime
  )

catkin_install_python(PROGRAMS 
  src/task_manager_ros/task_minion_ros_node.py
  src/task_manager_ros/task_master_ros_node.py
  src/test_nodes/test_node1.py
  src/test_nodes/test_node2.py
  src/test_nodes/test_node3.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
