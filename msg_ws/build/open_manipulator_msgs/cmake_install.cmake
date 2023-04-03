# Install script for directory: /home/sangho/msg_ws/src/open_manipulator_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sangho/msg_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/msg" TYPE FILE FILES
    "/home/sangho/msg_ws/src/open_manipulator_msgs/msg/JointPosition.msg"
    "/home/sangho/msg_ws/src/open_manipulator_msgs/msg/KinematicsPose.msg"
    "/home/sangho/msg_ws/src/open_manipulator_msgs/msg/OpenManipulatorState.msg"
    "/home/sangho/msg_ws/src/open_manipulator_msgs/msg/Coordinate.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/srv" TYPE FILE FILES
    "/home/sangho/msg_ws/src/open_manipulator_msgs/srv/GetJointPosition.srv"
    "/home/sangho/msg_ws/src/open_manipulator_msgs/srv/GetKinematicsPose.srv"
    "/home/sangho/msg_ws/src/open_manipulator_msgs/srv/SetJointPosition.srv"
    "/home/sangho/msg_ws/src/open_manipulator_msgs/srv/SetKinematicsPose.srv"
    "/home/sangho/msg_ws/src/open_manipulator_msgs/srv/SetDrawingTrajectory.srv"
    "/home/sangho/msg_ws/src/open_manipulator_msgs/srv/SetActuatorState.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/cmake" TYPE FILE FILES "/home/sangho/msg_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/sangho/msg_ws/devel/include/open_manipulator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/sangho/msg_ws/devel/share/roseus/ros/open_manipulator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/sangho/msg_ws/devel/share/common-lisp/ros/open_manipulator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/sangho/msg_ws/devel/share/gennodejs/ros/open_manipulator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/sangho/msg_ws/devel/lib/python3/dist-packages/open_manipulator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/sangho/msg_ws/devel/lib/python3/dist-packages/open_manipulator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sangho/msg_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/cmake" TYPE FILE FILES "/home/sangho/msg_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs/cmake" TYPE FILE FILES
    "/home/sangho/msg_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgsConfig.cmake"
    "/home/sangho/msg_ws/build/open_manipulator_msgs/catkin_generated/installspace/open_manipulator_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_msgs" TYPE FILE FILES "/home/sangho/msg_ws/src/open_manipulator_msgs/package.xml")
endif()

