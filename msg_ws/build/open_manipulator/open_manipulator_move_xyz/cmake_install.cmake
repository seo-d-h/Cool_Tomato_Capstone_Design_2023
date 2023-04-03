# Install script for directory: /home/sangho/msg_ws/src/open_manipulator/open_manipulator_move_xyz

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/sangho/msg_ws/build/open_manipulator/open_manipulator_move_xyz/catkin_generated/installspace/open_manipulator_move_xyz.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_move_xyz/cmake" TYPE FILE FILES
    "/home/sangho/msg_ws/build/open_manipulator/open_manipulator_move_xyz/catkin_generated/installspace/open_manipulator_move_xyzConfig.cmake"
    "/home/sangho/msg_ws/build/open_manipulator/open_manipulator_move_xyz/catkin_generated/installspace/open_manipulator_move_xyzConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_move_xyz" TYPE FILE FILES "/home/sangho/msg_ws/src/open_manipulator/open_manipulator_move_xyz/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz" TYPE PROGRAM FILES "/home/sangho/msg_ws/build/open_manipulator/open_manipulator_move_xyz/catkin_generated/installspace/publisher.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz" TYPE PROGRAM FILES "/home/sangho/msg_ws/build/open_manipulator/open_manipulator_move_xyz/catkin_generated/installspace/subscriber.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz/open_manipulator_move_xyz" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz/open_manipulator_move_xyz")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz/open_manipulator_move_xyz"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz" TYPE EXECUTABLE FILES "/home/sangho/msg_ws/devel/lib/open_manipulator_move_xyz/open_manipulator_move_xyz")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz/open_manipulator_move_xyz" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz/open_manipulator_move_xyz")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz/open_manipulator_move_xyz"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/open_manipulator_move_xyz/open_manipulator_move_xyz")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/open_manipulator_move_xyz" TYPE DIRECTORY FILES "/home/sangho/msg_ws/src/open_manipulator/open_manipulator_move_xyz/include/open_manipulator_move_xyz/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/open_manipulator_move_xyz" TYPE DIRECTORY FILES "/home/sangho/msg_ws/src/open_manipulator/open_manipulator_move_xyz/launch")
endif()

