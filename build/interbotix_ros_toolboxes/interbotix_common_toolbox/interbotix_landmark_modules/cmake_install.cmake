# Install script for directory: /home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/saranya/interbotix_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules/catkin_generated/installspace/interbotix_landmark_modules.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_landmark_modules/cmake" TYPE FILE FILES
    "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules/catkin_generated/installspace/interbotix_landmark_modulesConfig.cmake"
    "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules/catkin_generated/installspace/interbotix_landmark_modulesConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_landmark_modules" TYPE FILE FILES "/home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/interbotix_landmark_modules" TYPE PROGRAM FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules/catkin_generated/installspace/landmark_finder")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/interbotix_landmark_modules" TYPE PROGRAM FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules/catkin_generated/installspace/tf_map_to_landmark")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/interbotix_landmark_modules" TYPE PROGRAM FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_landmark_modules/catkin_generated/installspace/landmark_manager")
endif()

