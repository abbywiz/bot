# Install script for directory: /home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules

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
  include("/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_perception_modules/msg" TYPE FILE FILES "/home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/msg/ClusterInfo.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_perception_modules/srv" TYPE FILE FILES
    "/home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/srv/ClusterInfoArray.srv"
    "/home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/srv/FilterParams.srv"
    "/home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/srv/SnapPicture.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_perception_modules/cmake" TYPE FILE FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/installspace/interbotix_perception_modules-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/saranya/interbotix_ws/devel/include/interbotix_perception_modules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/saranya/interbotix_ws/devel/share/roseus/ros/interbotix_perception_modules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/saranya/interbotix_ws/devel/share/common-lisp/ros/interbotix_perception_modules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/saranya/interbotix_ws/devel/share/gennodejs/ros/interbotix_perception_modules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/saranya/interbotix_ws/devel/lib/python3/dist-packages/interbotix_perception_modules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/saranya/interbotix_ws/devel/lib/python3/dist-packages/interbotix_perception_modules" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/saranya/interbotix_ws/devel/lib/python3/dist-packages/interbotix_perception_modules" FILES_MATCHING REGEX "/home/saranya/interbotix_ws/devel/lib/python3/dist-packages/interbotix_perception_modules/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/installspace/interbotix_perception_modules.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_perception_modules/cmake" TYPE FILE FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/installspace/interbotix_perception_modules-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_perception_modules/cmake" TYPE FILE FILES
    "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/installspace/interbotix_perception_modulesConfig.cmake"
    "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/installspace/interbotix_perception_modulesConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_perception_modules" TYPE FILE FILES "/home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/interbotix_perception_modules" TYPE PROGRAM FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/installspace/armtag_tuner_gui")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/interbotix_perception_modules" TYPE PROGRAM FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/installspace/pointcloud_tuner_gui")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/interbotix_perception_modules" TYPE PROGRAM FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/installspace/static_trans_pub")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/interbotix_perception_modules" TYPE PROGRAM FILES "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_perception_toolbox/interbotix_perception_modules/catkin_generated/installspace/picture_snapper")
endif()

