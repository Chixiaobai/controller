# Install script for directory: /home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/install")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rosidl_interfaces" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/controller/controller" TYPE DIRECTORY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_c/controller/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/environment" TYPE FILE FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/environment" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/library_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/libcontroller__rosidl_generator_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_c.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/controller/controller" TYPE DIRECTORY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_typesupport_fastrtps_c/controller/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/libcontroller__rosidl_typesupport_fastrtps_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_c.so"
         OLD_RPATH "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/controller/controller" TYPE DIRECTORY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_cpp/controller/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/controller/controller" TYPE DIRECTORY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_typesupport_fastrtps_cpp/controller/" REGEX "/[^/]*\\.cpp$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/libcontroller__rosidl_typesupport_fastrtps_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_fastrtps_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/controller/controller" TYPE DIRECTORY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_typesupport_introspection_c/controller/" REGEX "/[^/]*\\.h$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/libcontroller__rosidl_typesupport_introspection_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_c.so"
         OLD_RPATH "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_c.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_c.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/libcontroller__rosidl_typesupport_c.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_c.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_c.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_c.so"
         OLD_RPATH "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_c.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/controller/controller" TYPE DIRECTORY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_typesupport_introspection_cpp/controller/" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/libcontroller__rosidl_typesupport_introspection_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_introspection_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_cpp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_cpp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/libcontroller__rosidl_typesupport_cpp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_cpp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_cpp.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_cpp.so"
         OLD_RPATH "/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_typesupport_cpp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/environment" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/environment" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller-0.0.0-py3.10.egg-info" TYPE DIRECTORY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_python/controller/controller.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller" TYPE DIRECTORY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_py/controller/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/install/local/lib/python3.10/dist-packages/controller"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_py/controller/controller_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_py/controller:/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_fastrtps_c.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_py/controller/controller_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_py/controller:/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_introspection_c.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_py/controller/controller_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so"
         OLD_RPATH "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_py/controller:/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/controller/controller_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_py.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_py.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_generator_py/controller/libcontroller__rosidl_generator_py.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_py.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_py.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_py.so"
         OLD_RPATH "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller:/opt/ros/humble/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcontroller__rosidl_generator_py.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/CartesianParams.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/ErrorMessage.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/JointAngle.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/JointParams.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/LinearMoveParams.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/Localization.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/MobileBaseCmd.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/MobileBaseState.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/MoveMessage.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/OdometryMessage.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/RealTimeBodyJoints.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/RealTimeBodyTcpCartesian.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/TcpOffsetParams.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/TcpPayloadParams.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/msg/TcpPoseParams.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/EnableRealtimeCmd.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/ErrorClear.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/ForwardKinematics.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetCartesianMechanicalRotationMaxAcc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetCartesianMechanicalRotationMaxVel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetCartesianMechanicalTranslationMaxAcc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetCartesianMechanicalTranslationMaxVel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetCartesianRotationMaxAcc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetCartesianRotationMaxVel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetCartesianTranslationMaxAcc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetCartesianTranslationMaxVel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetJointMaxAcc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetJointMaxVel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetJointMechanicalLimit.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetJointMechanicalMaxAcc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetJointMechanicalMaxVel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetJointSoftLimit.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetTcpOffset.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetTcpPayload.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/GetVersion.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/InverseKinematics.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/LinearMove.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/MultiJointMove.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SetCartesianRotationMaxAcc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SetCartesianRotationMaxVel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SetCartesianTranslationMaxAcc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SetCartesianTranslationMaxVel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SetJointMaxAcc.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SetJointMaxVel.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SetJointSoftLimit.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SetTcpOffset.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SetTcpPayload.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_adapter/controller/srv/SingleJointMove.idl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/CartesianParams.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/ErrorMessage.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/JointAngle.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/JointParams.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/LinearMoveParams.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/Localization.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/MobileBaseCmd.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/MobileBaseState.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/MoveMessage.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/OdometryMessage.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/RealTimeBodyJoints.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/RealTimeBodyTcpCartesian.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/TcpOffsetParams.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/TcpPayloadParams.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/msg" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/msg/TcpPoseParams.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/EnableRealtimeCmd.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/EnableRealtimeCmd_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/EnableRealtimeCmd_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/ErrorClear.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/ErrorClear_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/ErrorClear_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/ForwardKinematics.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/ForwardKinematics_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/ForwardKinematics_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetCartesianMechanicalRotationMaxAcc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianMechanicalRotationMaxAcc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianMechanicalRotationMaxAcc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetCartesianMechanicalRotationMaxVel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianMechanicalRotationMaxVel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianMechanicalRotationMaxVel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetCartesianMechanicalTranslationMaxAcc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianMechanicalTranslationMaxAcc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianMechanicalTranslationMaxAcc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetCartesianMechanicalTranslationMaxVel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianMechanicalTranslationMaxVel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianMechanicalTranslationMaxVel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetCartesianRotationMaxAcc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianRotationMaxAcc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianRotationMaxAcc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetCartesianRotationMaxVel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianRotationMaxVel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianRotationMaxVel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetCartesianTranslationMaxAcc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianTranslationMaxAcc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianTranslationMaxAcc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetCartesianTranslationMaxVel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianTranslationMaxVel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetCartesianTranslationMaxVel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetJointMaxAcc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMaxAcc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMaxAcc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetJointMaxVel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMaxVel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMaxVel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetJointMechanicalLimit.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMechanicalLimit_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMechanicalLimit_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetJointMechanicalMaxAcc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMechanicalMaxAcc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMechanicalMaxAcc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetJointMechanicalMaxVel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMechanicalMaxVel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointMechanicalMaxVel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetJointSoftLimit.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointSoftLimit_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetJointSoftLimit_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetTcpOffset.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetTcpOffset_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetTcpOffset_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetTcpPayload.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetTcpPayload_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetTcpPayload_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/GetVersion.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetVersion_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/GetVersion_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/InverseKinematics.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/InverseKinematics_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/InverseKinematics_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/LinearMove.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/LinearMove_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/LinearMove_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/MultiJointMove.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/MultiJointMove_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/MultiJointMove_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SetCartesianRotationMaxAcc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetCartesianRotationMaxAcc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetCartesianRotationMaxAcc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SetCartesianRotationMaxVel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetCartesianRotationMaxVel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetCartesianRotationMaxVel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SetCartesianTranslationMaxAcc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetCartesianTranslationMaxAcc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetCartesianTranslationMaxAcc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SetCartesianTranslationMaxVel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetCartesianTranslationMaxVel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetCartesianTranslationMaxVel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SetJointMaxAcc.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetJointMaxAcc_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetJointMaxAcc_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SetJointMaxVel.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetJointMaxVel_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetJointMaxVel_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SetJointSoftLimit.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetJointSoftLimit_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetJointSoftLimit_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SetTcpOffset.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetTcpOffset_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetTcpOffset_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SetTcpPayload.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetTcpPayload_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SetTcpPayload_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/srv/SingleJointMove.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SingleJointMove_Request.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/srv" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/srv/SingleJointMove_Response.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/environment" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/environment" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_index/share/ament_index/resource_index/packages/controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_cExport.cmake"
         "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_generator_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_generator_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_generator_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cExport.cmake"
         "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_cppExport.cmake"
         "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_generator_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_generator_cppExport.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cppExport.cmake"
         "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_typesupport_fastrtps_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_introspection_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_introspection_cExport.cmake"
         "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_introspection_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_introspection_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_introspection_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_introspection_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_introspection_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_cExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_cExport.cmake"
         "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_cExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_cExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_cExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_cExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_cExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_introspection_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_introspection_cppExport.cmake"
         "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_introspection_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_introspection_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_introspection_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_introspection_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_introspection_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_cppExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_cppExport.cmake"
         "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_cppExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_cppExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/controller__rosidl_typesupport_cppExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_cppExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/controller__rosidl_typesupport_cppExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_pyExport.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_pyExport.cmake"
         "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_generator_pyExport.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_pyExport-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/controller/cmake/export_controller__rosidl_generator_pyExport.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_generator_pyExport.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/CMakeFiles/Export/share/controller/cmake/export_controller__rosidl_generator_pyExport-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/rosidl_cmake-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller/cmake" TYPE FILE FILES
    "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_core/controllerConfig.cmake"
    "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/ament_cmake_core/controllerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/controller" TYPE FILE FILES "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/src/controller/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/controller__py/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/lizhou/Desktop/Humanoid/H10_W/h10_sdk/H10w_controller/fastdds_ws/build/controller/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
