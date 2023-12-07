# Install script for directory: /home/kiitan/catamaran_ws/src/numpy_eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kiitan/catamaran_ws/install")
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
  include("/home/kiitan/catamaran_ws/build/numpy_eigen/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen" TYPE SHARED_LIBRARY FILES "/home/kiitan/catamaran_ws/devel/lib/libnumpy_eigen.so")
  if(EXISTS "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/kiitan/catamaran_ws/build/numpy_eigen/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen_test.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen_test.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen_test.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen_test.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen" TYPE SHARED_LIBRARY FILES "/home/kiitan/catamaran_ws/devel/lib/libnumpy_eigen_test.so")
  if(EXISTS "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen_test.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen_test.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/kiitan/catamaran_ws/install/lib/python3/dist-packages/numpy_eigen/libnumpy_eigen_test.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kiitan/catamaran_ws/src/numpy_eigen/include/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$" REGEX "/\\.svn$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kiitan/catamaran_ws/build/numpy_eigen/catkin_generated/installspace/numpy_eigen.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/numpy_eigen/cmake" TYPE FILE FILES
    "/home/kiitan/catamaran_ws/build/numpy_eigen/catkin_generated/installspace/numpy_eigenConfig.cmake"
    "/home/kiitan/catamaran_ws/build/numpy_eigen/catkin_generated/installspace/numpy_eigenConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/numpy_eigen" TYPE FILE FILES "/home/kiitan/catamaran_ws/src/numpy_eigen/package.xml")
endif()

