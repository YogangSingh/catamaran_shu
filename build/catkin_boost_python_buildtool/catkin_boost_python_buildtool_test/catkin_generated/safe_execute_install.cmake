execute_process(COMMAND "/home/kiitan/catamaran_ws/build/catkin_boost_python_buildtool/catkin_boost_python_buildtool_test/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/kiitan/catamaran_ws/build/catkin_boost_python_buildtool/catkin_boost_python_buildtool_test/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
