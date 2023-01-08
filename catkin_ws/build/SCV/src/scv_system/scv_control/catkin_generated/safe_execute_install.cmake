execute_process(COMMAND "/home/dong/myroom/catkin_ws/build/SCV/src/scv_system/scv_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/dong/myroom/catkin_ws/build/SCV/src/scv_system/scv_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
