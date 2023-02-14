execute_process(COMMAND "/home/connor/ME326-FinalProject/build/me326_locobot_example/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/connor/ME326-FinalProject/build/me326_locobot_example/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
