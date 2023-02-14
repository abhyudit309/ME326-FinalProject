# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "me326_locobot_example: 0 messages, 1 services")

set(MSG_I_FLAGS "-Iroscpp:/opt/ros/noetic/share/roscpp/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Ivisualization_msgs:/opt/ros/noetic/share/visualization_msgs/cmake/../msg;-Itf:/opt/ros/noetic/share/tf/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(me326_locobot_example_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv" NAME_WE)
add_custom_target(_me326_locobot_example_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "me326_locobot_example" "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv" "geometry_msgs/Point:std_msgs/Header:geometry_msgs/PointStamped"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(me326_locobot_example
  "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/me326_locobot_example
)

### Generating Module File
_generate_module_cpp(me326_locobot_example
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/me326_locobot_example
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(me326_locobot_example_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(me326_locobot_example_generate_messages me326_locobot_example_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv" NAME_WE)
add_dependencies(me326_locobot_example_generate_messages_cpp _me326_locobot_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(me326_locobot_example_gencpp)
add_dependencies(me326_locobot_example_gencpp me326_locobot_example_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS me326_locobot_example_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(me326_locobot_example
  "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/me326_locobot_example
)

### Generating Module File
_generate_module_eus(me326_locobot_example
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/me326_locobot_example
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(me326_locobot_example_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(me326_locobot_example_generate_messages me326_locobot_example_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv" NAME_WE)
add_dependencies(me326_locobot_example_generate_messages_eus _me326_locobot_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(me326_locobot_example_geneus)
add_dependencies(me326_locobot_example_geneus me326_locobot_example_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS me326_locobot_example_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(me326_locobot_example
  "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/me326_locobot_example
)

### Generating Module File
_generate_module_lisp(me326_locobot_example
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/me326_locobot_example
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(me326_locobot_example_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(me326_locobot_example_generate_messages me326_locobot_example_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv" NAME_WE)
add_dependencies(me326_locobot_example_generate_messages_lisp _me326_locobot_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(me326_locobot_example_genlisp)
add_dependencies(me326_locobot_example_genlisp me326_locobot_example_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS me326_locobot_example_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(me326_locobot_example
  "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/me326_locobot_example
)

### Generating Module File
_generate_module_nodejs(me326_locobot_example
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/me326_locobot_example
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(me326_locobot_example_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(me326_locobot_example_generate_messages me326_locobot_example_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv" NAME_WE)
add_dependencies(me326_locobot_example_generate_messages_nodejs _me326_locobot_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(me326_locobot_example_gennodejs)
add_dependencies(me326_locobot_example_gennodejs me326_locobot_example_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS me326_locobot_example_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(me326_locobot_example
  "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PointStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/me326_locobot_example
)

### Generating Module File
_generate_module_py(me326_locobot_example
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/me326_locobot_example
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(me326_locobot_example_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(me326_locobot_example_generate_messages me326_locobot_example_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/connor/ME326-FinalProject/src/me326_locobot_example/srv/PixtoPoint.srv" NAME_WE)
add_dependencies(me326_locobot_example_generate_messages_py _me326_locobot_example_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(me326_locobot_example_genpy)
add_dependencies(me326_locobot_example_genpy me326_locobot_example_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS me326_locobot_example_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/me326_locobot_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/me326_locobot_example
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET roscpp_generate_messages_cpp)
  add_dependencies(me326_locobot_example_generate_messages_cpp roscpp_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(me326_locobot_example_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(me326_locobot_example_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET visualization_msgs_generate_messages_cpp)
  add_dependencies(me326_locobot_example_generate_messages_cpp visualization_msgs_generate_messages_cpp)
endif()
if(TARGET tf_generate_messages_cpp)
  add_dependencies(me326_locobot_example_generate_messages_cpp tf_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(me326_locobot_example_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(me326_locobot_example_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/me326_locobot_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/me326_locobot_example
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET roscpp_generate_messages_eus)
  add_dependencies(me326_locobot_example_generate_messages_eus roscpp_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(me326_locobot_example_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(me326_locobot_example_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET visualization_msgs_generate_messages_eus)
  add_dependencies(me326_locobot_example_generate_messages_eus visualization_msgs_generate_messages_eus)
endif()
if(TARGET tf_generate_messages_eus)
  add_dependencies(me326_locobot_example_generate_messages_eus tf_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(me326_locobot_example_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(me326_locobot_example_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/me326_locobot_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/me326_locobot_example
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET roscpp_generate_messages_lisp)
  add_dependencies(me326_locobot_example_generate_messages_lisp roscpp_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(me326_locobot_example_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(me326_locobot_example_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET visualization_msgs_generate_messages_lisp)
  add_dependencies(me326_locobot_example_generate_messages_lisp visualization_msgs_generate_messages_lisp)
endif()
if(TARGET tf_generate_messages_lisp)
  add_dependencies(me326_locobot_example_generate_messages_lisp tf_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(me326_locobot_example_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(me326_locobot_example_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/me326_locobot_example)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/me326_locobot_example
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET roscpp_generate_messages_nodejs)
  add_dependencies(me326_locobot_example_generate_messages_nodejs roscpp_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(me326_locobot_example_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(me326_locobot_example_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET visualization_msgs_generate_messages_nodejs)
  add_dependencies(me326_locobot_example_generate_messages_nodejs visualization_msgs_generate_messages_nodejs)
endif()
if(TARGET tf_generate_messages_nodejs)
  add_dependencies(me326_locobot_example_generate_messages_nodejs tf_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(me326_locobot_example_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(me326_locobot_example_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/me326_locobot_example)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/me326_locobot_example\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/me326_locobot_example
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/me326_locobot_example")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/me326_locobot_example
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET roscpp_generate_messages_py)
  add_dependencies(me326_locobot_example_generate_messages_py roscpp_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(me326_locobot_example_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(me326_locobot_example_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET visualization_msgs_generate_messages_py)
  add_dependencies(me326_locobot_example_generate_messages_py visualization_msgs_generate_messages_py)
endif()
if(TARGET tf_generate_messages_py)
  add_dependencies(me326_locobot_example_generate_messages_py tf_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(me326_locobot_example_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(me326_locobot_example_generate_messages_py nav_msgs_generate_messages_py)
endif()
