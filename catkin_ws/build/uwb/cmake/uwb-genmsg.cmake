# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "uwb: 4 messages, 0 services")

set(MSG_I_FLAGS "-Iuwb:/data/SP1/catkin_ws/src/uwb/msg;-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(uwb_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg" NAME_WE)
add_custom_target(_uwb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uwb" "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg" "std_msgs/Header"
)

get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg" NAME_WE)
add_custom_target(_uwb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uwb" "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg" "std_msgs/Header"
)

get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg" NAME_WE)
add_custom_target(_uwb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uwb" "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg" "std_msgs/Header"
)

get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg" NAME_WE)
add_custom_target(_uwb_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uwb" "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb
)
_generate_msg_cpp(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb
)
_generate_msg_cpp(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb
)
_generate_msg_cpp(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb
)

### Generating Services

### Generating Module File
_generate_module_cpp(uwb
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(uwb_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(uwb_generate_messages uwb_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg" NAME_WE)
add_dependencies(uwb_generate_messages_cpp _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg" NAME_WE)
add_dependencies(uwb_generate_messages_cpp _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg" NAME_WE)
add_dependencies(uwb_generate_messages_cpp _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg" NAME_WE)
add_dependencies(uwb_generate_messages_cpp _uwb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uwb_gencpp)
add_dependencies(uwb_gencpp uwb_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uwb_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb
)
_generate_msg_eus(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb
)
_generate_msg_eus(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb
)
_generate_msg_eus(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb
)

### Generating Services

### Generating Module File
_generate_module_eus(uwb
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(uwb_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(uwb_generate_messages uwb_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg" NAME_WE)
add_dependencies(uwb_generate_messages_eus _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg" NAME_WE)
add_dependencies(uwb_generate_messages_eus _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg" NAME_WE)
add_dependencies(uwb_generate_messages_eus _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg" NAME_WE)
add_dependencies(uwb_generate_messages_eus _uwb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uwb_geneus)
add_dependencies(uwb_geneus uwb_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uwb_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb
)
_generate_msg_lisp(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb
)
_generate_msg_lisp(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb
)
_generate_msg_lisp(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb
)

### Generating Services

### Generating Module File
_generate_module_lisp(uwb
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(uwb_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(uwb_generate_messages uwb_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg" NAME_WE)
add_dependencies(uwb_generate_messages_lisp _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg" NAME_WE)
add_dependencies(uwb_generate_messages_lisp _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg" NAME_WE)
add_dependencies(uwb_generate_messages_lisp _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg" NAME_WE)
add_dependencies(uwb_generate_messages_lisp _uwb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uwb_genlisp)
add_dependencies(uwb_genlisp uwb_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uwb_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb
)
_generate_msg_py(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb
)
_generate_msg_py(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb
)
_generate_msg_py(uwb
  "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/jade/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb
)

### Generating Services

### Generating Module File
_generate_module_py(uwb
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(uwb_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(uwb_generate_messages uwb_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRangeRaw.msg" NAME_WE)
add_dependencies(uwb_generate_messages_py _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRange.msg" NAME_WE)
add_dependencies(uwb_generate_messages_py _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBMultiRange.msg" NAME_WE)
add_dependencies(uwb_generate_messages_py _uwb_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb/msg/UWBRangeStats.msg" NAME_WE)
add_dependencies(uwb_generate_messages_py _uwb_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uwb_genpy)
add_dependencies(uwb_genpy uwb_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uwb_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(uwb_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(uwb_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(uwb_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(uwb_generate_messages_py std_msgs_generate_messages_py)
