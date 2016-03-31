# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "uwb_dummy: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iuwb_dummy:/data/SP1/catkin_ws/src/uwb_dummy/msg;-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(uwb_dummy_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg" NAME_WE)
add_custom_target(_uwb_dummy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "uwb_dummy" "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(uwb_dummy
  "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb_dummy
)

### Generating Services

### Generating Module File
_generate_module_cpp(uwb_dummy
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb_dummy
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(uwb_dummy_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(uwb_dummy_generate_messages uwb_dummy_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg" NAME_WE)
add_dependencies(uwb_dummy_generate_messages_cpp _uwb_dummy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uwb_dummy_gencpp)
add_dependencies(uwb_dummy_gencpp uwb_dummy_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uwb_dummy_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(uwb_dummy
  "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb_dummy
)

### Generating Services

### Generating Module File
_generate_module_eus(uwb_dummy
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb_dummy
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(uwb_dummy_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(uwb_dummy_generate_messages uwb_dummy_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg" NAME_WE)
add_dependencies(uwb_dummy_generate_messages_eus _uwb_dummy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uwb_dummy_geneus)
add_dependencies(uwb_dummy_geneus uwb_dummy_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uwb_dummy_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(uwb_dummy
  "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb_dummy
)

### Generating Services

### Generating Module File
_generate_module_lisp(uwb_dummy
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb_dummy
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(uwb_dummy_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(uwb_dummy_generate_messages uwb_dummy_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg" NAME_WE)
add_dependencies(uwb_dummy_generate_messages_lisp _uwb_dummy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uwb_dummy_genlisp)
add_dependencies(uwb_dummy_genlisp uwb_dummy_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uwb_dummy_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(uwb_dummy
  "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb_dummy
)

### Generating Services

### Generating Module File
_generate_module_py(uwb_dummy
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb_dummy
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(uwb_dummy_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(uwb_dummy_generate_messages uwb_dummy_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/data/SP1/catkin_ws/src/uwb_dummy/msg/Coordinates.msg" NAME_WE)
add_dependencies(uwb_dummy_generate_messages_py _uwb_dummy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(uwb_dummy_genpy)
add_dependencies(uwb_dummy_genpy uwb_dummy_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS uwb_dummy_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb_dummy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/uwb_dummy
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(uwb_dummy_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb_dummy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/uwb_dummy
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(uwb_dummy_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb_dummy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/uwb_dummy
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(uwb_dummy_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb_dummy)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb_dummy\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/uwb_dummy
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(uwb_dummy_generate_messages_py std_msgs_generate_messages_py)
