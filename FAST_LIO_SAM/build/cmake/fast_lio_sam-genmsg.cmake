# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fast_lio_sam: 1 messages, 2 services")

set(MSG_I_FLAGS "-Ifast_lio_sam:/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fast_lio_sam_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg" NAME_WE)
add_custom_target(_fast_lio_sam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fast_lio_sam" "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg" ""
)

get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv" NAME_WE)
add_custom_target(_fast_lio_sam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fast_lio_sam" "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv" ""
)

get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv" NAME_WE)
add_custom_target(_fast_lio_sam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fast_lio_sam" "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fast_lio_sam
)

### Generating Services
_generate_srv_cpp(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fast_lio_sam
)
_generate_srv_cpp(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fast_lio_sam
)

### Generating Module File
_generate_module_cpp(fast_lio_sam
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fast_lio_sam
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fast_lio_sam_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fast_lio_sam_generate_messages fast_lio_sam_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_cpp _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_cpp _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_cpp _fast_lio_sam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fast_lio_sam_gencpp)
add_dependencies(fast_lio_sam_gencpp fast_lio_sam_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fast_lio_sam_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fast_lio_sam
)

### Generating Services
_generate_srv_eus(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fast_lio_sam
)
_generate_srv_eus(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fast_lio_sam
)

### Generating Module File
_generate_module_eus(fast_lio_sam
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fast_lio_sam
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fast_lio_sam_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fast_lio_sam_generate_messages fast_lio_sam_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_eus _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_eus _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_eus _fast_lio_sam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fast_lio_sam_geneus)
add_dependencies(fast_lio_sam_geneus fast_lio_sam_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fast_lio_sam_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fast_lio_sam
)

### Generating Services
_generate_srv_lisp(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fast_lio_sam
)
_generate_srv_lisp(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fast_lio_sam
)

### Generating Module File
_generate_module_lisp(fast_lio_sam
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fast_lio_sam
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fast_lio_sam_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fast_lio_sam_generate_messages fast_lio_sam_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_lisp _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_lisp _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_lisp _fast_lio_sam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fast_lio_sam_genlisp)
add_dependencies(fast_lio_sam_genlisp fast_lio_sam_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fast_lio_sam_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fast_lio_sam
)

### Generating Services
_generate_srv_nodejs(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fast_lio_sam
)
_generate_srv_nodejs(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fast_lio_sam
)

### Generating Module File
_generate_module_nodejs(fast_lio_sam
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fast_lio_sam
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fast_lio_sam_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fast_lio_sam_generate_messages fast_lio_sam_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_nodejs _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_nodejs _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_nodejs _fast_lio_sam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fast_lio_sam_gennodejs)
add_dependencies(fast_lio_sam_gennodejs fast_lio_sam_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fast_lio_sam_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fast_lio_sam
)

### Generating Services
_generate_srv_py(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fast_lio_sam
)
_generate_srv_py(fast_lio_sam
  "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fast_lio_sam
)

### Generating Module File
_generate_module_py(fast_lio_sam
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fast_lio_sam
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fast_lio_sam_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fast_lio_sam_generate_messages fast_lio_sam_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/msg/Pose6D.msg" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_py _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_pose.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_py _fast_lio_sam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/fast_lio_sam_ws/src/FAST_LIO_SAM/srv/save_map.srv" NAME_WE)
add_dependencies(fast_lio_sam_generate_messages_py _fast_lio_sam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fast_lio_sam_genpy)
add_dependencies(fast_lio_sam_genpy fast_lio_sam_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fast_lio_sam_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fast_lio_sam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fast_lio_sam
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(fast_lio_sam_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fast_lio_sam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fast_lio_sam
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(fast_lio_sam_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fast_lio_sam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fast_lio_sam
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(fast_lio_sam_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fast_lio_sam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fast_lio_sam
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(fast_lio_sam_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fast_lio_sam)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fast_lio_sam\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fast_lio_sam
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(fast_lio_sam_generate_messages_py geometry_msgs_generate_messages_py)
endif()
