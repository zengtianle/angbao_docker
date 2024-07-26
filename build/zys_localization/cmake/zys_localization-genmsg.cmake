# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "zys_localization: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(zys_localization_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv" NAME_WE)
add_custom_target(_zys_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "zys_localization" "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(zys_localization
  "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zys_localization
)

### Generating Module File
_generate_module_cpp(zys_localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zys_localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(zys_localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(zys_localization_generate_messages zys_localization_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv" NAME_WE)
add_dependencies(zys_localization_generate_messages_cpp _zys_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zys_localization_gencpp)
add_dependencies(zys_localization_gencpp zys_localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zys_localization_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(zys_localization
  "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zys_localization
)

### Generating Module File
_generate_module_eus(zys_localization
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zys_localization
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(zys_localization_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(zys_localization_generate_messages zys_localization_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv" NAME_WE)
add_dependencies(zys_localization_generate_messages_eus _zys_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zys_localization_geneus)
add_dependencies(zys_localization_geneus zys_localization_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zys_localization_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(zys_localization
  "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zys_localization
)

### Generating Module File
_generate_module_lisp(zys_localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zys_localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(zys_localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(zys_localization_generate_messages zys_localization_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv" NAME_WE)
add_dependencies(zys_localization_generate_messages_lisp _zys_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zys_localization_genlisp)
add_dependencies(zys_localization_genlisp zys_localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zys_localization_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(zys_localization
  "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zys_localization
)

### Generating Module File
_generate_module_nodejs(zys_localization
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zys_localization
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(zys_localization_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(zys_localization_generate_messages zys_localization_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv" NAME_WE)
add_dependencies(zys_localization_generate_messages_nodejs _zys_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zys_localization_gennodejs)
add_dependencies(zys_localization_gennodejs zys_localization_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zys_localization_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(zys_localization
  "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zys_localization
)

### Generating Module File
_generate_module_py(zys_localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zys_localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(zys_localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(zys_localization_generate_messages zys_localization_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/courierlo/autoslam_gm/src/zys_localization/srv/localization_srv.srv" NAME_WE)
add_dependencies(zys_localization_generate_messages_py _zys_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(zys_localization_genpy)
add_dependencies(zys_localization_genpy zys_localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS zys_localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zys_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/zys_localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(zys_localization_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zys_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/zys_localization
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(zys_localization_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zys_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/zys_localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(zys_localization_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zys_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/zys_localization
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(zys_localization_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zys_localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zys_localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/zys_localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(zys_localization_generate_messages_py std_msgs_generate_messages_py)
endif()
