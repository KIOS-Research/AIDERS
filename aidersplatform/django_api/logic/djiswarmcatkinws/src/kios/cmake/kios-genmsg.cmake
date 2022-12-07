# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "kios: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ikios:/home/aanast01/catkin_ws/src/kios/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(kios_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg" NAME_WE)
add_custom_target(_kios_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kios" "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg" ""
)

get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg" NAME_WE)
add_custom_target(_kios_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kios" "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg" "kios/GpsInput:std_msgs/Header"
)

get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg" NAME_WE)
add_custom_target(_kios_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kios" "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg" "kios/BasicCommandDJI:std_msgs/Header:kios/GpsInput:kios/FlightControlData"
)

get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg" NAME_WE)
add_custom_target(_kios_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kios" "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg" NAME_WE)
add_custom_target(_kios_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kios" "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg" ""
)

get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg" NAME_WE)
add_custom_target(_kios_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kios" "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg" "geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg" NAME_WE)
add_custom_target(_kios_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kios" "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios
)
_generate_msg_cpp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios
)
_generate_msg_cpp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios
)
_generate_msg_cpp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios
)
_generate_msg_cpp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios
)
_generate_msg_cpp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios
)
_generate_msg_cpp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios
)

### Generating Services

### Generating Module File
_generate_module_cpp(kios
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(kios_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(kios_generate_messages kios_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_cpp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg" NAME_WE)
add_dependencies(kios_generate_messages_cpp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_cpp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_cpp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg" NAME_WE)
add_dependencies(kios_generate_messages_cpp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg" NAME_WE)
add_dependencies(kios_generate_messages_cpp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_cpp _kios_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kios_gencpp)
add_dependencies(kios_gencpp kios_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kios_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(kios
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios
)
_generate_msg_eus(kios
  "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios
)
_generate_msg_eus(kios
  "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios
)
_generate_msg_eus(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios
)
_generate_msg_eus(kios
  "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios
)
_generate_msg_eus(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios
)
_generate_msg_eus(kios
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios
)

### Generating Services

### Generating Module File
_generate_module_eus(kios
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(kios_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(kios_generate_messages kios_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_eus _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg" NAME_WE)
add_dependencies(kios_generate_messages_eus _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_eus _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_eus _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg" NAME_WE)
add_dependencies(kios_generate_messages_eus _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg" NAME_WE)
add_dependencies(kios_generate_messages_eus _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_eus _kios_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kios_geneus)
add_dependencies(kios_geneus kios_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kios_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios
)
_generate_msg_lisp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios
)
_generate_msg_lisp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios
)
_generate_msg_lisp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios
)
_generate_msg_lisp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios
)
_generate_msg_lisp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios
)
_generate_msg_lisp(kios
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios
)

### Generating Services

### Generating Module File
_generate_module_lisp(kios
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(kios_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(kios_generate_messages kios_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_lisp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg" NAME_WE)
add_dependencies(kios_generate_messages_lisp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_lisp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_lisp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg" NAME_WE)
add_dependencies(kios_generate_messages_lisp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg" NAME_WE)
add_dependencies(kios_generate_messages_lisp _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_lisp _kios_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kios_genlisp)
add_dependencies(kios_genlisp kios_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kios_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(kios
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios
)
_generate_msg_nodejs(kios
  "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios
)
_generate_msg_nodejs(kios
  "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios
)
_generate_msg_nodejs(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios
)
_generate_msg_nodejs(kios
  "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios
)
_generate_msg_nodejs(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios
)
_generate_msg_nodejs(kios
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios
)

### Generating Services

### Generating Module File
_generate_module_nodejs(kios
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(kios_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(kios_generate_messages kios_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_nodejs _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg" NAME_WE)
add_dependencies(kios_generate_messages_nodejs _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_nodejs _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_nodejs _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg" NAME_WE)
add_dependencies(kios_generate_messages_nodejs _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg" NAME_WE)
add_dependencies(kios_generate_messages_nodejs _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_nodejs _kios_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kios_gennodejs)
add_dependencies(kios_gennodejs kios_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kios_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(kios
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios
)
_generate_msg_py(kios
  "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios
)
_generate_msg_py(kios
  "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg"
  "${MSG_I_FLAGS}"
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios
)
_generate_msg_py(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios
)
_generate_msg_py(kios
  "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios
)
_generate_msg_py(kios
  "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios
)
_generate_msg_py(kios
  "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios
)

### Generating Services

### Generating Module File
_generate_module_py(kios
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(kios_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(kios_generate_messages kios_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/BasicCommandDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_py _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/MissionDji.msg" NAME_WE)
add_dependencies(kios_generate_messages_py _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/InputDJI.msg" NAME_WE)
add_dependencies(kios_generate_messages_py _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_py _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/FlightControlData.msg" NAME_WE)
add_dependencies(kios_generate_messages_py _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/DroneState.msg" NAME_WE)
add_dependencies(kios_generate_messages_py _kios_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/aanast01/catkin_ws/src/kios/msg/GpsInput.msg" NAME_WE)
add_dependencies(kios_generate_messages_py _kios_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kios_genpy)
add_dependencies(kios_genpy kios_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kios_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kios
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(kios_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(kios_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(kios_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/kios
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(kios_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(kios_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(kios_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kios
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(kios_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(kios_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(kios_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/kios
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(kios_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(kios_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(kios_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kios
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(kios_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(kios_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(kios_generate_messages_py sensor_msgs_generate_messages_py)
endif()
