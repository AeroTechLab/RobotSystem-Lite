cmake_minimum_required( VERSION 2.8 )
project( RobotControl )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )

set( SOURCES_DIR ${CMAKE_SOURCE_DIR}/src )

set( LIBRARY_DIR ${CMAKE_SOURCE_DIR}/libs CACHE PATH "Linked libraries directory" FORCE )
set( MODULES_DIR ${CMAKE_SOURCE_DIR}/plugins CACHE PATH "Dynamic modules output directory" FORCE )

set( SIGNAL_IO_MODULES_PATH ${MODULES_DIR}/signal_io )
set( ACTUATOR_CONTROL_MODULES_PATH ${MODULES_DIR}/actuator_control )
set( ROBOT_CONTROL_MODULES_PATH ${MODULES_DIR}/robot_control )

include( ${SOURCES_DIR}/debug/CMakeLists.txt )
include( ${SOURCES_DIR}/curves/CMakeLists.txt )
include( ${SOURCES_DIR}/json/CMakeLists.txt )
include( ${SOURCES_DIR}/kalman/CMakeLists.txt )
include( ${SOURCES_DIR}/signal_processing/CMakeLists.txt )
include( ${SOURCES_DIR}/threads/CMakeLists.txt )
include( ${SOURCES_DIR}/timing/CMakeLists.txt )

include_directories( ${SOURCES_DIR} )
link_directories( ${LIBRARY_DIR} )


# (REAL-TIME) CONTROL APPLICATION
add_executable( RobotControl ${SOURCES_DIR}/main.c ${SOURCES_DIR}/system.c ${SOURCES_DIR}/robots.c ${SOURCES_DIR}/actuators.c ${SOURCES_DIR}/sensors.c ${SOURCES_DIR}/motors.c ${SOURCES_DIR}/ipc.c ${SOURCES_DIR}/data_io.c )
target_compile_definitions( RobotControl PUBLIC -DDEBUG -DSIGNAL_IO_MODULES_PATH=${SIGNAL_IO_MODULES_PATH} -DACTUATOR_CONTROL_MODULES_PATH=${ACTUATOR_CONTROL_MODULES_PATH} -DROBOT_CONTROL_MODULES_PATH=${ROBOT_CONTROL_MODULES_PATH} )
target_link_libraries( RobotControl DataLogging FunctionCurves SimpleJSON KalmanFilters SignalProcessing MultiThreading Timing ${CMAKE_DL_LIBS} )
if( UNIX AND NOT APPLE )
  target_link_libraries( RobotControl -lrt )
endif()

# PLUGINS/MODULES

add_library( ForcePIControl MODULE ${SOURCES_DIR}/actuator_control/force_pi_control.c )
set_target_properties( ForcePIControl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${ACTUATOR_CONTROL_MODULES_PATH} )
set_target_properties( ForcePIControl PROPERTIES PREFIX "" )
target_include_directories( ForcePIControl PUBLIC ${SOURCES_DIR}/actuator_control/ )

add_library( SimpleControl MODULE ${SOURCES_DIR}/actuator_control/simple_control.c )
set_target_properties( SimpleControl PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${ACTUATOR_CONTROL_MODULES_PATH} )
set_target_properties( SimpleControl PROPERTIES PREFIX "" )
target_include_directories( SimpleControl PUBLIC ${SOURCES_DIR}/actuator_control/ )

add_library( SimpleJoint MODULE ${SOURCES_DIR}/robot_control/simple_joint.c )
set_target_properties( SimpleJoint PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${ROBOT_CONTROL_MODULES_PATH} )
set_target_properties( SimpleJoint PROPERTIES PREFIX "" )
target_include_directories( SimpleJoint PUBLIC ${SOURCES_DIR}/robot_control/ )

add_library( AnkleBot MODULE ${SOURCES_DIR}/robot_control/anklebot.c )
set_target_properties( AnkleBot PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${ROBOT_CONTROL_MODULES_PATH} )
set_target_properties( AnkleBot PROPERTIES PREFIX "" )
target_include_directories( AnkleBot PUBLIC ${SOURCES_DIR}/robot_control/ )

add_library( EMGJoint MODULE ${SOURCES_DIR}/robot_control/emg/emg_control.c ${SOURCES_DIR}/robot_control/emg/emg_muscular_model.c )
set_target_properties( EMGJoint PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${ROBOT_CONTROL_MODULES_PATH} )
set_target_properties( EMGJoint PROPERTIES PREFIX "" )
target_include_directories( EMGJoint PUBLIC ${SOURCES_DIR}/robot_control/ ${SOURCES_DIR}/robot_control/emg/ )

# install( TARGETS RobRehabGui RobRehabServer RUNTIME DESTINATION . )
