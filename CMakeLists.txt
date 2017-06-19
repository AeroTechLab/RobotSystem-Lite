cmake_minimum_required( VERSION 2.8 )
project( RobRehabSystem )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR} )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )

set( SOURCES_DIR ${CMAKE_SOURCE_DIR}/src )

set( LIBRARY_DIR ${CMAKE_SOURCE_DIR}/libs CACHE PATH "Platform-Utils variable" FORCE )
set( MODULES_DIR ${CMAKE_SOURCE_DIR}/plugins CACHE PATH "Platform-Utils variable" FORCE )
set( PlatformUtils_DIR ${SOURCES_DIR}/utils CACHE PATH "Platform-Utils variable" FORCE )

include( ${SOURCES_DIR}/control/CMakeLists.txt )

include_directories( ${SOURCES_DIR} )
link_directories( ${LIBRARY_DIR} )


# (REAL-TIME) CONTROL APPLICATION
add_executable( RobRehabControl ${SOURCES_DIR}/robrehab_system.c ${SOURCES_DIR}/robrehab_control.c ${SOURCES_DIR}/shm_control.c )
target_compile_definitions( RobRehabControl PUBLIC -DDEBUG )
target_link_libraries( RobRehabControl RobotControl PlatformUtils )

# (AXIS/JOINT) SERVER
add_executable( RobRehabServer ${SOURCES_DIR}/robrehab_system.c ${SOURCES_DIR}/robrehab_network.c ${SOURCES_DIR}/shm_control.c )
target_compile_definitions( RobRehabServer PUBLIC -DDEBUG )
target_link_libraries( RobRehabServer PlatformUtils )

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
