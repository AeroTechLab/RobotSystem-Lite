////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>       //
//                                                                            //
//  This file is part of RobRehabSystem.                                      //
//                                                                            //
//  RobRehabSystem is free software: you can redistribute it and/or modify    //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobRehabSystem is distributed in the hope that it will be useful,         //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


/// @file robots.h
/// @brief Generic robot functions
///
/// Interface for configurable robot control. Specific underlying implementation and further configuration are defined as explained in @ref robot_config

/// @page robot_config Robot Configuration
/// The robot-level configuration (see @ref configuration_levels) is read using the [data I/O implementation currently defined](https://bitiquinho.github.io/Platform-Utils/structDataIO.html). Configuration of listed joint actuators (see @ref joint_axis_rationale) is loaded recursively (as described in @ref actuator_config)
///
/// Any configuration file/location path must be provided without its format extension, and relative to CONFIG_DIR/robots/, where CONFIG_DIR is the [defined base data path](https://bitiquinho.github.io/Platform-Utils/classDATA__IO__INTERFACE.html)
///
/// The possible configuration fields and their values are here exemplified for the case of a JSON format configuration:
/// @code
/// {
///   "controller": {               // Robot controller configuration
///     "type": "<library_name>",     // Path (without extension, relative to MODULES_DIR/robot_control/) to plugin with robot controller implementation
///     "config": "..."               // Configuration string passed to Controller specific initialization
///   },
///   "actuators": [                // List of robot actuators identifiers (strings) or configurations (objects)
///     "<actuator_identifier>",      // Actuator string identifier (configuration file name)
///     { /*...*/ }                   // Inline actuator configuration (data object)
///   ]           
/// }
/// @endcode

#ifndef ROBOTS_H
#define ROBOTS_H

#include "robot_control_interface.h"

#include <stdbool.h>
#include <stddef.h>

typedef struct _JointData JointData;      ///< Single robot joint internal data structure (see @ref joint_axis_rationale)  
typedef JointData* Joint;                 ///< Opaque reference to robot joint internal data structure (see @ref joint_axis_rationale) 

typedef struct _AxisData AxisData;        ///< Single robot axis internal data structure (see @ref joint_axis_rationale)    
typedef AxisData* Axis;                   ///< Opaque reference to robot axis internal data structure (see @ref joint_axis_rationale) 

typedef struct _RobotData RobotData;      ///< Single robot internal data structure    
typedef RobotData* Robot;                 ///< Opaque reference to robot internal data structure

                  
/// @brief Creates and initializes robot data structure based on given information                                              
/// @param[in] configFileName name/path of file/location containing output configuration, as explained at @ref robot_config
/// @return identifier to newly created and initialized robot
Robot Robot_Init( const char* configFileName );

/// @brief Deallocates internal data of given robot                        
/// @param[in] robot robot identifier
void Robot_End( Robot robot );

/// @brief Initializes (if not running) update/operation thread for the given robot
/// @param[in] robot robot identifier
/// @return true on if control state was changed, false otherwise
bool Robot_Enable( Robot robot );
                                                                 
/// @brief Terminates (if running) update/operation thread for the given robot
/// @param[in] robot robot identifier
/// @return true on if control state was changed, false otherwise
bool Robot_Disable( Robot robot );

/// @brief Calls underlying actuators and control implementations to change measurement/control state          
/// @param[in] robot robot identifier
/// @param[in] controlState new control state to be set
/// @return true on if control state was changed, false otherwise
bool Robot_SetControlState( Robot robot, enum RobotState controlState );

/// @brief Gets reference to specified joint of given robot (see @ref joint_axis_rationale)    
/// @param[in] robot robot identifier
/// @param[in] jointIndex index of robot joint (in the order listed on robot's configuration)
/// @return pointer/reference to robot joint (NULL on errors or no joint of specified index)
Joint Robot_GetJoint( Robot robot, size_t jointIndex );

/// @brief Gets reference to specified axis of given robot (see @ref joint_axis_rationale)         
/// @param[in] robot robot identifier
/// @param[in] axisIndex index of robot axis (in the order listed on robot's configuration)
/// @return pointer/reference to robot axis (NULL on errors or no axis of specified index)
Axis Robot_GetAxis( Robot robot, size_t axisIndex );

/// @brief Gets string identifier for specified joint of given robot         
/// @param[in] robot robot identifier
/// @param[in] jointIndex index of robot joint (in the order listed on robot's configuration)
/// @return pointer to string of robot joint name (NULL on errors or no joint of specified index)
const char* Robot_GetJointName( Robot robot, size_t jointIndex );

/// @brief Calls underlying (plugin) implementation to get string identifier for specified axis of given robot               
/// @param[in] robot robot identifier
/// @param[in] axisIndex index of robot axis (in the order listed on robot's configuration)
/// @return pointer to string of robot axis name (NULL on errors or no axis of specified index)
const char* Robot_GetAxisName( Robot robot, size_t axisIndex );

/// @brief Calls underlying (plugin) implementation to get current value of specified measure for given joint         
/// @param[in] joint reference to joint
/// @param[in] ref_measures index/identifier of desired variable
/// @return true on if new values were acquired, false otherwise
bool Robot_GetJointMeasures( Joint joint, RobotVariables* ref_measures );

/// @brief Gets current value of specified measure for given axis         
/// @param[in] axis reference to joint
/// @param[in] ref_measures index/identifier of desired variable
/// @return true on if new values were acquired, false otherwise
bool Robot_GetAxisMeasures( Axis axis, RobotVariables* ref_measures );

/// @brief Sets value of specified setpoint for given joint          
/// @param[in] joint reference to joint
/// @param[in] ref_setpoints index/identifier of updated variable
void Robot_SetJointSetpoints( Joint joint, RobotVariables* ref_setpoints );

/// @brief Sets value of specified setpoint for given axis       
/// @param[in] axis reference to axis
/// @param[in] ref_setpoints index/identifier of updated variable
void Robot_SetAxisSetpoints( Axis axis, RobotVariables* ref_setpoints );

/// @brief Calls underlying (plugin) implementation to get number of joint degrees-of-freedom for given robot        
/// @param[in] robot robot identifier
/// @return number of joint degrees-of-freedom
size_t Robot_GetJointsNumber( Robot robot );

/// @brief Calls underlying (plugin) implementation to get number of axis degrees-of-freedom for given robot              
/// @param[in] robot robot identifier
/// @return number of axis degrees-of-freedom
size_t Robot_GetAxesNumber( Robot robot );


#endif // ROBOTS_H 
