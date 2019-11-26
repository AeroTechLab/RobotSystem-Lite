////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2019 Leonardo Consoni <leonardojc@protonmail.com>      //
//                                                                            //
//  This file is part of RobotSystem-Lite.                                    //
//                                                                            //
//  RobotSystem-Lite is free software: you can redistribute it and/or modify  //
//  it under the terms of the GNU Lesser General Public License as published  //
//  by the Free Software Foundation, either version 3 of the License, or      //
//  (at your option) any later version.                                       //
//                                                                            //
//  RobotSystem-Lite is distributed in the hope that it will be useful,       //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of            //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              //
//  GNU Lesser General Public License for more details.                       //
//                                                                            //
//  You should have received a copy of the GNU Lesser General Public License  //
//  along with RobotSystem-Lite. If not, see <http://www.gnu.org/licenses/>.  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


/// @file robot.h
/// @brief Generic robot functions
///
/// Interface for configurable robot control. Specific underlying implementation (plug-in) and further configuration are defined as explained in @ref robot_config.
/// A robot works with 2 sets of coordinates: axes (read-write) and joints (read-only). For a detailed explanation, see @ref joint_axis_rationale.
/// Even if RobotSystem handles one robot at a time, this code supports multiple robots for reusage in different applications.

/// @page robot_config Robot Configuration
/// The robot-level configuration (see [Configuration Levels](https://github.com/EESC-MKGroup/RobotSystem-Lite#robot-multi-level-configuration) is read using the [data I/O interface](https://labdin.github.io/Data-IO-Interface/data__io_8h.html). Configuration of listed joint actuators is loaded recursively (as described in @ref actuator_config)
///
/// Any configuration path must be provided without file extension, and relative to and relative to [<root_dir>](https://github.com/EESC-MKGroup/RobotSystem-Lite/blob/master/README.md#running)/config/robots/
///
/// The possible configuration fields and their values are here exemplified for the case of current JSON file I/O implementation (optional parameters are presented with default values and marked with '[o]' in their description):
/// @code
/// {
///   "controller": {               // Robot controller configuration
///     "type": "<library_name>",   // Path (without extension, relative to MODULES_DIR/robot_control/) to plugin with robot controller implementation
///     "config": ""                // [o] Custom-format configuration string passed to controller (plugin) specific initialization
///   },
///   "actuators": [                // List of robot actuators identifiers (strings) or configurations (objects)
///     "<actuator_1_id>",          // Actuator string identifier (configuration file name)
///     "<actuator_2_id>", ...      
///   ],
///   "extra_inputs": [             // [o] Additional inputs configuration (as for inputs in sensor configuration)
///     {
///       "interface": { ... },
///       "signal_processing": { ... }
///     }, ...
///   ],
///   "extra_outputs": [            // [o] Additional outputs configuration (as for outputs in motor configuration)
///     {
///       "interface": { ... }
///     }, ...
///   ]
///   "log": {                      // [o] Set logging of axis setpoint/measurement and extra input/output numeric data over time
///     "to_file": false,             // [o] Save data logging to <log_dir>/[<user_name>-]<robot_name>-<time_stamp>.log, to log file 
///                                   //     Default value will set terminal logging
///     "precision": 3                // [o] Decimal precision for logged numeric values
///   }
/// }
/// @endcode

#ifndef ROBOT_H
#define ROBOT_H

#include "robot_control/robot_control.h"

#include <stdbool.h>
#include <stddef.h>

                  
/// @brief Creates and initializes robot data structure based on given information                                              
/// @param[in] configPathName path to robot configuration, as explained at @ref robot_config
/// @return true on successful initialization, false otherwise
bool Robot_Init( const char* configPathName );

/// @brief Deallocates internal data of given robot                        
void Robot_End();

/// @brief Initializes (if not running) update/operation thread for the given robot
/// @return true if control state was changed, false otherwise
bool Robot_Enable();
                                                                 
/// @brief Terminates (if running) update/operation thread for the given robot
/// @return true if control state was changed, false otherwise
bool Robot_Disable();

/// @brief Change control state of given robot actuators and underlying (plugin) control implementation           
/// @param[in] controlState new control state to be set
/// @return true if control state was changed, false otherwise
bool Robot_SetControlState( enum ControlState controlState );

/// @brief Calls underlying (plugin) implementation to get string identifier for specified joint in given robot                
/// @param[in] jointIndex index of robot joint (in the order listed on robot's configuration)
/// @return pointer to string of robot joint name (NULL on errors or no joint of specified index)
const char* Robot_GetJointName( size_t jointIndex );

/// @brief Calls underlying (plugin) implementation to get string identifier for specified axis in given robot               
/// @param[in] axisIndex index of robot axis (in the order listed on robot's configuration)
/// @return pointer to string of robot axis name (NULL on errors or no axis of specified index)
const char* Robot_GetAxisName( size_t axisIndex );

/// @brief Gets current value of specified joint measurements (see @ref joint_axis_rationale)          
/// @param[in] jointIndex index of robot axis (in the order listed on robot's configuration)
/// @param[out] ref_measures pointer/reference to variables structure where values will be stored
/// @return true on if new values were acquired, false otherwise
bool Robot_GetJointMeasures( size_t jointIndex, DoFVariables* ref_measures );

/// @brief Gets current value of specified axis measurements (see @ref joint_axis_rationale)          
/// @param[in] axisIndex index of robot axis (in the order listed on robot's configuration)
/// @param[out] ref_measures pointer/reference to variables structure where values will be stored
/// @return true on if new values were acquired, false otherwise
bool Robot_GetAxisMeasures( size_t axisIndex, DoFVariables* ref_measures );

/// @brief Sets value of specified setpoint for given axis       
/// @param[in] axisIndex index of robot axis (in the order listed on robot's configuration)
/// @param[in] ref_setpoints pointer/reference to variables structure with the new setpoints
void Robot_SetAxisSetpoints( size_t axisIndex, DoFVariables* ref_setpoints );

/// @brief Calls underlying (plugin) implementation to get number of joint degrees-of-freedom for given robot        
/// @return number of joint degrees-of-freedom
size_t Robot_GetJointsNumber();

/// @brief Calls underlying (plugin) implementation to get number of axis degrees-of-freedom for given robot              
/// @return number of axis degrees-of-freedom
size_t Robot_GetAxesNumber();


#endif // ROBOT_H 
