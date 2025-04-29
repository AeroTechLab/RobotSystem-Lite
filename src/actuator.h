////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2020 Leonardo Consoni <leonardojc@protonmail.com>      //
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


/// @file actuator.h
/// @brief Generic actuator control (measurement + actuation) functions
///
/// Interface for configurable actuator control. Available configuration options are detailed in @ref actuator_config.
/// Every actuator contains a single motor and a set of sensors combined with a [Kalman filter](https://github.com/AeroTechLab/Simple-Kalman-Filter) for motion measurements (see [Configuration Levels](https://github.com/AeroTechLab/RobotSystem-Lite#robot-multi-level-configuration). 

/// @page actuator_config Actuator Configuration
/// The actuator-level configuration (see [Configuration Levels](https://github.com/AeroTechLab/RobotSystem-Lite#robot-multi-level-configuration) is read using the [data I/O interface](https://labdin.github.io/Data-IO-Interface/data__io_8h.html). Configuration of sensors and motors listed is loaded recursively (as in @ref sensor_config and @ref motor_config)
///
/// Any configuration path must be provided without file extension, and relative to [<root_dir>](https://github.com/AeroTechLab/RobotSystem-Lite/blob/master/README.md#running)/config/actuators/
///
/// The possible configuration fields and their values are here exemplified for the case of current JSON file I/O implementation (optional parameters are presented with default values and marked with '[o]' in their description):
/// @code
/// {
///   "sensors": [                        // List of sensors available for configured actuator
///     { 
///       "variable": "POSITION",           // Dimension measured by sensor (POSITION, VELOCITY, FORCE or ACCELERATION)
///       "config": "<sensor_1_id>",        // Sensor string identifier, used for searching its configuration file
///       "deviation": 1.0                  // [o] Measurement error (standard deviation) associated with the sensor
///     },
///     { 
///       "input_variable": "FORCE",
///       "config": "<sensor_2_id>"          
///     }, ...
///   ],
///   "motor": {                          // Actuation motor used on configured actuator
///     "variable": "VELOCITY",             // Controlled dimension/variable (POSITION, VELOCITY, FORCE or ACCELERATION)
///     "config": "<motor_identifier>",     // Motor string identifier (configuration file path) or inline configuration object 
///     "limit": -1.0                       // [o] Absolute maximum allowed for control motor setpoint/output
///   },
///   "log": {                            // [o] Set logging of measurement and setpoint numeric data over time
///     "to_file": false,                   // [o] Save data logging to <log_dir>/[<user_name>-]<actuator_name>-<time_stamp>.log, to log file 
///                                         //     Default value will set terminal logging
///     "precision": 3                      // [o] Decimal precision for logged numeric values
///   }
/// }
/// @endcode


#ifndef ACTUATOR_H
#define ACTUATOR_H 

#include "robot_control/robot_control.h"

#include <stdbool.h>

typedef struct _ActuatorData ActuatorData;      ///< Single actuator internal data structure    
typedef ActuatorData* Actuator;                 ///< Opaque reference to actuator internal data structure

                                                                  
/// @brief Creates and initializes actuator data structure based on given information                                              
/// @param[in] configName name of file containing configuration parameters, as explained at @ref actuator_config
/// @return reference/pointer to newly created and initialized actuator data structure
Actuator Actuator_Init( const char* configName );

/// @brief Deallocates internal data of given actuator                        
/// @param[in] actuator reference to actuator
void Actuator_End( Actuator actuator );

/// @brief Allows motor output on given actuator 
/// @param[in] actuator reference to actuator
/// @return true on enabled output, false otherwise
bool Actuator_Enable( Actuator actuator );
                                                                  
/// @brief Prevents motor output on given actuator 
/// @param[in] actuator reference to actuator
void Actuator_Disable( Actuator actuator );

/// @brief Calls underlying sensors implementations (plugins) to change measurement state          
/// @param[in] actuator reference to actuator
/// @param[in] controlState new control state to be set
/// @return true if control state was changed, false otherwise
bool Actuator_SetControlState( Actuator actuator, enum ControlState controlState );

/// @brief Reads sensors of given actuator               
/// @param[in] actuator reference to actuator
/// @param[out] ref_measures pointer to variables structure where values will be stored
/// @param[in] timeDelta time passed between measurements for sensor filtering/fusion state prediction
/// @return true if new measurements were taken, false otherwise
bool Actuator_GetMeasures( Actuator actuator, DoFVariables* ref_measures, double timeDelta );

/// @brief Writes possible motor setpoint values for given actuator       
/// @param[in] actuator reference to actuator
/// @param[in] ref_setpoints pointer/reference to variables structure with the new setpoints
/// @return control action applied on motor of given actuator (control variable specified in @ref actuator_config)
double Actuator_SetSetpoints( Actuator actuator, DoFVariables* ref_setpoints );


#endif // ACTUATOR_H
