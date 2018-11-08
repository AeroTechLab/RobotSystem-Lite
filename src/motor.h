////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2018 Leonardo Consoni <consoni_2519@hotmail.com>       //
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


/// @file motor.h
/// @brief Generic motors (actuation/signal output) functions
///
/// Interface for configurable motor control. Specific underlying implementation (plug-in) and further configuration are defined as explained in @ref motor_config

/// @page motor_config Motor Configuration
/// The motor-level configuration (see [Configuration Levels](https://github.com/LabDin/RobotSystem-Lite#robot-multi-level-configuration) is read using the [data I/O interface](https://labdin.github.io/Data-IO-Interface/data__io_8h.html)
///
/// Any configuration file/location path must be provided without its format extension, and relative to [<root_dir>](https://github.com/LabDin/RobotSystem-Lite/blob/master/README.md#running)/config/sensors/
///
/// The possible configuration fields and their values are here exemplified for the case of a JSON format configuration (optional parameters are presented with default values and marked with '[o]' in their description):
/// @code
/// {
///   "output_interface": {         // Hardware/virtual interface properties
///     "type": "<library_name>",     // Path (without extension) to plugin with signal output implementation (loaded from MODULES_DIR/signal_io/)
///     "config": "",                 // [o] Signal input/output device configuration string passed to plugin initialization call
///     "channel": 0                  // Device channel to which output/actuation values will be sent
///   },
///   "output_gain": {              // [o] Signal scaling parameters
///     "multiplier": 1.0,            // [o] Value that multiplies the signal value
///     "divisor": 1.0                // [o] Value that divides the signal value
///   },
///   "reference": "<sensor_identifier>",       // [o] String identifier (file name) or data object of configuration for motor reference sensor, if any 
///   "log": {                      // [o] Set logging of output numeric data over time
///     "file": "",                   // [o] Path (without extension), relative to to <log_dir>/motors/, to log file. Default value will set terminal logging
///     "precision": 3                // [o] Decimal precision for logged numeric values
///   }
/// }
/// @endcode

#ifndef MOTOR_H
#define MOTOR_H


#include "data_io/interface/data_io.h"

#include <stdbool.h>

typedef struct _MotorData MotorData;       ///< Single motor internal data structure    
typedef MotorData* Motor;                  ///< Opaque reference to motor internal data structure

                                                              
/// @brief Creates and initializes motor data structure based on given information                                              
/// @param[in] configuration reference to data object containing configuration parameters, as explained at @ref motor_config
/// @return reference/pointer to newly created and initialized motor data structure
Motor Motor_Init( DataHandle configuration );

/// @brief Deallocates internal data of given motor                        
/// @param[in] motor reference to motor
void Motor_End( Motor motor );

/// @brief Allows hardware/virtual device of given motor to output signal
/// @param[in] motor reference to motor
/// @return true on enabled output, false otherwise
bool Motor_Enable( Motor motor );
                                                                  
/// @brief Prevents hardware/virtual device of given motor from outputing signal
/// @param[in] motor reference to motor
void Motor_Disable( Motor motor );

/// @brief Calls underlying signal output implementation to check for errors on given motor              
/// @param[in] motor reference to motor
/// @return true on detected error, false otherwise
bool Motor_HasError( Motor motor );

/// @brief Calls underlying signal output implementation to reset possible device errors             
/// @param[in] motor reference to motor
void Motor_Reset( Motor motor );

/// @brief Enables/disables motor setpoint offset acquisition                
/// @param[in] motor reference to motor
/// @param[in] enabled true to start offset aquisition, false otherwise
void Motor_SetOffset( Motor motor, bool enabled );

/// @brief Writes specified value to given motor ouput device                
/// @param[in] motor reference to motor
/// @param[in] setpoint value to be written/generated
void Motor_WriteControl( Motor motor, double setpoint );


#endif  // MOTOR_H
