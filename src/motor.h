////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2025 Leonardo Consoni <leonardojc@protonmail.com>      //
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
/// The motor-level configuration (see [Configuration Levels](https://github.com/AeroTechLab/RobotSystem-Lite#robot-multi-level-configuration) is read using the [data I/O interface](https://labdin.github.io/Data-IO-Interface/data__io_8h.html)
///
/// Any configuration file/location path must be provided without its format extension, and relative to [<root_dir>](https://github.com/AeroTechLab/RobotSystem-Lite/blob/master/README.md#running)/config/sensors/
///
/// The possible configuration fields and their values are here exemplified for the case of a JSON format configuration (optional parameters are presented with default values and marked with '[o]' in their description):
/// @code
/// {
///   "interface": {                  // Hardware/virtual output interface properties
///     "type": "<library_name>",       // Path (without extension) to plugin with signal output implementation (loaded from MODULES_DIR/signal_io/)
///     "config": "",                   // [o] Signal input/output device configuration string passed to plugin initialization call
///     "channel": 0                    // Device channel to which output/actuation values will be sent
///   },
///   "reference": {                  // [o] Offset reference ("ref") input configuration (as for single input configuration in sensor configuration)
///     "interface": { ... },         //     Values for offset reference are only aquired during control offset state
///     "signal_processing": { ... }
///   },
///   "output": "set",                // [o] String with math expression for conversion from control setpoint ("set") and offset reference ("ref") to output
///                                   //     Possible operations are the ones supported by TinyExpr library: https://codeplea.com/tinyexpr
///   "log": {                        // [o] Set logging of setpoint, offset and output numeric data over time
///     "to_file": false,               // [o] Save data logging to <log_dir>/[<user_name>-]<motor_name>-<time_stamp>.log, to log file 
///                                     //     Default value will set terminal logging
///     "precision": 3                  // [o] Decimal precision for logged numeric values
///   }
/// }
/// @endcode

#ifndef MOTOR_H
#define MOTOR_H


#include <stdbool.h>

typedef struct _MotorData MotorData;       ///< Single motor internal data structure    
typedef MotorData* Motor;                  ///< Opaque reference to motor internal data structure

                                                              
/// @brief Creates and initializes motor data structure based on given information                                              
/// @param[in] configName name of file containing configuration parameters, as explained at @ref motor_config
/// @return reference/pointer to newly created and initialized motor data structure
Motor Motor_Init( const char* configName );

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

/// @brief Enables motor setpoint offset acquisition                
/// @param[in] motor reference to motor
void Motor_SetOffset( Motor motor );

/// @brief Enables motor output/operation               
/// @param[in] motor reference to motor
void Motor_SetOperation( Motor motor );

/// @brief Writes specified value to given motor ouput device                
/// @param[in] motor reference to motor
/// @param[in] setpoint value to be written/generated
void Motor_WriteControl( Motor motor, double setpoint );


#endif  // MOTOR_H
