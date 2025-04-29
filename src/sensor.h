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


/// @file sensor.h
/// @brief Generic sensor (measurement reading) functions
///
/// Interface for configurable sensor reading and state change. Specific underlying implementation (plug-in) and further configuration are defined as explained in @ref sensor_config

/// @page sensor_config Sensor Configuration
/// The sensor-level configuration (see [Configuration Levels](https://github.com/AeroTechLab/RobotSystem-Lite#robot-multi-level-configuration) is read using the [data I/O interface](https://labdin.github.io/Data-IO-Interface/data__io_8h.html)
///
/// Any configuration file/location path must be provided without its format extension, and relative to [<root_dir>](https://github.com/AeroTechLab/RobotSystem-Lite/blob/master/README.md#running)/config/sensors/
///
/// The possible configuration fields and their values are here exemplified for the case of a JSON format configuration (optional parameters are presented with default values and marked with '[o]' in their description):
/// @code
/// {
///   "inputs": [                               // List with input sources ("in0", "in1", ..., respectively) parameters
///     {
///       "interface": {                          // Hardware/virtual interface properties
///         "type": "<library_name>",               // Path (without extension) to plugin with signal input implementation (loaded from MODULES_DIR/signal_io/)
///         "config": "",                           // [o] Signal input/output device identifier passed to plugin initialization call
///         "channel": 0                            // Device channel from which input values will be read
///       },
///       "signal_processing": {                  // [o] Internal signal processing options
///         "rectified": false,                     // [o] Rectify signal if true
///         "normalized": false,                    // [o] Normalize signal (after calibration) if true
///         "min_frequency": -1.0                   // [o] Low-pass filter cutoff frequency, relative to (factor of) the sampling frequency (negative for no filtering)
///         "max_frequency": -1.0                   // [o] High-pass filter cutoff frequency, relative to (factor of) the sampling frequency (negative for no filtering)
///       }
///     }, ...
///   ],
///   "output": "in0",                          // [o] String with math expression for conversion from sensor inputs to output (like "tanh( in0 - in1 )")
///                                             //     Possible operations are the ones supported by TinyExpr library: https://codeplea.com/tinyexpr
///   "log": {                                  // [o] Set logging of inputs and measurement numeric data over time
///     "to_file": false,                         // [o] Save data logging to <log_dir>/[<user_name>-]<sensor_name>-<time_stamp>.log, to log file 
///                                               //     Default value will set terminal logging
///     "precision": 3                            // [o] Decimal precision for logged numeric values
///   }
/// }
/// @endcode

#ifndef SENSOR_H
#define SENSOR_H


#include <stdbool.h>


typedef struct _SensorData SensorData;    ///< Single sensor internal data structure    
typedef SensorData* Sensor;               ///< Opaque reference to sensor internal data structure

                                                                   
/// @brief Creates and initializes sensor data structure based on given information                                              
/// @param[in] configName name of file containing configuration parameters, as explained at @ref sensor_config
/// @return reference/pointer to newly created and initialized sensor data structure
Sensor Sensor_Init( const char* configName );

/// @brief Deallocates internal data of given sensor                        
/// @param[in] sensor reference to sensor
void Sensor_End( Sensor sensor );

/// @brief Performs single reading and processing of signal measured by given sensor
/// @param[in] sensor reference to sensor
/// @return current value of processed signal (0.0 on erros)
double Sensor_Update( Sensor sensor );

/// @brief Calls underlying signal reading implementation (plugin) to check for errors on given sensor              
/// @param[in] sensor reference to sensor
/// @return true on detected error, false otherwise
bool Sensor_HasError( Sensor sensor );

/// @brief Resets signal processing state and possible sensor device errors      
/// @param[in] sensor reference to sensor
void Sensor_Reset( Sensor sensor );

/// @brief Sets given sensor to offset acquisition mode                     
/// @param[in] sensor reference to sensor
void Sensor_SetOffset( Sensor sensor );

/// @brief Sets given sensor to range calibration mode       
/// @param[in] sensor reference to sensor
void Sensor_SetCalibration( Sensor sensor );

/// @brief Sets given sensor to measurement/operation mode                        
/// @param[in] sensor reference to sensor
void Sensor_SetMeasurement( Sensor sensor );


#endif // SENSOR_H
