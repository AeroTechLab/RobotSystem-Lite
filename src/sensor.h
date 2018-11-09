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


/// @file sensor.h
/// @brief Generic sensor (measurement reading) functions
///
/// Interface for configurable sensor reading and state change. Specific underlying implementation (plug-in) and further configuration are defined as explained in @ref sensor_config

/// @page sensor_config Sensor Configuration
/// The sensor-level configuration (see [Configuration Levels](https://github.com/EESC-MKGroup/RobotSystem-Lite#robot-multi-level-configuration) is read using the [data I/O interface](https://labdin.github.io/Data-IO-Interface/data__io_8h.html)
///
/// Any configuration file/location path must be provided without its format extension, and relative to [<root_dir>](https://github.com/EESC-MKGroup/RobotSystem-Lite/blob/master/README.md#running)/config/sensors/
///
/// The possible configuration fields and their values are here exemplified for the case of a JSON format configuration (optional parameters are presented with default values and marked with '[o]' in their description):
/// @code
/// {
///   "input_interface": {                      // Hardware/virtual interface properties
///     "type": "<library_names>",                // Path (without extension) to plugin with signal input implementation (loaded from MODULES_DIR/signal_io/)
///     "config": "",                             // [o] Signal input/output device identifier passed to plugin initialization call
///     "channel": 0                              // Device channel from which input values will be read
///   },
///   "input_gain": {                           // [o] Input signal scaling parameters
///     "multiplier": 1.0,                        // [o] Value that multiplies the signal value
///     "divisor": 1.0                            // [o] Value that divides the signal value
///   },
///   "signal_processing": {                    // [o] Internal signal processing options
///     "rectified": false,                       // [o] Rectify signal if true
///     "normalized": false,                      // [o] Normalize signal (after calibration) if true
///     "min_frequency": -1.0                     // [o] Low-pass filter cut frequency, relative to (factor of) the sampling frequency (negative for no filtering)
///     "max_frequency": -1.0                     // [o] High-pass filter cut frequency, relative to (factor of) the sampling frequency (negative for no filtering)
///   },
///   "reference": "<sensor_identifier>",       // [o] String identifier (file name) or data object of configuration for measure reference sensor, if any
///   "differential_gain": {                    // [o] Input-reference differential signal scaling parameters
///     "multiplier": 1.0,                        // [o] Value that multiplies the signal value
///     "divisor": 1.0                            // [o] Value that divides the signal value
///   },
///   "log": {                                  // [o] Set logging of measurement numeric data over time
///     "file": "",                               // [o] Path (without extension), relative to to <log_dir>/sensors/, to log file. Default value will set terminal logging
///     "precision": 3                            // [o] Decimal precision for logged numeric values
///   }
/// }
/// @endcode

#ifndef SENSOR_H
#define SENSOR_H


#include "data_io/interface/data_io.h" 

#include <stdbool.h>


/// Selectable signal processing phases/modes
enum SensorState 
{ 
  SENSOR_STATE_MEASUREMENT,    ///< Default mode: signal is processed and result is returned assuming normal operation (measurement offset and gain, processing, reference offset and differential gain are applied)
  SENSOR_STATE_CALIBRATION,    ///< Minimum and maximum values from filtered signal are registered for posterior normalization, if specified
  SENSOR_STATE_OFFSET,         ///< Raw values mean is stored for posterior offset removal (no processed result is returned)
  SENSOR_STATES_NUMBER         ///< Number of signal processing phases/modes
};


typedef struct _SensorData SensorData;    ///< Single sensor internal data structure    
typedef SensorData* Sensor;               ///< Opaque reference to sensor internal data structure

                                                                   
/// @brief Creates and initializes sensor data structure based on given information                                              
/// @param[in] configuration reference to data object containing configuration parameters, as explained at @ref sensor_config
/// @return reference/pointer to newly created and initialized sensor data structure
Sensor Sensor_Init( DataHandle configuration );

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

/// @brief Sets current processing phase/state/mode of given sensor                     
/// @param[in] sensor reference to sensor
/// @param[in] newProcessingState desired signal processing phase
void Sensor_SetState( Sensor sensor, enum SensorState newState );


#endif // SENSOR_H
