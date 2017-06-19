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


/// @file sensors.h
/// @brief Generic sensor (measure reading) functions
///
/// Interface for configurable sensor reading and control. Specific underlying implementation and further configuration are defined as explained in @ref sensor_config

/// @page sensor_config Sensor Configuration
/// The sensor-level configuration (see @ref configuration_levels) is read using the [data I/O implementation currently defined](https://bitiquinho.github.io/Platform-Utils/structDataIO.html). Internal conversion curve, if needed, is defined according to @ref curve_config
///
/// Any configuration file/location path must be provided without its format extension, and relative to CONFIG_DIR/sensors/, where CONFIG_DIR is the [defined base data path](https://bitiquinho.github.io/Platform-Utils/classDATA__IO__INTERFACE.html)
///
/// The possible configuration fields and their values are here exemplified for the case of a JSON format configuration:
/// @code
/// {
///   "input_interface": {                      // Hardware/virtual interface properties
///     "type": "<library_names>",                // Path (without extension) to plugin with signal input implementation (loaded from MODULES_DIR/signal_io/)
///     "config": "...",                          // Signal input/output device identifier passed to plugin initialization call
///     "channel": 0                              // Device channel from which input values will be read
///   },
///   "input_gain": {                           // Signal scaling parameters
///     "multiplier": 1.0,                        // Value that multiplies the signal value
///     "divisor": 1.0                            // Value that divides the signal value
///   },
///   "signal_processing": {                    // Internal signal processor options
///     "rectified": false,                       // Rectify signal if true
///     "normalized": false,                      // Normalize signal (after calibration) if true
///     "smoothing_frequency": -1.0               // Low-pass filter cut frequency, relative to (factor of) the sampling frequency (negative for no smoothing)
///   },
///   "relative_to": "<sensor_identifier>",     // String identifier (file name) or data object of configuration for measure reference sensor, if any
///   "conversion_curve": {                     // Curve configuration string identifier or data object, if needed for more complex relation between raw and processed values
///     "segments": [                                             // List of segments that compose the entire curve
///       { 
///         "type": "polynomial",                                   // Segment defined by a polynomial expression
///         "bounds": [ -0.5, -0.0032 ],                            // Limits of segment function domain
///         "parameters": [ -652.05, -0.3701 ]                      // Polynom coefficients (from bigger to lower order)
///       },
///       { 
///         "type": "cubic_spline",                                 // Segment defined by cubic (4 coefficients) spline points
///         "bounds": [ -0.0032, 0.0032 ],                          // Limits of segment function domain
///         "parameters": [ 1.7165, -652.05, -1.5608, -671.77 ]     // Function value and derivative (respectively) on each spline bound
///       }
///     ],
///     "scale_factor": 1.0                                       // Multiply curve value by a factor
///   },
///   "log_data": false                         // Set true to save signal reading log to LOGS_DIR/signal_io
/// }
/// @endcode

#ifndef SENSORS_H
#define SENSORS_H

#include "data_io.h"

#include "signal_processing/signal_processing.h"


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
/// @param[out] rawBuffer pointer to preallocated buffer to hold all original raw samples (size returned by GetInputBufferLength()) acquired by underlying signal reading implementation (NULL, if not needed)
/// @return current value of processed signal (0.0 on erros)
double Sensor_Update( Sensor sensor, double* rawBuffer );
                                                                   
/// @brief Gets length of internal raw input samples buffer for given sensor
/// @param[in] sensor reference to sensor
/// @return size (in elements) of internal input buffer
size_t Sensor_GetInputBufferLength( Sensor sensor );

/// @brief Calls underlying signal reading implementation to check for errors on given sensor              
/// @param[in] sensor reference to sensor
/// @return true on detected error, false otherwise
bool Sensor_HasError( Sensor sensor );

/// @brief Resets signal processing state and possible sensor device errors      
/// @param[in] sensor reference to sensor
void Sensor_Reset( Sensor sensor );

/// @brief Sets current processing phase/state/mode of given sensor                     
/// @param[in] sensor reference to sensor
/// @param[in] newProcessingState desired signal processing phase
void Sensor_SetState( Sensor sensor, enum SigProcState newProcessingState );


#endif // SENSORS_H
