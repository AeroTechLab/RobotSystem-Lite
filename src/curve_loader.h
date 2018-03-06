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


/// @file sensors.h
/// @brief Generic sensor (measurement reading) functions
///
/// Interface for configurable sensor reading and state change. Specific underlying implementation and further configuration are defined as explained in @ref sensor_config

/// @page sensor_config Sensor Configuration
/// The sensor-level configuration (see @ref configuration_levels) is read using the [data I/O interface](https://bitiquinho.github.io/Platform-Utils/structDataIO.html).
///
/// Any configuration file/location path must be provided without its format extension, and relative to CONFIG_DIR/sensors/, where CONFIG_DIR is the [defined base data path](https://bitiquinho.github.io/Platform-Utils/classDATA__IO__INTERFACE.html)
///
/// The possible configuration fields and their values are here exemplified for the case of a JSON format configuration (mandatory parameters are marked with '*' in their description, remaining optional ones are presented with default values:
/// @code
/// {                                           
///   "segments": [                                               // List of segments that compose the entire curve
///     { 
///       "type": "polynomial",                                   // Segment defined by a polynomial expression
///       "bounds": [ -0.5, -0.0032 ],                            // Limits of segment function domain
///       "parameters": [ -652.05, -0.3701 ]                      // Polynom coefficients (from bigger to lower order)
///     },
///     { 
///       "type": "cubic_spline",                                 // Segment defined by cubic (4 coefficients) spline points
///       "bounds": [ -0.0032, 0.0032 ],                          // Limits of segment function domain
///       "parameters": [ 1.7165, -652.05, -1.5608, -671.77 ]     // Function value and derivative (respectively) on each spline bound
///     }
///   ],
///   "scale_factor": 1.0                                         // Multiply curve value by a factor
/// }
/// @endcode


#ifndef CURVE_LOADER_H
#define CURVE_LOADER_H


#include "curves/curves.h"
#include "data_io.h"

Curve Curve_Load( DataHandle curveData );


#endif // CURVE_LOADER_H
