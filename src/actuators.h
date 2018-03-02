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


/// @file actuators.h
/// @brief Generic actuator control (measurement + actuation) functions
///
/// Interface for configurable actuator control. Available configuration options are detailed in @ref actuator_config.
/// Every actuator contains a single motor and a set of sensors combined with a [Kalman filter]() for motion measurements (see @ref configuration_levels). 

/// @page actuator_config Actuator Configuration
/// The actuator-level configuration (see @ref configuration_levels) is read using the [data I/O interface](https://bitiquinho.github.io/Platform-Utils/structDataIO.html). Configuration of sensors and motors listed is loaded recursively (as in @ref sensor_config and @ref motor_config)
///
/// Any configuration path must be provided without file extension, and relative to CONFIG_DIR/actuators/, where CONFIG_DIR is the [defined base data path](https://bitiquinho.github.io/Platform-Utils/classDATA__IO__INTERFACE.html)
///
/// The possible configuration fields and their values are here exemplified for the case of current JSON file I/O implementation:
/// @code
/// {
///   "sensors": [                        // List of sensors available for configured actuator
///     { 
///       "input_variable": "POSITION",       // Dimension measured by sensor (POSITION, VELOCITY, FORCE or ACCELERATION)
///       "config": "<sensor_identifier>",    // Sensor string identifier, used for searching its configuration file
///       "deviation": 1.0                    // Measurement error (standard deviation) associated with the sensor
///     },
///     { 
///       "input_variable": "FORCE",
///       "config": { /*...*/ }               // Inline sensor configuration (data object)
///     }
///   ],
///   "motor": {                          // Actuation motor used on configured actuator
///     "output_variable": "VELOCITY",      // Controlled dimension/variable (POSITION, VELOCITY, FORCE or ACCELERATION)
///     "config": "<motor_identifier>"      // Motor string identifier (configuration file path) or inline configuration object 
///   }
/// }
/// @endcode


#ifndef ACTUATOR_H
#define ACTUATOR_H 

#include "data_io.h"

/// Defined possible control states enumeration. Passed to internal motor and sensors
enum ActuatorState 
{ 
  ACTUATOR_OFFSET,             ///< State of reference (zero) taking for controller measurements 
  ACTUATOR_CALIBRATION,        ///< State of limits (min-max) taking for controller measurements 
  ACTUATOR_OPERATION,          ///< State of normal controller operation 
  ACTUATOR_STATES_NUMBER       ///< Total number of control states 
};

/// Control used variables list indexes enumeration
typedef struct ActuatorVariables
{
  double position, velocity, force, acceleration;
}
ActuatorVariables;

typedef struct _ActuatorData ActuatorData;      ///< Single actuator internal data structure    
typedef ActuatorData* Actuator;                 ///< Opaque reference to actuator internal data structure

                                                                  
/// @brief Creates and initializes actuator data structure based on given information                                              
/// @param[in] configuration reference to data object containing configuration parameters, as explained at @ref actuator_config
/// @return reference/pointer to newly created and initialized actuator data structure
Actuator Actuator_Init( DataHandle configuration );

/// @brief Deallocates internal data of given actuator                        
/// @param[in] actuator reference to actuator
void Actuator_End( Actuator actuator );

/// @brief Allows motor output on given actuator 
/// @param[in] actuator reference to actuator
void Actuator_Enable( Actuator actuator );
                                                                  
/// @brief Prevents motor output on given actuator 
/// @param[in] actuator reference to actuator
void Actuator_Disable( Actuator actuator );
                                                                
/// @brief Verifies if motor output on given actuator is enabled
/// @param[in] actuator reference to actuator
/// @return true on enabled output, false otherwise
bool Actuator_IsEnabled( Actuator actuator );

/// @brief Calls underlying motor and sensors implementations (plugins) to check for errors on given actuator              
/// @param[in] actuator reference to actuator
/// @return true on error detected, false otherwise
bool Actuator_HasError( Actuator actuator );

/// @brief Calls underlying sensors and motor implementations (plugins) to clear possible device errors             
/// @param[in] actuator reference to actuator
void Actuator_Reset( Actuator actuator );

/// @brief Calls underlying sensors implementations (plugins) to change measurement state          
/// @param[in] actuator reference to actuator
/// @param[in] controlState new control state to be set
/// @return true if control state was changed, false otherwise
bool Actuator_SetControlState( Actuator actuator, enum ActuatorState controlState );

/// @brief Reads sensors of given actuator               
/// @param[in] actuator reference to actuator
/// @param[out] ref_measures pointer to variables structure where values will be stored
/// @param[in] timeDelta time passed between measurements for sensor filtering/fusion state prediction
/// @return true if new measurements were taken, false otherwise
bool Actuator_GetMeasures( Actuator actuator, ActuatorVariables* ref_measures, double timeDelta );

/// @brief Writes possible motor setpoint values for given actuator       
/// @param[in] actuator reference to actuator
/// @param[in] ref_setpoints pointer/reference to variables structure with the new setpoints
/// @return control action applied on motor of given actuator (control variable specified in @ref actuator_config)
double Actuator_SetSetpoints( Actuator actuator, ActuatorVariables* ref_setpoints );


#endif // ACTUATOR_H
