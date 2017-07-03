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


/// @file actuators.h
/// @brief Generic actuator (measuring + control) functions
///
/// Interface for configurable actuator control. Specific underlying implementation and further configuration are defined as explained in @ref actuator_config

/// @page actuator_config Actuator Configuration
/// The actuator-level configuration (see @ref configuration_levels) is read using the [data I/O implementation currently defined](https://bitiquinho.github.io/Platform-Utils/structDataIO.html). Configuration of sensors and motors listed is loaded recursively (as in @ref sensor_config and @ref motor_config)
///
/// Any configuration file/location path must be provided without its format extension, and relative to CONFIG_DIR/actuators/, where CONFIG_DIR is the [defined base data path](https://bitiquinho.github.io/Platform-Utils/classDATA__IO__INTERFACE.html)
///
/// The possible configuration fields and their values are here exemplified for the case of a JSON format configuration:
/// @code
/// {
///   "sensors": [                        // List of sensors available for configured actuator
///     { 
///       "input_variable": "POSITION",       // Dimension measured by sensor (POSITION, VELOCITY, FORCE or ACCELERATION)
///       "config": "<sensor_identifier>"     // Sensor string identifier, used for searching its configuration file
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

#include "actuator_control_interface.h"

#include "data_io.h"

/// Defined possible control states enumeration. Passed to generic or plugin specific robot control implementations
enum ActuatorState 
{ 
  ACTUATOR_OFFSET,             ///< State for definition of reference (zero) for controller measurements 
  ACTUATOR_CALIBRATION,        ///< State for definition of limits (min-max) for controller measurements 
  ACTUATOR_OPERATION,          ///< State for normal controller operation 
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

/// @brief Calls underlying sensors and motor implementations to check for errors on given actuator              
/// @param[in] actuator reference to actuator
/// @return true on detected error, false otherwise
bool Actuator_HasError( Actuator actuator );

/// @brief Calls underlying sensors and motor implementations to reset possible device errors             
/// @param[in] actuator reference to actuator
void Actuator_Reset( Actuator actuator );

/// @brief Calls underlying sensors implementations to change measurement/control state          
/// @param[in] actuator reference to actuator
/// @param[in] controlState new control state to be set
/// @return true on if control state was changed, false otherwise
bool Actuator_SetControlState( Actuator actuator, enum ActuatorState controlState );

/// @brief Reads sensors of given actuator               
/// @param[in] actuator reference to actuator
/// @param[out] ref_measures pointer to array big enough to hold the ControlVariable list of measures (NULL if not needed)
/// @param[in] timeDelta desired time interval reference for control
/// @return pointer to ControlVariable array of measured values (measuredBuffer or internal one, if NULL)
ActuatorVariables* Actuator_GetMeasures( Actuator actuator, ActuatorVariables* ref_measures, double timeDelta );

/// @brief Calls underlying actuator control (plugin) implementation with provided data, for given actuator       
/// @param[in] actuator reference to actuator
/// @param[in] ref_setpoints pointer to ControlVariable array of actuator setpoints
/// @return control action applied on motor of given actuator (dimension specified in @ref actuator_config)
double Actuator_SetSetpoints( Actuator actuator, ActuatorVariables* ref_setpoints );


#endif // ACTUATOR_H
