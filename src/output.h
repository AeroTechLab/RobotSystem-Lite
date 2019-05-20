////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2019 Leonardo Consoni <consoni_2519@hotmail.com>       //
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


/// @file output.h
/// @brief Generic motors (actuation/signal output) functions
///
/// Interface for configurable motor control. Specific underlying implementation (plug-in) and further configuration are defined as explained in @ref motor_config


#ifndef OUTPUT_H
#define OUTPUT_H


#include "data_io/interface/data_io.h" 

#include <stdbool.h>

typedef struct _OutputData OutputData;       ///< Single motor internal data structure    
typedef OutputData* Output;                  ///< Opaque reference to motor internal data structure

                                                              
/// @brief Creates and initializes motor data structure based on given information                                              
/// @param[in] configName name of file containing configuration parameters, as explained at @ref motor_config
/// @return reference/pointer to newly created and initialized motor data structure
Output Output_Init( DataHandle configuration );

/// @brief Deallocates internal data of given motor                        
/// @param[in] motor reference to motor
void Output_End( Output motor );

/// @brief Allows hardware/virtual device of given motor to output signal
/// @param[in] motor reference to motor
/// @return true on enabled output, false otherwise
bool Output_Enable( Output motor );
                                                                  
/// @brief Prevents hardware/virtual device of given motor from outputing signal
/// @param[in] motor reference to motor
void Output_Disable( Output motor );

/// @brief Calls underlying signal output implementation to check for errors on given motor              
/// @param[in] motor reference to motor
/// @return true on detected error, false otherwise
bool Output_HasError( Output motor );

/// @brief Calls underlying signal output implementation to reset possible device errors             
/// @param[in] motor reference to motor
void Output_Reset( Output motor );

/// @brief Writes specified value to given motor ouput device                
/// @param[in] motor reference to motor
/// @param[in] setpoint value to be written/generated
void Output_Update( Output motor, double setpoint );


#endif  // OUTPUT_H
