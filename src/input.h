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


/// @file input.h
/// @brief Generic input (measurement reading) functions
///
/// Interface for configurable input reading and state change

#ifndef INPUT_H
#define INPUT_H


#include "signal_processing/signal_processing.h" 
#include "data_io/interface/data_io.h" 

#include <stdbool.h>


typedef struct _InputData InputData;    ///< Single input internal data structure    
typedef InputData* Input;               ///< Opaque reference to input internal data structure

                                                                   
/// @brief Creates and initializes input data structure based on given information                                              
/// @param[in] configuration reference to data object containing configuration parameters, as explained at @ref input_config
/// @return reference/pointer to newly created and initialized input data structure
Input Input_Init( DataHandle configuration );

/// @brief Deallocates internal data of given input                        
/// @param[in] input reference to input
void Input_End( Input input );

/// @brief Performs single reading and processing of signal measured by given input
/// @param[in] input reference to input
/// @return current value of processed signal (0.0 on erros)
double Input_Update( Input input );

/// @brief Calls underlying signal reading implementation (plugin) to check for errors on given input              
/// @param[in] input reference to input
/// @return true on detected error, false otherwise
bool Input_HasError( Input input );

/// @brief Resets signal processing state and possible input device errors      
/// @param[in] input reference to input
void Input_Reset( Input input );

/// @brief Sets current processing phase/state/mode of given input                     
/// @param[in] input reference to input
/// @param[in] newProcessingState desired signal processing phase
void Input_SetState( Input input, enum SigProcState newProcessingState );


#endif // INPUT_H
 
