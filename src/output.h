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


/// @file output.h
/// @brief Generic output (signal writing/generation) functions
///
/// Interface for configurable output writing (as shown in @ref motor_config)


#ifndef OUTPUT_H
#define OUTPUT_H


#include "data_io/interface/data_io.h" 

#include <stdbool.h>

typedef struct _OutputData OutputData;       ///< Single output internal data structure    
typedef OutputData* Output;                  ///< Opaque reference to output internal data structure

                                                              
/// @brief Creates and initializes output data structure based on given information                                              
/// @param[in] configuration reference to data object containing configuration parameters, as explained at @ref motor_config
/// @return reference/pointer to newly created and initialized output data structure
Output Output_Init( DataHandle configuration );

/// @brief Deallocates internal data of given output                        
/// @param[in] output reference to output
void Output_End( Output output );

/// @brief Allows hardware/virtual device of given output to output signal
/// @param[in] output reference to output
/// @return true on enabled output, false otherwise
bool Output_Enable( Output output );
                                                                  
/// @brief Prevents hardware/virtual device of given output from outputing signal
/// @param[in] output reference to output
void Output_Disable( Output output );

/// @brief Calls underlying signal output implementation to check for errors on given output              
/// @param[in] output reference to output
/// @return true on detected error, false otherwise
bool Output_HasError( Output output );

/// @brief Calls underlying signal output implementation to reset possible device errors             
/// @param[in] output reference to output
void Output_Reset( Output output );

/// @brief Writes specified value to given output ouput device                
/// @param[in] output reference to output
/// @param[in] value value to be written/generated
void Output_Update( Output output, double value );


#endif  // OUTPUT_H
