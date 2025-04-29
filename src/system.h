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


/// @file system.h
/// @brief Main application execution functions
///
/// Interface for calling RobotSystem initialization, update and shutdown from the "main" entry-point of different operating systems


#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>

/// @brief Initialize RobotSystem with list of command-line string arguments                                             
/// @param[in] argc number of string arguments, at least 2: application name and robot config path (see @ref robot_config) or help flag
/// @param[in] argv pointer/vector of string arguments, the application name followed by other [command-line options]()
/// @return true if RobotSystem was initialized successfully, false otherwise
bool System_Init( const int argc, const char** argv );

/// @brief End RobotSystem execution, freeing data structures and closing connecions
void System_End( void );

/// @brief Call RobotSystem update step
void System_Update( void );


#endif // SYSTEM_H
