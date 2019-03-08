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


/// @file shared_dof_variables.h
/// @brief RobotSystem-Lite clients request/receive interface
///
/// Messages transporting online update values for robot DoFs ([axes or joints](https://github.com/EESC-MKGroup/Robot-Control-Interface#the-jointaxis-rationale)) control variables should arrive as quickly as possible, and there is no advantage in resending lost packets, as their validity is short in time. 
/// Thereby, these messages are exchanged with RobotSystem-Lite through lower-latency UDP sockets, on port 50001 for axes and 50002 for joints. 
/// Measurements for both axes and joints go from the main application to its clients, axes setpoints go in the opposite direction. 
/// Messages consist of byte and [single precision floating-point](https://en.wikipedia.org/wiki/Single-precision_floating-point_format) arrays (to prevent string parsing overhead), with data organized like:
///
/// DoFs number | Index 1 | Position | Velocity |  Force  | Acceleration | Inertia | Stiffness | Damping | Index 2 | ...
/// :---------: | :-----: | :------: | :------: | :-----: | :----------: | :-----: | :-------: | :-----: | :-----: | :-:
///    1 byte   | 1 byte  | 4 bytes  | 4 bytes  | 4 bytes |   4 bytes    | 4 bytes |  4 bytes  | 4 bytes | 1 byte  | ...


#ifndef SHARED_DOF_VARIABLES_H
#define SHARED_DOF_VARIABLES_H

/// Enumeration of floating-point values for a single DoF update message
enum RobotDoFVariable { DOF_POSITION, DOF_VELOCITY, DOF_FORCE, DOF_ACCELERATION, DOF_INERTIA, DOF_STIFFNESS, DOF_DAMPING, DOF_FLOATS_NUMBER };

#define DOF_DATA_BLOCK_SIZE DOF_FLOATS_NUMBER * sizeof(float)   ///< Size in bytes of all floating-point values for a single DoF update message

#endif // SHARED_DOF_VARIABLES_H
