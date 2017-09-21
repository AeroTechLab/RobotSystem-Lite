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


#ifndef SHARED_ROBOT_CONTROL_H
#define SHARED_ROBOT_CONTROL_H

enum { ROBOT_CMD_DISABLE = 1, ROBOT_ST_DISABLED = ROBOT_CMD_DISABLE, 
       ROBOT_CMD_ENABLE, ROBOT_ST_ENABLED = ROBOT_CMD_ENABLE, 
       ROBOT_CMD_RESET, ROBOT_ST_ERROR = ROBOT_CMD_RESET,
       ROBOT_CMD_PASSIVATE, ROBOT_ST_PASSIVE = ROBOT_CMD_PASSIVATE,
       ROBOT_CMD_OPERATE, ROBOT_ST_OPERATING = ROBOT_CMD_OPERATE, 
       ROBOT_CMD_OFFSET, ROBOT_ST_OFFSETTING = ROBOT_CMD_OFFSET,
       ROBOT_CMD_CALIBRATE, ROBOT_ST_CALIBRATING = ROBOT_CMD_CALIBRATE, 
       ROBOT_CMD_PREPROCESS, ROBOT_ST_PREPROCESSING = ROBOT_CMD_PREPROCESS, 
       ROBOT_CMD_SET_USER, ROBOT_CMD_SET_CONFIG,
       ROBOT_BYTES_NUMBER };

#endif // SHARED_JOINT_CONTROL_H
