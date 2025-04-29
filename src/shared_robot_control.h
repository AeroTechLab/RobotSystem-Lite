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


/// @file shared_robot_control.h
/// @brief RobotSystem-Lite clients request/receive interface
///
/// Messages requesting state changes or information about the robot are sent by clients occasionally and their arrival should be as guaranteed as possible. Therefore, these messages are transmitted to the server through TCP sockets, on port 50000.


#ifndef SHARED_ROBOT_CONTROL_H
#define SHARED_ROBOT_CONTROL_H

/// Single byte codes used in request/receive messages for robot state/configuration control
enum RobotControlCode { 
       /// Request information about available robot configurations
       ROBOT_REQ_LIST_CONFIGS = 1,
       /// Reply code for ROBOT_REQ_LIST_CONFIGS. Followed, in the same message, by a JSON-format string like:
       /// @code
       /// { "robots":[ "<available_robot1_name>", "<available_robot2_name>", :"<available_robot3_name>" ] }
       /// @endcode
        ROBOT_REP_CONFIGS_LISTED = ROBOT_REQ_LIST_CONFIGS,
       /// Request information about current robot configuration, its available [axes and joints](https://github.com/AeroTechLab/Robot-Control-Interface#the-jointaxis-rationale))
       ROBOT_REQ_GET_CONFIG,
       /// Reply code for ROBOT_REQ_GET_CONFIG. Followed, in the same message, by a JSON-format string like:
       /// @code
       /// { "id":"<robot_name>", "axes":[ "<axis1_name>", "<axis2_name>" ], "joints":[ "<joint1_name>", "<joint2_name>" ] }
       /// @endcode
       ROBOT_REP_GOT_CONFIG = ROBOT_REQ_GET_CONFIG,
       ROBOT_REQ_SET_CONFIG,                            ///< Request setting new @ref robot_config, reloading all parameters. Must be followed, in the same message, by a string with the new @ref robot_config name
       ROBOT_REP_CONFIG_SET = ROBOT_REQ_SET_CONFIG,     ///< Confirmation reply to ROBOT_REQ_SET_CONFIG. Followed by the same JSON string type as in ROBOT_REP_GOT_CONFIG
       ROBOT_REQ_SET_USER,                              ///< Request setting new user/folder name for [data logging](https://github.com/AeroTechLab/Simple-Data-Logging). Must be followed, in the same message, by a string with the name
       ROBOT_REP_USER_SET = ROBOT_REQ_SET_USER,         ///< Confirmation reply to ROBOT_REQ_SET_USER
       ROBOT_REQ_DISABLE,                               ///< Request turning off the robot and stopping its control thread
       ROBOT_REP_DISABLED = ROBOT_REQ_DISABLE,          ///< Confirmation reply to ROBOT_REQ_DISABLE
       ROBOT_REQ_ENABLE,                                ///< Request turning on the robot and starting its control thread
       ROBOT_REP_ENABLED = ROBOT_REQ_ENABLE,            ///< Confirmation reply to ROBOT_REQ_ENABLE
       ROBOT_REQ_PASSIVATE,                             ///< Request setting robot to a fully compliant control state (passed on to control implementation)
       ROBOT_REP_PASSIVE = ROBOT_REQ_PASSIVATE,         ///< Confirmation reply to ROBOT_REQ_PASSIVATE
       ROBOT_REQ_OFFSET,                                ///< Request setting robot to offset measurement state (passed on to control implementation)
       ROBOT_REP_OFFSETTING = ROBOT_REQ_OFFSET,         ///< Confirmation reply to ROBOT_REQ_OFFSET
       ROBOT_REQ_CALIBRATE,                             ///< Request setting robot to motion range measurement state (passed on to control implementation)
       ROBOT_REP_CALIBRATING = ROBOT_REQ_CALIBRATE,     ///< Confirmation reply to ROBOT_REQ_CALIBRATE
       ROBOT_REQ_OPERATE,                               ///< Request setting robot to normal operation state (passed on to control implementation)
       ROBOT_REP_OPERATING = ROBOT_REQ_OPERATE,         ///< Confirmation reply to ROBOT_REQ_OPERATE
       ROBOT_REQ_PREPROCESS,                            ///< Request setting robot to implementation-specific pre-operation state (passed on to control implementation)
       ROBOT_REP_PREPROCESSING = ROBOT_REQ_PREPROCESS,  ///< Confirmation reply to ROBOT_REQ_PREPROCESS
       ROBOT_REQ_RESET,                                 ///< Clear errors and calibration values for the robot of corresponding index
       ROBOT_REP_ERROR = ROBOT_REQ_RESET                ///< Robot error/failure signal, can come before ROBOT_REQ_RESET
};

#endif // SHARED_ROBOT_CONTROL_H
