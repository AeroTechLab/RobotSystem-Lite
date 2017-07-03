//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
//  Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>             //
//                                                                                  //
//  This file is part of Robot Control Library.                                     //
//                                                                                  //
//  Robot Control Library is free software: you can redistribute it and/or modify   //
//  it under the terms of the GNU Lesser General Public License as published        //
//  by the Free Software Foundation, either version 3 of the License, or            //
//  (at your option) any later version.                                             //
//                                                                                  //
//  Robot Control Library is distributed in the hope that it will be useful,        //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of                  //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                    //
//  GNU Lesser General Public License for more details.                             //
//                                                                                  //
//  You should have received a copy of the GNU Lesser General Public License        //
//  along with Robot Control Library. If not, see <http://www.gnu.org/licenses/>.   //
//                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////


/// @file robot_control_interface.h
/// @brief Generic robot control functions.
///
/// Common robot control interface to be implemented by plugins. Follows definitions by control_definitions.h.

#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159      ///< Defines mathematical Pi value if standard math.h one is not available
#endif

#include "plugin_loader/loader_macros.h"

#define ROBOT_CONTROLLER_INVALID_HANDLE NULL    ///< Reference/pointer to be returned on controller creation failure

typedef void* RobotController;                 ///< Generic/opaque type to reference robot/actuator controllers

/// Defined possible control states enumeration. Passed to generic or plugin specific robot control implementations
enum RobotState 
{ 
  ROBOT_PASSIVE,
  ROBOT_OFFSET,             ///< State for definition of reference (zero) for controller measurements 
  ROBOT_CALIBRATION,        ///< State for definition of limits (min-max) for controller measurements 
  ROBOT_PREPROCESSING,      ///< State for custom automatic preprocessing of controller parameters 
  ROBOT_OPERATION,          ///< State for normal controller operation 
  ROBOT_STATES_NUMBER       ///< Total number of control states 
};

/// Control used variables list indexes enumeration
typedef struct RobotVariables
{
  double position, velocity, force, acceleration, inertia, stiffness, damping;
}
RobotVariables;

/// Robot control interface declaration macro
#define ROBOT_CONTROL_INTERFACE( Interface, INIT_FUNCTION ) \
        INIT_FUNCTION( RobotController, Interface, InitController, const char* ) \
        INIT_FUNCTION( void, Interface, EndController, RobotController ) \
        INIT_FUNCTION( size_t, Interface, GetJointsNumber, RobotController ) \
        INIT_FUNCTION( const char**, Interface, GetJointNamesList, RobotController ) \
        INIT_FUNCTION( const bool*, Interface, GetJointsChangedList, RobotController ) \
        INIT_FUNCTION( size_t, Interface, GetAxesNumber, RobotController ) \
        INIT_FUNCTION( const char**, Interface, GetAxisNamesList, RobotController ) \
        INIT_FUNCTION( const bool*, Interface, GetAxesChangedList, RobotController ) \
        INIT_FUNCTION( void, Interface, SetControlState, RobotController, enum RobotState ) \
        INIT_FUNCTION( void, Interface, RunControlStep, RobotController, RobotVariables**, RobotVariables**, RobotVariables**, RobotVariables**, double )
        
#endif  // ROBOT_CONTROL_INTERFACE_H
    

/// @class ROBOT_CONTROL_INTERFACE 
/// @brief Robot control methods to be implemented by plugins
///           
/// @memberof ROBOT_CONTROL_INTERFACE
/// @fn RobotController InitController( const char* configurationString )                                                                                
/// @brief Calls plugin specific robot controller initialization  
/// @param configurationString string containing the robot/plugin specific configuration
/// @return generic pointer/reference to the created controller  
///           
/// @memberof ROBOT_CONTROL_INTERFACE
/// @fn void EndController( RobotController controller )
/// @brief Calls plugin specific robot controller data deallocation                              
/// @param[in] controller reference to robot controller being terminated 
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn void RunControlStep( RobotController controller, RobotVariables** jointMeasuresList, RobotVariables** axisMeasuresList, RobotVariables** jointSetpointsList, RobotVariables** axisSetpointsList, double timeDelta )                                                                        
/// @brief Calls plugin specific logic to process single control pass and joints/axes coordinate conversions (see @ref joint_axis_rationale) 
/// @param[in] controller reference to robot controller being updated
/// @param[in,out] jointMeasuresList list of per degree-of-freedom control variables representing current robot joints measures                                    
/// @param[in,out] axisMeasuresList list of per degree-of-freedom control variables representing current robot effector measures                                             
/// @param[in,out] jointSetpointsList list of per degree-of-freedom control variables representing robot joints desired states
/// @param[in,out] axisSetpointsList list of per degree-of-freedom control variables representing robot effector desired states
/// @param[in] timeDelta time (in seconds) since the last control pass was called
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn void SetControlState( RobotController controller, enum RobotState controlState )
/// @brief Pass control state to trigger possible plugin specific behaviour
/// @param[in] controller reference to robot controller being updated
/// @param[in] controlState member of state enumeration defined in control_definitions.h
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn size_t GetJointsNumber( RobotController controller )
/// @brief Get number of joint coordinates/degrees-of-freedom for given robot (see @ref joint_axis_rationale) 
/// @param[in] controller reference to robot corresponding controller
/// @return number of coordinates/degrees-of-freedom
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn char** GetJointNamesList( RobotController controller )
/// @brief Get names of all joints for given robot (see @ref joint_axis_rationale) 
/// @param[in] controller reference to robot corresponding controller
/// @return list of joint name strings
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn size_t GetAxesNumber( RobotController controller )
/// @brief Get number of effector coordinates/degrees-of-freedom for given robot (see @ref joint_axis_rationale) 
/// @param[in] controller reference to robot corresponding controller
/// @return number of coordinates/degrees-of-freedom
///           
/// @memberof ROBOT_CONTROL_INTERFACE        
/// @fn char** GetAxisNamesList( RobotController controller )
/// @brief Get names of all effector axes for given robot (see @ref joint_axis_rationale) 
/// @param[in] controller reference to robot corresponding controller
/// @return list of effector axis name strings
///
/// @memberof ROBOT_CONTROL_INTERFACE
