//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
//  Original work Copyright (c) 2014 Wilian dos Santos                                  //
//  Modified work Copyright (c) 2016-2019 Leonardo Consoni <consoni_2519@hotmail.com>   //
//                                                                                      //
//  This file is part of RobotSystem-Lite.                                              //
//                                                                                      //
//  RobotSystem-Lite is free software: you can redistribute it and/or modify            //
//  it under the terms of the GNU Lesser General Public License as published            //
//  by the Free Software Foundation, either version 3 of the License, or                //
//  (at your option) any later version.                                                 //
//                                                                                      //
//  RobotSystem-Lite is distributed in the hope that it will be useful,                 //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of                      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                        //
//  GNU Lesser General Public License for more details.                                 //
//                                                                                      //
//  You should have received a copy of the GNU Lesser General Public License            //
//  along with RobotSystem-Lite. If not, see <http://www.gnu.org/licenses/>.            //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////


#include "robot_control/robot_control.h"

#include <stdlib.h>
#include <math.h>
#include <string.h> 
#include <stdio.h>


typedef struct _ControlData
{
  double velocitySetpoint;
  double previousForceError;
  enum RobotState currentControlState
}
ControlData;

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


#define DOFS_NUMBER 1
const char* JOINT_NAMES[ DOFS_NUMBER ] = { "dof_joint" };
const char* AXIS_NAMES[ DOFS_NUMBER ] = { "dof_axis" };


RobotController InitController( const char* configurationString )
{
  ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
   
  return (RobotController) newController;
}

void EndController( RobotController ref_controller )
{
  if( ref_controller == NULL ) return;
  
  free( (void*) ref_controller );
}

size_t GetJointsNumber( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  return DOFS_NUMBER;
}

const char** GetJointNamesList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  return JOINT_NAMES;
}

size_t GetAxesNumber( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  return DOFS_NUMBER;   
}

const char** GetAxisNamesList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  return AXIS_NAMES;
}

void SetControlState( RobotController ref_controller, enum RobotState newControlState )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  if( newControlState == ROBOT_PASSIVE )
  {
    // ...
  }
  else if( newControlState == ROBOT_OFFSET )
  {
    // ...
  }
  else if( newControlState == ROBOT_CALIBRATION )
  {
    // ...
  }
  else if( newControlState == ROBOT_PREPROCESSING )
  {
    // ...
  }
  else if( newControlState == ROBOT_OPERATION )
  {
    // ...
  }
  
  controller->currentControlState = newControlState;
}

void RunControlStep( RobotController ref_controller, RobotVariables** jointMeasuresTable, RobotVariables** axisMeasuresTable, RobotVariables** jointSetpointsTable, RobotVariables** axisSetpointsTable, double timeDelta )
{
  const double K_P = 370;//1.6527; // 370 * ( F_in_max / V_out_max )
  const double K_I = 3.5;//0.0156; // 3.5 * ( F_in_max / V_out_max )
  
  ControlData* controller = (ControlData*) ref_controller;

  axisMeasuresTable[ 0 ]->position = jointMeasuresTable[ 0 ]->position * 180.0 / M_PI - 90.0;
  axisMeasuresTable[ 0 ]->velocity = jointMeasuresTable[ 0 ]->velocity * 180.0 / M_PI;
  axisMeasuresTable[ 0 ]->acceleration = jointMeasuresTable[ 0 ]->acceleration * 180.0 / M_PI;
  axisMeasuresTable[ 0 ]->force = jointMeasuresTable[ 0 ]->force;
    
  // Simple PD impedance control example
  //jointSetpointsTable[ 0 ]->position = axisSetpointsTable[ 0 ]->position * M_PI / 180.0 + M_PI / 2.0;
  //jointSetpointsTable[ 0 ]->velocity = axisSetpointsTable[ 0 ]->velocity * M_PI / 180.0;
  //double positionError = jointSetpointsTable[ 0 ]->position - jointMeasuresTable[ 0 ]->position;    // e_p = x_d - x
  //double velocityError = jointSetpointsTable[ 0 ]->velocity - jointMeasuresTable[ 0 ]->velocity;    // e_v = xdot_d - xdot
  // F_actuator = K * e_p + B * e_v - D * x_dot
  //jointSetpointsTable[ 0 ]->force = jointSetpointsTable[ 0 ]->stiffness * positionError - jointSetpointsTable[ 0 ]->damping * velocityError;
  
  //jointSetpointsTable[ 0 ]->force = // Your control logic implementation

  // Simple PI torque/velocity discrete control
  double forceError = jointSetpointsTable[ 0 ]->force - jointMeasuresTable[ 0 ]->force;  
  controller->velocitySetpoint += K_P * ( forceError - controller->previousForceError ) + K_I * timeDelta * forceError;
  jointSetpointsTable[ 0 ]->velocity = controller->velocitySetpoint;
  controller->previousForceError = forceError;
}
