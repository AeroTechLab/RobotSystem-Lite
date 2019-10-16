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


double velocitySetpoint;
double previousForceError;
enum ControlState currentControlState


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


#define DOFS_NUMBER 1
const char* JOINT_NAMES[ DOFS_NUMBER ] = { "dof_joint" };
const char* AXIS_NAMES[ DOFS_NUMBER ] = { "dof_axis" };


bool InitController( const char* configurationString ) { return true; }

void EndController() { return; }

size_t GetJointsNumber() { return DOFS_NUMBER; }

const char** GetJointNamesList() { return JOINT_NAMES; }

size_t GetAxesNumber() { return DOFS_NUMBER; }

const char** GetAxisNamesList() { return AXIS_NAMES; }

size_t GetExtraInputsNumber() { return 0; }

void SetExtraInputsList( double* extraInputsList ) { return; }

size_t GetExtraOutputsNumber() { return 0; }

void GetExtraOutputsList( double* extraOutputsList ) { return; }

void SetControlState(enum ControlState newControlState )
{  
  if( newControlState == CONTROL_PASSIVE )
  {
    // ...
  }
  else if( newControlState == CONTROL_OFFSET )
  {
    // ...
  }
  else if( newControlState == CONTROL_CALIBRATION )
  {
    // ...
  }
  else if( newControlState == CONTROL_PREPROCESSING )
  {
    // ...
  }
  else if( newControlState == CONTROL_OPERATION )
  {
    // ...
  }
  
  currentControlState = newControlState;
}

void RunControlStep( DoFVariables** jointMeasuresList, DoFVariables** axisMeasuresList, DoFVariables** jointSetpointsList, DoFVariables** axisSetpointsList, double timeDelta )
{
  const double K_P = 370;//1.6527; // 370 * ( F_in_max / V_out_max )
  const double K_I = 3.5;//0.0156; // 3.5 * ( F_in_max / V_out_max )

  axisMeasuresList[ 0 ]->position = jointMeasuresList[ 0 ]->position * 180.0 / M_PI - 90.0;
  axisMeasuresList[ 0 ]->velocity = jointMeasuresList[ 0 ]->velocity * 180.0 / M_PI;
  axisMeasuresList[ 0 ]->acceleration = jointMeasuresList[ 0 ]->acceleration * 180.0 / M_PI;
  axisMeasuresList[ 0 ]->force = jointMeasuresList[ 0 ]->force;
    
  // Simple PD impedance control example
  //jointSetpointsList[ 0 ]->position = axisSetpointsList[ 0 ]->position * M_PI / 180.0 + M_PI / 2.0;
  //jointSetpointsList[ 0 ]->velocity = axisSetpointsList[ 0 ]->velocity * M_PI / 180.0;
  //double positionError = jointSetpointsList[ 0 ]->position - jointMeasuresList[ 0 ]->position;    // e_p = x_d - x
  //double velocityError = jointSetpointsList[ 0 ]->velocity - jointMeasuresList[ 0 ]->velocity;    // e_v = xdot_d - xdot
  // F_actuator = K * e_p + B * e_v - D * x_dot
  //jointSetpointsList[ 0 ]->force = jointSetpointsList[ 0 ]->stiffness * positionError - jointSetpointsList[ 0 ]->damping * velocityError;
  
  //jointSetpointsList[ 0 ]->force = // Your control logic implementation

  // Simple PI torque/velocity discrete control
  double forceError = jointSetpointsList[ 0 ]->force - jointMeasuresList[ 0 ]->force;  
  velocitySetpoint += K_P * ( forceError - previousForceError ) + K_I * timeDelta * forceError;
  jointSetpointsList[ 0 ]->velocity = velocitySetpoint;
  previousForceError = forceError;
}
