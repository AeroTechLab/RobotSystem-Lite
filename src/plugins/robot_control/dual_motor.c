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

#include <string.h>
#include <math.h>

#include "robot_control/robot_control.h"

#include "debug/data_logging.h"

#define DOFS_NUMBER 2
#define DELAY_SETPOINTS_NUMBER 5

const double WAVE_IMPEDANCE = 1.0;

typedef struct _ControllerData
{
  double setpointsTable[ DOFS_NUMBER ][ DELAY_SETPOINTS_NUMBER ];
  size_t setpointCount;
  enum RobotState controlState;
  double elapsedTime;
  Log samplingLog;
}
ControlData;

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle1", "angle2" };


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


RobotController InitController( const char* configurationString )
{
  ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
  
  Log_SetDirectory( "" );
  newController->samplingLog = Log_Init( "motor_sampling", 8 );
  
  newController->elapsedTime = 0.0;
  
  newController->controlState = ROBOT_PASSIVE;
  
  return newController;
}

void EndController( RobotController ref_controller )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
    
  Log_End( controller->samplingLog );
    
  free( ref_controller );
}

size_t GetJointsNumber( RobotController controller )
{
  return DOFS_NUMBER;
}

const char** GetJointNamesList( RobotController ref_controller )
{
  return (const char**) DOF_NAMES;
}

size_t GetAxesNumber( RobotController ref_controller )
{
  return DOFS_NUMBER;
}

const char** GetAxisNamesList( RobotController ref_controller )
{
  return (const char**) DOF_NAMES;
}

void SetControlState( RobotController ref_controller, enum RobotState newControlState )
{
  if( ref_controller == NULL ) return;
  ControlData* controller = (ControlData*) ref_controller;
  
  fprintf( stderr, "Setting robot control phase: %x\n", newControlState );
  
  if( newControlState == ROBOT_PREPROCESSING ) controller->elapsedTime = 0.0;
  
  controller->controlState = newControlState;
}

double ProcessWave( double inputWave, RobotVariables* ref_jointMeasures, RobotVariables* ref_axisSetpoints, double timeDelta )
{
  double outputForce = -ref_jointMeasures->force;
  ref_axisSetpoints->position += ref_axisSetpoints->velocity * timeDelta / 2.0;
  ref_axisSetpoints->velocity = ( sqrt( 2.0 * WAVE_IMPEDANCE ) * inputWave - outputForce ) / WAVE_IMPEDANCE;
  ref_axisSetpoints->position += ref_axisSetpoints->velocity * timeDelta / 2.0;
  ref_axisSetpoints->stiffness = 0.02;
  double outputWave = ( WAVE_IMPEDANCE * ref_axisSetpoints->velocity - outputForce ) / sqrt( 2.0 * WAVE_IMPEDANCE );
  
  return outputWave;
}

void ControlJoint( RobotVariables* ref_jointMeasures, RobotVariables* ref_axisMeasures, RobotVariables* ref_jointSetpoints, RobotVariables* ref_axisSetpoints, double timeDelta )
{
  ref_axisMeasures->acceleration = ref_jointMeasures->acceleration;
  ref_axisMeasures->velocity = ref_jointMeasures->velocity;
  ref_axisMeasures->position = ref_jointMeasures->position;
  ref_axisMeasures->force = ref_jointMeasures->force;
  ref_axisMeasures->stiffness = ref_jointMeasures->stiffness;
  ref_axisMeasures->damping = ref_jointMeasures->damping;
  
  ref_jointSetpoints->velocity = ref_axisSetpoints->velocity;                           // xdot_d
  ref_jointSetpoints->position = ref_axisSetpoints->position;                           // x_d
  ref_jointSetpoints->acceleration = ref_axisSetpoints->acceleration;
  ref_jointSetpoints->force = ref_axisSetpoints->force;
  ref_jointSetpoints->stiffness = ref_axisSetpoints->stiffness;                         // K = lamda^2 * m
  ref_jointSetpoints->damping = ref_axisSetpoints->damping;                             // B = D = lamda * m
  
  double positionError = ref_jointSetpoints->position - ref_jointMeasures->position;    // e_p = x_d - x
  double velocityError = ref_jointSetpoints->velocity - ref_jointMeasures->velocity;    // e_v = xdot_d - xdot
  // F_actuator = K * e_p + B * e_v - D * x_dot
  double controlForce = ref_jointSetpoints->stiffness * positionError - ref_jointSetpoints->damping * velocityError;
  double dampingForce = ref_jointSetpoints->damping * ref_jointMeasures->velocity;
  ref_jointSetpoints->force += controlForce - dampingForce;
  
  fprintf( stderr, "position=%.5f, setpoint=%.5f, control force=%.5f\n", ref_jointMeasures->position, ref_jointSetpoints->position, ref_jointSetpoints->force );
}

void RunControlStep( RobotController ref_controller, RobotVariables** jointMeasuresList, RobotVariables** axisMeasuresList, 
                                                     RobotVariables** jointSetpointsList, RobotVariables** axisSetpointsList, double timeDelta )
{
  if( ref_controller == NULL ) return;
  ControlData* controller = (ControlData*) ref_controller;
  
  size_t currentSetpointIndex = controller->setpointCount % DELAY_SETPOINTS_NUMBER;
  
  controller->setpointsTable[ 1 ][ currentSetpointIndex ] = ProcessWave( controller->setpointsTable[ 0 ][ currentSetpointIndex ], 
                                                                         jointMeasuresList[ 0 ], axisSetpointsList[ 0 ], timeDelta );
  ControlJoint( jointMeasuresList[ 0 ], axisMeasuresList[ 0 ], jointSetpointsList[ 0 ], axisSetpointsList[ 0 ], timeDelta );
  
  //if( controller->controlState == ROBOT_PREPROCESSING )
  //{
  //  Log_EnterNewLine( controller->samplingLog, controller->elapsedTime );
  //  Log_RegisterValues( controller->samplingLog, 4, jointMeasuresList[ 0 ]->force, jointMeasuresList[ 0 ]->position, jointMeasuresList[ 0 ]->velocity, jointMeasuresList[ 0 ]->acceleration );
  //}
  
  controller->setpointsTable[ 0 ][ currentSetpointIndex ] = ProcessWave( controller->setpointsTable[ 1 ][ currentSetpointIndex ], 
                                                                         jointMeasuresList[ 1 ], axisSetpointsList[ 1 ], timeDelta );
  ControlJoint( jointMeasuresList[ 1 ], axisMeasuresList[ 1 ], jointSetpointsList[ 1 ], axisSetpointsList[ 1 ], timeDelta );
  
  controller->setpointCount++;
  controller->elapsedTime += timeDelta;
  
  //if( controller->controlState != ROBOT_OPERATION && controller->controlState != ROBOT_PREPROCESSING ) jointSetpointsList[ 0 ]->force = jointSetpointsList[ 1 ]->force = 0.0;
}
