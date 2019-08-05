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

#include <string.h>
#include <math.h>

#include "robot_control/robot_control.h"

#include "debug/data_logging.h"

#define DOFS_NUMBER 2
#define DELAY_SETPOINTS_NUMBER 1//5

const double MIN_WAVE_IMPEDANCE = 1.0;

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle1", "angle2" };

static struct
{
  double setpointsTable[ DOFS_NUMBER ][ DELAY_SETPOINTS_NUMBER ];
  double lastInputWavesList[ DOFS_NUMBER ];
  double lastFilteredWavesList[ DOFS_NUMBER ];
  size_t setpointCount;
  enum ControlState state;
  double elapsedTime;
  Log samplingLog;
}
controlData;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


bool InitController( const char* configurationString )
{
  memset( &controlData, 0, sizeof(controlData) );
  
  Log_SetDirectory( "" );
  controlData.samplingLog = Log_Init( "motor_sampling", 8 );
  
  controlData.elapsedTime = 0.0;
  
  controlData.state = CONTROL_PASSIVE;
  
  return true;
}

void EndController()
{
  Log_End( controlData.samplingLog );
}

size_t GetJointsNumber()
{
  return DOFS_NUMBER;
}

const char** GetJointNamesList()
{
  return (const char**) DOF_NAMES;
}

size_t GetAxesNumber()
{
  return DOFS_NUMBER;
}

const char** GetAxisNamesList()
{
  return (const char**) DOF_NAMES;
}

size_t GetExtraInputsNumber( void ) { return 0; }
      
void SetExtraInputsList( double* inputsList ) { }

size_t GetExtraOutputsNumber( void ) { return 0; }
         
void GetExtraOutputsList( double* outputsList ) { }

void SetControlState( enum ControlState newControlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", newControlState );
  
  if( newControlState == CONTROL_PREPROCESSING ) controlData.elapsedTime = 0.0;
  
  controlData.state = newControlState;
}

void ControlJoint( DoFVariables* ref_jointMeasures, DoFVariables* ref_axisMeasures, DoFVariables* ref_jointSetpoints, DoFVariables* ref_axisSetpoints )
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
  //ref_jointSetpoints->force = 1.0 * positionError + 0.2 * velocityError; 
  
  //fprintf( stderr, "position=%.5f, setpoint=%.5f, control force=%.5f\n", ref_jointMeasures->position, ref_jointSetpoints->position, ref_jointSetpoints->force );
}

void RunControlStep( DoFVariables** jointMeasuresList, DoFVariables** axisMeasuresList, DoFVariables** jointSetpointsList, DoFVariables** axisSetpointsList, double timeDelta )
{
  axisSetpointsList[ 0 ]->position = 0.0;//jointMeasuresList[ 1 ]->position;
  axisSetpointsList[ 1 ]->position = 0.0;//jointMeasuresList[ 0 ]->position;
  axisSetpointsList[ 0 ]->velocity = 0.0;//jointMeasuresList[ 1 ]->velocity;
  axisSetpointsList[ 1 ]->velocity = 0.0;//jointMeasuresList[ 0 ]->velocity;
  
  ControlJoint( jointMeasuresList[ 0 ], axisMeasuresList[ 0 ], jointSetpointsList[ 0 ], axisSetpointsList[ 0 ] );
  ControlJoint( jointMeasuresList[ 1 ], axisMeasuresList[ 1 ], jointSetpointsList[ 1 ], axisSetpointsList[ 1 ] );
  
  fprintf( stderr, "position 1=%.5f, position 2=%.5f\n", axisMeasuresList[ 0 ]->position, axisMeasuresList[ 1 ]->position );
  
  controlData.elapsedTime += timeDelta;
  
  //if( controlData.state != ROBOT_OPERATION && controlData.state != ROBOT_PREPROCESSING ) jointSetpointsList[ 0 ]->force = jointSetpointsList[ 1 ]->force = 0.0;
}
