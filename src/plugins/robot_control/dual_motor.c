////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2018 Leonardo Consoni <consoni_2519@hotmail.com>       //
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
#define DELAY_SETPOINTS_NUMBER 5

const double MIN_WAVE_IMPEDANCE = 1.0;

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle1", "angle2" };

static struct
{
  double setpointsTable[ DOFS_NUMBER ][ DELAY_SETPOINTS_NUMBER ];
  double lastInputWavesList[ DOFS_NUMBER ];
  double lastFilteredWavesList[ DOFS_NUMBER ];
  size_t setpointCount;
  enum RobotState state;
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
  
  controlData.state = ROBOT_PASSIVE;
  
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

void SetControlState( enum RobotState newControlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", newControlState );
  
  if( newControlState == ROBOT_PREPROCESSING ) controlData.elapsedTime = 0.0;
  
  controlData.state = newControlState;
}

double FilterWave( double inputWave, double* ref_lastInputWave, double* ref_lastFilteredWave, double filterStrength, double delayTimeDelta )
{
  delayTimeDelta = delayTimeDelta * filterStrength / 100.0;
  double lastInputWave = (*ref_lastInputWave);
  double lastFilteredWave = (*ref_lastFilteredWave);
	double filteredWave = ( ( 2 - delayTimeDelta ) * lastFilteredWave + delayTimeDelta * ( inputWave + lastInputWave ) ) / ( 2 + delayTimeDelta );
	(*ref_lastInputWave) = inputWave;
  (*ref_lastFilteredWave) = filteredWave;
  return filteredWave;
}

double ProcessWave( double inputWave, RobotVariables* ref_jointMeasures, RobotVariables* ref_axisSetpoints, double timeDelta )
{
  double waveImpedance = ref_axisSetpoints->damping;
  
  double outputForce = -ref_jointMeasures->force;
  ref_axisSetpoints->position += ref_axisSetpoints->velocity * timeDelta / 2.0;
  ref_axisSetpoints->velocity = ( sqrt( 2.0 * waveImpedance ) * inputWave - outputForce ) / waveImpedance;
  ref_axisSetpoints->position += ref_axisSetpoints->velocity * timeDelta / 2.0;
  //ref_axisSetpoints->position *= ref_axisSetpoints->stiffness / 100.0;
  
  double outputWave = ( waveImpedance * ref_axisSetpoints->velocity - outputForce ) / sqrt( 2.0 * waveImpedance );
  
  //fprintf( stderr, "ui=%.5f, fo=%.5f, vi=%.5f, uo=%.5f, po=%.3f\r", inputWave, outputForce, ref_axisSetpoints->velocity, outputWave, ref_axisSetpoints->position );
  
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
  
  //fprintf( stderr, "position=%.5f, setpoint=%.5f, control force=%.5f\n", ref_jointMeasures->position, ref_jointSetpoints->position, ref_jointSetpoints->force );
}

void RunControlStep( RobotVariables** jointMeasuresList, RobotVariables** axisMeasuresList, RobotVariables** jointSetpointsList, RobotVariables** axisSetpointsList, double timeDelta )
{
  size_t currentSetpointIndex = controlData.setpointCount % DELAY_SETPOINTS_NUMBER;
  //double setpoint_0 = controlData.setpointsTable[ 0 ][ currentSetpointIndex ];
  double setpoint_0 = FilterWave( controlData.setpointsTable[ 0 ][ currentSetpointIndex ], 
                                  &(controlData.lastInputWavesList[ 0 ]), &(controlData.lastFilteredWavesList[ 0 ]),
                                  axisSetpointsList[ 0 ]->stiffness, timeDelta * DELAY_SETPOINTS_NUMBER );
  //double setpoint_1 = controlData.setpointsTable[ 1 ][ currentSetpointIndex ];
  double setpoint_1 = FilterWave( controlData.setpointsTable[ 1 ][ currentSetpointIndex ], 
                                  &(controlData.lastInputWavesList[ 1 ]), &(controlData.lastFilteredWavesList[ 1 ]),
                                  axisSetpointsList[ 1 ]->stiffness, timeDelta * DELAY_SETPOINTS_NUMBER );
  
  if( axisSetpointsList[ 0 ]->damping < MIN_WAVE_IMPEDANCE ) axisSetpointsList[ 0 ]->damping = MIN_WAVE_IMPEDANCE;
  axisSetpointsList[ 1 ]->stiffness = axisSetpointsList[ 0 ]->stiffness;
  axisSetpointsList[ 1 ]->damping = axisSetpointsList[ 0 ]->damping;
  
  controlData.setpointsTable[ 1 ][ currentSetpointIndex ] = ProcessWave( setpoint_0, jointMeasuresList[ 0 ], axisSetpointsList[ 0 ], timeDelta );
  ControlJoint( jointMeasuresList[ 0 ], axisMeasuresList[ 0 ], jointSetpointsList[ 0 ], axisSetpointsList[ 0 ], timeDelta );
  
  //if( controlData.state == ROBOT_PREPROCESSING )
  //{
  //  Log_EnterNewLine( controlData.samplingLog, controlData.elapsedTime );
  //  Log_RegisterValues( controlData.samplingLog, 4, jointMeasuresList[ 0 ]->force, jointMeasuresList[ 0 ]->position, jointMeasuresList[ 0 ]->velocity, jointMeasuresList[ 0 ]->acceleration );
  //}
  
  controlData.setpointsTable[ 0 ][ currentSetpointIndex ] = ProcessWave( setpoint_1, jointMeasuresList[ 1 ], axisSetpointsList[ 1 ], timeDelta );
  ControlJoint( jointMeasuresList[ 1 ], axisMeasuresList[ 1 ], jointSetpointsList[ 1 ], axisSetpointsList[ 1 ], timeDelta );
  
  controlData.setpointCount++;
  controlData.elapsedTime += timeDelta;
  
  //if( controlData.state != ROBOT_OPERATION && controlData.state != ROBOT_PREPROCESSING ) jointSetpointsList[ 0 ]->force = jointSetpointsList[ 1 ]->force = 0.0;
}
