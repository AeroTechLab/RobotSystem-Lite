////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2020 Leonardo Consoni <leonardojc@protonmail.com>      //
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

const double MAX_WAVE_IMPEDANCE = 10.0;
const double MIN_WAVE_IMPEDANCE_FACTOR = 0.1;

const double MAX_WAVE_BANDWIDTH = 0.2;
const double MIN_WAVE_BANDWIDTH_FACTOR = 0.1;
const double MAX_WAVE_BANDWIDTH_FACTOR = 1.0;

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle1", "angle2" };

static struct
{
  double wavesTable[ DOFS_NUMBER ][ DELAY_SETPOINTS_NUMBER ];
  double inputPositionsTable[ DOFS_NUMBER ][ DELAY_SETPOINTS_NUMBER ];
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

double FilterWave( double inputWave, double* ref_lastInputWave, double* ref_lastFilteredWave, double bandwidth )
{
  double lastInputWave = (*ref_lastInputWave);
  double lastFilteredWave = (*ref_lastFilteredWave);
	double filteredWave = ( ( 2 - bandwidth ) * lastFilteredWave + bandwidth * ( inputWave + lastInputWave ) ) / ( 2 + bandwidth );
	(*ref_lastInputWave) = inputWave;
  (*ref_lastFilteredWave) = filteredWave;
  return filteredWave;
}

double CorrectWave( double inputWave, double waveImpedance, double inputPosition, double currentPosition, double bandwidth )
{
  double positionError = inputPosition - currentPosition;
  double waveCorrection = sqrt( 2.0 * waveImpedance ) * bandwidth * positionError;
  if( positionError * inputWave < 0 ) waveCorrection = 0.0;
  else if( fabs( waveCorrection ) > fabs( inputWave ) ) waveCorrection = -inputWave;
  inputWave += waveCorrection;
  
  return inputWave;
}

double ExtractForce( double inputWave, double waveImpedance, double inputVelocity )
{
  double outputForce = -( waveImpedance * inputVelocity - sqrt( 2.0 * waveImpedance ) * inputWave );
  
  return outputForce;
}

double ExtractVelocity( double inputWave, double waveImpedance, double inputForce )
{
  double outputVelocity = ( sqrt( 2.0 * waveImpedance ) * inputWave + inputForce ) / waveImpedance;
  
  return outputVelocity;
}

double BuildWave( double waveImpedance, double velocity, double force )
{
  double outputWave = ( waveImpedance * velocity - force ) / sqrt( 2.0 * waveImpedance );
  
  return outputWave;
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
  ref_jointSetpoints->stiffness = ref_axisSetpoints->stiffness;                         
  ref_jointSetpoints->damping = ref_axisSetpoints->damping;                             
  
  //fprintf( stderr, "position=%.5f, setpoint=%.5f, control force=%.5f\n", ref_jointMeasures->position, ref_jointSetpoints->position, ref_jointSetpoints->force );
}

void RunControlStep( DoFVariables** jointMeasuresList, DoFVariables** axisMeasuresList, DoFVariables** jointSetpointsList, DoFVariables** axisSetpointsList, double timeDelta )
{
  size_t setpointIndex = controlData.setpointCount % DELAY_SETPOINTS_NUMBER;
  
  if( axisSetpointsList[ 0 ]->stiffness < MIN_WAVE_BANDWIDTH_FACTOR ) axisSetpointsList[ 0 ]->stiffness = MIN_WAVE_BANDWIDTH_FACTOR;
  if( axisSetpointsList[ 0 ]->stiffness > MAX_WAVE_BANDWIDTH_FACTOR ) axisSetpointsList[ 0 ]->stiffness = MAX_WAVE_BANDWIDTH_FACTOR;
  if( axisSetpointsList[ 0 ]->damping < MIN_WAVE_IMPEDANCE_FACTOR ) axisSetpointsList[ 0 ]->damping = MIN_WAVE_IMPEDANCE_FACTOR;
  axisSetpointsList[ 1 ]->stiffness = axisSetpointsList[ 0 ]->stiffness = MIN_WAVE_BANDWIDTH_FACTOR;
  axisSetpointsList[ 1 ]->damping = axisSetpointsList[ 0 ]->damping = MIN_WAVE_IMPEDANCE_FACTOR;
  jointMeasuresList[ 1 ]->stiffness = jointMeasuresList[ 0 ]->stiffness = MIN_WAVE_BANDWIDTH_FACTOR;
  jointMeasuresList[ 1 ]->damping = jointMeasuresList[ 0 ]->damping = MIN_WAVE_IMPEDANCE_FACTOR;
  
  double waveBandwidth = MAX_WAVE_BANDWIDTH * axisSetpointsList[ 0 ]->stiffness;
  double waveImpedance = MAX_WAVE_IMPEDANCE * axisSetpointsList[ 0 ]->damping;
  
  double wave_0 = FilterWave( controlData.wavesTable[ 0 ][ setpointIndex ], 
                              &(controlData.lastInputWavesList[ 0 ]), &(controlData.lastFilteredWavesList[ 0 ]), waveBandwidth );

  double wave_1 = FilterWave( controlData.wavesTable[ 1 ][ setpointIndex ], 
                              &(controlData.lastInputWavesList[ 1 ]), &(controlData.lastFilteredWavesList[ 1 ]), waveBandwidth );
  
  wave_0 = CorrectWave( wave_0, waveImpedance, controlData.inputPositionsTable[ 0 ][ setpointIndex ], jointMeasuresList[ 0 ]->position, waveBandwidth );
  axisSetpointsList[ 0 ]->force = ExtractForce( wave_0, waveImpedance, jointMeasuresList[ 0 ]->velocity );
  ControlJoint( jointMeasuresList[ 0 ], axisMeasuresList[ 0 ], jointSetpointsList[ 0 ], axisSetpointsList[ 0 ] );
  
  controlData.wavesTable[ 1 ][ setpointIndex ] = BuildWave( waveImpedance, jointMeasuresList[ 0 ]->velocity, axisSetpointsList[ 0 ]->force );
  controlData.inputPositionsTable[ 1 ][ setpointIndex ] = axisMeasuresList[ 0 ]->position;
  
  //if( controlData.state == ROBOT_PREPROCESSING )
  //{
  //  Log_EnterNewLine( controlData.samplingLog, controlData.elapsedTime );
  //  Log_RegisterValues( controlData.samplingLog, 4, jointMeasuresList[ 0 ]->force, jointMeasuresList[ 0 ]->position, jointMeasuresList[ 0 ]->velocity, jointMeasuresList[ 0 ]->acceleration );
  //}
  
  wave_1 = CorrectWave( wave_1, waveImpedance, controlData.inputPositionsTable[ 1 ][ setpointIndex ], jointMeasuresList[ 1 ]->position, waveBandwidth );
  axisSetpointsList[ 1 ]->force = ExtractForce( wave_1, waveImpedance, jointMeasuresList[ 1 ]->velocity );
  ControlJoint( jointMeasuresList[ 1 ], axisMeasuresList[ 1 ], jointSetpointsList[ 1 ], axisSetpointsList[ 1 ] );
  
  controlData.wavesTable[ 0 ][ setpointIndex ] = BuildWave( waveImpedance, jointMeasuresList[ 1 ]->velocity, axisSetpointsList[ 1 ]->force );
  controlData.inputPositionsTable[ 0 ][ setpointIndex ] = axisMeasuresList[ 1 ]->position;
  
  controlData.setpointCount++;
  controlData.elapsedTime += timeDelta;
  
  //if( controlData.state != ROBOT_OPERATION && controlData.state != ROBOT_PREPROCESSING ) jointSetpointsList[ 0 ]->force = jointSetpointsList[ 1 ]->force = 0.0;
}
