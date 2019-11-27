////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2019 Leonardo Consoni <leonardojc@protonmail.com>      //
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


#include "robot_control/robot_control.h"

#include <math.h>
#include <string.h>

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

enum ControlState controlState = CONTROL_PASSIVE;

double positionProportionalGain = 0.0;
double forceProportionalGain = 0.0, forceIntegralGain = 0.0;

double lastForceError = 0.0;
double velocitySetpoint = 0.0;

double runningTime = 0.0;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


bool InitController( const char* configurationString ) 
{
  positionProportionalGain = strtod( strtok( (char*) configurationString, " " ), NULL );
  forceProportionalGain = strtod( strtok( NULL, " " ), NULL );
  forceIntegralGain = strtod( strtok( NULL, " " ), NULL );
  
  return true; 
}

void EndController() 
{ 
  return; 
}

size_t GetJointsNumber() { return DOFS_NUMBER; }

const char** GetJointNamesList() { return DOF_NAMES; }

size_t GetAxesNumber() { return DOFS_NUMBER; }

const char** GetAxisNamesList() { return DOF_NAMES; }

size_t GetExtraInputsNumber( void ) { return 0; }
      
void SetExtraInputsList( double* inputsList ) { return; }

size_t GetExtraOutputsNumber( void ) { return 0; }
         
void GetExtraOutputsList( double* outputsList ) { return; }

void SetControlState( enum ControlState newControlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", newControlState );
  
  controlState = newControlState;
  
  velocitySetpoint = 0.0;
  runningTime = 0.0;
}

void RunControlStep( DoFVariables** jointMeasuresList, DoFVariables** axisMeasuresList, DoFVariables** jointSetpointsList, DoFVariables** axisSetpointsList, double timeDelta )
{
  axisMeasuresList[ 0 ]->position = jointMeasuresList[ 0 ]->position;
  axisMeasuresList[ 0 ]->velocity = jointMeasuresList[ 0 ]->velocity;
  axisMeasuresList[ 0 ]->acceleration = jointMeasuresList[ 0 ]->acceleration;
  axisMeasuresList[ 0 ]->force = jointMeasuresList[ 0 ]->force;
  axisMeasuresList[ 0 ]->stiffness = jointMeasuresList[ 0 ]->stiffness;
  axisMeasuresList[ 0 ]->damping = jointMeasuresList[ 0 ]->damping;
  axisMeasuresList[ 0 ]->inertia = jointMeasuresList[ 0 ]->inertia;
  
  runningTime += timeDelta;

  double totalForceSetpoint = axisSetpointsList[ 0 ]->force;
  
  if( controlState == CONTROL_OPERATION || controlState == CONTROL_CALIBRATION )
  {
    if( controlState == CONTROL_CALIBRATION ) axisSetpointsList[ 0 ]->force = 2 * sin( 2 * M_PI * runningTime / 4 );
  
    double positionError = axisSetpointsList[ 0 ]->position - axisMeasuresList[ 0 ]->position;
    
    if( controlState == CONTROL_OPERATION ) totalForceSetpoint += positionProportionalGain * positionError;
    
    double forceError = totalForceSetpoint - axisMeasuresList[ 0 ]->force;
    velocitySetpoint += forceProportionalGain * ( forceError - lastForceError ) + forceIntegralGain * timeDelta * forceError;
    axisSetpointsList[ 0 ]->velocity = velocitySetpoint;
    lastForceError = forceError;
    
    fprintf( stderr, "pd=%.3f, p=%.3f, fd=%.3f, f=%.3f, k=%.1f, kp=%.1f, ki=%.1f, vd=%.3f\n", axisSetpointsList[ 0 ]->position, axisMeasuresList[ 0 ]->position,
                                                                                              axisSetpointsList[ 0 ]->force, axisMeasuresList[ 0 ]->force, 
                                                                                              positionProportionalGain, forceProportionalGain, forceIntegralGain, velocitySetpoint );
  }
  
  jointSetpointsList[ 0 ]->position = axisSetpointsList[ 0 ]->position;
  jointSetpointsList[ 0 ]->velocity = axisSetpointsList[ 0 ]->velocity;
  jointSetpointsList[ 0 ]->acceleration = axisSetpointsList[ 0 ]->acceleration;
  jointSetpointsList[ 0 ]->force = totalForceSetpoint;
  jointSetpointsList[ 0 ]->stiffness = axisSetpointsList[ 0 ]->stiffness;
  jointSetpointsList[ 0 ]->damping = axisSetpointsList[ 0 ]->damping;
  jointSetpointsList[ 0 ]->inertia = axisSetpointsList[ 0 ]->inertia;
}
