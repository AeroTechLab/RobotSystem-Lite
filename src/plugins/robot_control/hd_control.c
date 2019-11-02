//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
//  Copyright (c) 2016-2019 Leonardo Consoni <consoni_2519@hotmail.com>             //
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


#include "robot_control/robot_control.h"

#include <math.h>

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

enum ControlState controlState = CONTROL_PASSIVE;

double lastForceError = 0.0;
double velocitySetpoint = 0.0;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


bool InitController( const char* configurationString ) { return true; }

void EndController() { return; }

size_t GetJointsNumber() { return DOFS_NUMBER; }

const char** GetJointNamesList() { return (const char**) DOF_NAMES; }

size_t GetAxesNumber() { return DOFS_NUMBER; }

const char** GetAxisNamesList() { return (const char**) DOF_NAMES; }

size_t GetExtraInputsNumber( void ) { return 0; }
      
void SetExtraInputsList( double* inputsList ) { return; }

size_t GetExtraOutputsNumber( void ) { return 0; }
         
void GetExtraOutputsList( double* outputsList ) { return; }

void SetControlState( enum ControlState newControlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", newControlState );
  
  controlState = newControlState;
}

void RunControlStep( DoFVariables** jointMeasuresList, DoFVariables** axisMeasuresList, DoFVariables** jointSetpointsList, DoFVariables** axisSetpointsList, double timeDelta )
{
  axisMeasuresList[ 0 ]->position = jointMeasuresList[ 0 ]->position;
  axisMeasuresList[ 0 ]->velocity = jointMeasuresList[ 0 ]->velocity;
  axisMeasuresList[ 0 ]->acceleration = jointMeasuresList[ 0 ]->acceleration;
  axisMeasuresList[ 0 ]->force = jointMeasuresList[ 0 ]->force;
  axisMeasuresList[ 0 ]->stiffness = jointMeasuresList[ 0 ]->stiffness;
  axisMeasuresList[ 0 ]->damping = jointMeasuresList[ 0 ]->damping;
  
  double proportionalGain = 0.0, integralGain = 0.0;
  if( controlState == CONTROL_OPERATION )
  {
    proportionalGain = axisSetpointsList[ 0 ]->stiffness; 
    integralGain = axisSetpointsList[ 0 ]->damping;
  }
  
  double forceError = axisSetpointsList[ 0 ]->force - axisMeasuresList[ 0 ]->force;
  velocitySetpoint += proportionalGain * ( forceError - lastForceError ) + integralGain * timeDelta * forceError;
  axisSetpointsList[ 0 ]->velocity = velocitySetpoint;
  lastForceError = forceError;
  
  //fprintf( stderr, "f=%.4f, fd=%.4f, kp=%.1f, ki=%.1f, vd=%.4f\n", axisMeasuresList[ 0 ]->force, axisSetpointsList[ 0 ]->force, proportionalGain, integralGain, velocitySetpoint );
  
  jointSetpointsList[ 0 ]->position = axisSetpointsList[ 0 ]->position;
  jointSetpointsList[ 0 ]->velocity = axisSetpointsList[ 0 ]->velocity;
  jointSetpointsList[ 0 ]->acceleration = axisSetpointsList[ 0 ]->acceleration;
  jointSetpointsList[ 0 ]->force = axisSetpointsList[ 0 ]->force;
  jointSetpointsList[ 0 ]->stiffness = axisSetpointsList[ 0 ]->stiffness;
  jointSetpointsList[ 0 ]->damping = axisSetpointsList[ 0 ]->damping;
}
