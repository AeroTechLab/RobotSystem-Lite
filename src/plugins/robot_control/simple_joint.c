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


#include "robot_control/robot_control.h"

#include <math.h>

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );

bool InitController( const char* configurationString )
{
  return true;
}

void EndController()
{
  
}

size_t GetJointsNumber()
{
  return DOFS_NUMBER;
}

const char** GetJointNamesList()
{
  return DOF_NAMES;
}

size_t GetAxesNumber()
{
  return DOFS_NUMBER;
}

const char** GetAxisNamesList()
{
  return DOF_NAMES;
}

void SetControlState( enum RobotState controlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", controlState );
}

void RunControlStep( RobotVariables** jointMeasuresTable, RobotVariables** axisMeasuresTable, RobotVariables** jointSetpointsTable, RobotVariables** axisSetpointsTable, double elapsedTime )
{
  axisMeasuresTable[ 0 ]->position = jointMeasuresTable[ 0 ]->position;
  axisMeasuresTable[ 0 ]->velocity = jointMeasuresTable[ 0 ]->velocity;
  axisMeasuresTable[ 0 ]->acceleration = jointMeasuresTable[ 0 ]->acceleration;
  axisMeasuresTable[ 0 ]->force = jointMeasuresTable[ 0 ]->force;
  axisMeasuresTable[ 0 ]->stiffness = jointMeasuresTable[ 0 ]->stiffness;
  axisMeasuresTable[ 0 ]->damping = jointMeasuresTable[ 0 ]->damping;
  
  jointSetpointsTable[ 0 ]->position = axisSetpointsTable[ 0 ]->position;
  jointSetpointsTable[ 0 ]->velocity = axisSetpointsTable[ 0 ]->velocity;
  jointSetpointsTable[ 0 ]->acceleration = axisSetpointsTable[ 0 ]->acceleration;
  jointSetpointsTable[ 0 ]->force = axisSetpointsTable[ 0 ]->force;
  jointSetpointsTable[ 0 ]->stiffness = axisSetpointsTable[ 0 ]->stiffness;
  jointSetpointsTable[ 0 ]->damping = axisSetpointsTable[ 0 ]->damping;
  
  double stiffness = jointSetpointsTable[ 0 ]->position; 
  double positionError = jointSetpointsTable[ 0 ]->position - jointMeasuresTable[ 0 ]->position;
  jointSetpointsTable[ 0 ]->force = stiffness * positionError;
}
