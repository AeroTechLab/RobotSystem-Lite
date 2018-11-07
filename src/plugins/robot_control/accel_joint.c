//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
//  Copyright (c) 2016-2018 Leonardo Consoni <consoni_2519@hotmail.com>             //
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

#define DOFS_NUMBER 3

const char* DOF_NAMES[ DOFS_NUMBER ] = { "X", "Y", "Z" };

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
  return (char**) DOF_NAMES;
}

size_t GetAxesNumber()
{
  return DOFS_NUMBER;
}

const char** GetAxisNamesList()
{
  return (char**) DOF_NAMES;
}

void SetControlState( enum RobotState newControlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", newControlState );
}

void RunControlStep( RobotVariables** jointMeasuresTable, RobotVariables** axisMeasuresTable, RobotVariables** jointSetpointsTable, RobotVariables** axisSetpointsTable, double timeDelta )
{
  for( size_t dofIndex = 0; dofIndex < DOFS_NUMBER; dofIndex++ )
  {
    axisMeasuresTable[ dofIndex ]->position = jointMeasuresTable[ dofIndex ]->position;
    axisMeasuresTable[ dofIndex ]->velocity = jointMeasuresTable[ dofIndex ]->velocity;
    axisMeasuresTable[ dofIndex ]->acceleration = jointMeasuresTable[ dofIndex ]->acceleration;
    axisMeasuresTable[ dofIndex ]->force = jointMeasuresTable[ dofIndex ]->force;
    axisMeasuresTable[ dofIndex ]->stiffness = jointMeasuresTable[ dofIndex ]->stiffness;
    axisMeasuresTable[ dofIndex ]->damping = jointMeasuresTable[ dofIndex ]->damping;
  
    jointSetpointsTable[ dofIndex ]->position = axisSetpointsTable[ dofIndex ]->position;
    jointSetpointsTable[ dofIndex ]->velocity = axisSetpointsTable[ dofIndex ]->velocity;
    jointSetpointsTable[ dofIndex ]->acceleration = axisSetpointsTable[ dofIndex ]->acceleration;
    jointSetpointsTable[ dofIndex ]->force = axisSetpointsTable[ dofIndex ]->force;
    jointSetpointsTable[ dofIndex ]->stiffness = axisSetpointsTable[ dofIndex ]->stiffness;
    jointSetpointsTable[ dofIndex ]->damping = axisSetpointsTable[ dofIndex ]->damping;
  }
}
