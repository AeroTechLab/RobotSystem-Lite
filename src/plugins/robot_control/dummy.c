//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
//  Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>             //
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


#include "robot_control_interface.h"

#include <math.h>

#define DOFS_NUMBER 1

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );

Controller InitController( const char* configurationString )
{
  return NULL;
}

void EndController( Controller controller )
{
  
}

size_t GetJointsNumber( Controller controller )
{
  return DOFS_NUMBER;
}

char** GetJointNamesList( Controller controller )
{
  return (char**) DOF_NAMES;
}

size_t GetAxesNumber( Controller controller )
{
  return DOFS_NUMBER;
}

char** GetAxisNamesList( Controller controller )
{
  return (char**) DOF_NAMES;
}

void SetControlState( Controller ref_controller, enum ControlState controlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", controlState );
}

void RunControlStep( Controller controller, double** jointMeasuresTable, double** axisMeasuresTable, double** jointSetpointsTable, double** axisSetpointsTable )
{
  axisMeasuresTable[ 0 ][ CONTROL_POSITION ] = jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
  axisMeasuresTable[ 0 ][ CONTROL_VELOCITY ] = jointMeasuresTable[ 0 ][ CONTROL_VELOCITY ];
  axisMeasuresTable[ 0 ][ CONTROL_ACCELERATION ] = jointMeasuresTable[ 0 ][ CONTROL_ACCELERATION ];
  axisMeasuresTable[ 0 ][ CONTROL_FORCE ] = jointMeasuresTable[ 0 ][ CONTROL_FORCE ];
  axisMeasuresTable[ 0 ][ CONTROL_STIFFNESS ] = jointMeasuresTable[ 0 ][ CONTROL_STIFFNESS ];
  axisMeasuresTable[ 0 ][ CONTROL_DAMPING ] = jointMeasuresTable[ 0 ][ CONTROL_DAMPING ];
  
  jointSetpointsTable[ 0 ][ CONTROL_POSITION ] = axisSetpointsTable[ 0 ][ CONTROL_POSITION ];
  jointSetpointsTable[ 0 ][ CONTROL_VELOCITY ] = axisSetpointsTable[ 0 ][ CONTROL_VELOCITY ];
  jointSetpointsTable[ 0 ][ CONTROL_ACCELERATION ] = axisSetpointsTable[ 0 ][ CONTROL_ACCELERATION ];
  jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = axisSetpointsTable[ 0 ][ CONTROL_FORCE ];
  jointSetpointsTable[ 0 ][ CONTROL_STIFFNESS ] = axisSetpointsTable[ 0 ][ CONTROL_STIFFNESS ];
  jointSetpointsTable[ 0 ][ CONTROL_DAMPING ] = axisSetpointsTable[ 0 ][ CONTROL_DAMPING ];
  
  double stiffness = jointSetpointsTable[ 0 ][ CONTROL_STIFFNESS ]; 
  double positionError = jointSetpointsTable[ 0 ][ CONTROL_POSITION ] - jointMeasuresTable[ 0 ][ CONTROL_POSITION ];
  jointSetpointsTable[ 0 ][ CONTROL_FORCE ] = stiffness * positionError;
}
