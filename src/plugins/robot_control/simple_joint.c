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

#include "robot_control_interface.h"

#define DOFS_NUMBER 1

typedef struct _ControllerData
{
  bool axesChangedList[ DOFS_NUMBER ];
  bool jointsChangedList[ DOFS_NUMBER ];
  double lastAxisPositionsList[ DOFS_NUMBER ];
  double lastJointPositionsList[ DOFS_NUMBER ];
}
ControlData;

const char* DOF_NAMES[ DOFS_NUMBER ] = { "angle" };


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE ) 


RobotController InitController( const char* configurationString )
{
  ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
  
  return newController;
}

void EndController( RobotController ref_controller )
{
  if( ref_controller != NULL ) free( ref_controller );
}

size_t GetJointsNumber( RobotController controller )
{
  return DOFS_NUMBER;
}

const char** GetJointNamesList( RobotController ref_controller )
{
  return (const char**) DOF_NAMES;
}

const bool* GetJointsChangedList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const bool*) controller->jointsChangedList;
}

size_t GetAxesNumber( RobotController ref_controller )
{
  return DOFS_NUMBER;
}

const char** GetAxisNamesList( RobotController ref_controller )
{
  return (const char**) DOF_NAMES;
}

const bool* GetAxesChangedList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return NULL;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const bool*) controller->axesChangedList;
}

void SetControlState( RobotController ref_controller, enum RobotState controlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", controlState );
}

void RunControlStep( RobotController ref_controller, RobotVariables** jointMeasuresList, RobotVariables** axisMeasuresList, 
                                                     RobotVariables** jointSetpointsList, RobotVariables** axisSetpointsList, double timeDelta )
{
  if( ref_controller == NULL ) return;
  ControlData* controller = (ControlData*) ref_controller;
  
  axisMeasuresList[ 0 ]->position = jointMeasuresList[ 0 ]->position;
  axisMeasuresList[ 0 ]->velocity = jointMeasuresList[ 0 ]->velocity;
  axisMeasuresList[ 0 ]->acceleration = jointMeasuresList[ 0 ]->acceleration;
  axisMeasuresList[ 0 ]->force = jointMeasuresList[ 0 ]->force;
  axisMeasuresList[ 0 ]->stiffness = jointMeasuresList[ 0 ]->stiffness;
  axisMeasuresList[ 0 ]->damping = jointMeasuresList[ 0 ]->damping;
  
  controller->axesChangedList[ 0 ] = false;
  if( fabs( axisMeasuresList[ 0 ]->position - controller->lastAxisPositionsList[ 0 ] ) > 0.01 || fabs( axisMeasuresList[ 0 ]->force ) > 0.05 )
  {
    controller->axesChangedList[ 0 ] = true;
    controller->lastAxisPositionsList[ 0 ] = axisMeasuresList[ 0 ]->position;
  }
  
  controller->jointsChangedList[ 0 ] = false;
  if( fabs( jointMeasuresList[ 0 ]->position - controller->lastJointPositionsList[ 0 ] ) > 0.01 || fabs( jointMeasuresList[ 0 ]->force ) > 0.05 )
  {
    controller->jointsChangedList[ 0 ] = true;
    controller->lastJointPositionsList[ 0 ] = jointMeasuresList[ 0 ]->position;
  }
  
  double stiffness = axisSetpointsList[ 0 ]->stiffness; 
  double positionError = axisSetpointsList[ 0 ]->position - axisMeasuresList[ 0 ]->position;
  double dampingForce = axisSetpointsList[ 0 ]->damping * axisMeasuresList[ 0 ]->velocity;
  double controlForce = stiffness * positionError - dampingForce;
  
  jointSetpointsList[ 0 ]->position = axisSetpointsList[ 0 ]->position;
  jointSetpointsList[ 0 ]->velocity = axisSetpointsList[ 0 ]->velocity;
  jointSetpointsList[ 0 ]->acceleration = axisSetpointsList[ 0 ]->acceleration;
  jointSetpointsList[ 0 ]->force = axisSetpointsList[ 0 ]->force + controlForce;
  jointSetpointsList[ 0 ]->stiffness = axisSetpointsList[ 0 ]->stiffness;
  jointSetpointsList[ 0 ]->damping = axisSetpointsList[ 0 ]->damping;
}
