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


#include "robot_control_interface.h"

#define DOFS_NUMBER 2

const char* AXIS_NAMES[ DOFS_NUMBER ] = { "DP", "IE" };
const char* JOINT_NAMES[ DOFS_NUMBER ] = { "RIGHT", "LEFT" };

DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE )

RobotController InitController( const char* configurationString )
{
  return NULL;
}

void EndController( RobotController controller )
{

}

size_t GetJointsNumber( RobotController controller )
{
  return DOFS_NUMBER;
}

const char** GetJointNamesList( RobotController controller )
{
  return (const char**) JOINT_NAMES;
}

size_t GetAxesNumber( RobotController controller )
{
  return DOFS_NUMBER;
}

const char** GetAxisNamesList( RobotController controller )
{
  return (const char**) AXIS_NAMES;
}

void SetControlState( RobotController controller, enum RobotState controlState )
{
  fprintf( stderr, "Setting robot control phase: %x\n", controlState );
}

void RunControlStep( RobotController controller, RobotVariables** jointMeasuresTable, RobotVariables** axisMeasuresTable, RobotVariables** jointSetpointsTable, RobotVariables** axisSetpointsTable, double timeDelta )
{
  const double BALL_LENGTH = 0.14;
  const double BALL_BALL_WIDTH = 0.19;
  const double SHIN_LENGTH = 0.42;
  const double ACTUATOR_LENGTH = 0.443;

  double positionMean = ( jointMeasuresTable[ 0 ]->position + jointMeasuresTable[ 1 ]->position ) / 2.0;
  double dpSin = ( pow( BALL_LENGTH, 2 ) + pow( SHIN_LENGTH, 2 ) - pow( ACTUATOR_LENGTH - positionMean, 2 ) ) / ( 2 * BALL_LENGTH * SHIN_LENGTH );
  if( dpSin > 1.0 ) dpSin = 1.0;
  else if( dpSin < -1.0 ) dpSin = -1.0;
  double dpPosition = asin( dpSin ); // + dpOffset; ?
  axisMeasuresTable[ 0 ]->velocity = ( dpPosition - axisMeasuresTable[ 0 ]->position ) / timeDelta;
  axisMeasuresTable[ 0 ]->position = dpPosition;

  double positionDiff = ( jointMeasuresTable[ 0 ]->position - jointMeasuresTable[ 1 ]->position );
  double iePosition = atan( positionDiff / BALL_BALL_WIDTH );  // + ieOffset; ?
  axisMeasuresTable[ 1 ]->velocity = ( iePosition - axisMeasuresTable[ 1 ]->position ) / timeDelta;
  axisMeasuresTable[ 1 ]->position = iePosition;

  double rightForce = jointMeasuresTable[ 0 ]->force;
  double leftForce = jointMeasuresTable[ 1 ]->force;
  axisMeasuresTable[ 0 ]->force = ( leftForce + rightForce ) * BALL_LENGTH;
  axisMeasuresTable[ 1 ]->force = ( leftForce - rightForce ) * BALL_BALL_WIDTH / 2.0;

  double dpRefStiffness = 10.0;//axisSetpointsTable[ 0 ]->stiffness;
  double dpPositionError = axisSetpointsTable[ 0 ]->position - axisMeasuresTable[ 0 ]->position;
  double dpRefDamping = axisSetpointsTable[ 0 ]->damping;
  double dpVelocity = axisMeasuresTable[ 0 ]->velocity;
  double dpRefTorque = dpRefStiffness * dpPositionError - dpRefDamping * dpVelocity;
  axisSetpointsTable[ 0 ]->force = dpRefTorque;

  double ieRefStiffness = axisSetpointsTable[ 1 ]->stiffness;
  double iePositionError = axisSetpointsTable[ 1 ]->position - axisMeasuresTable[ 1 ]->position;
  double ieRefDamping = axisSetpointsTable[ 1 ]->damping;
  double ieVelocity = axisMeasuresTable[ 1 ]->velocity;
  double ieRefTorque = ieRefStiffness * iePositionError - ieRefDamping * ieVelocity;
  axisSetpointsTable[ 1 ]->force = ieRefTorque;

  // measured force = setpoint force
  axisMeasuresTable[ 0 ]->force = axisSetpointsTable[ 0 ]->force;
  axisMeasuresTable[ 1 ]->force = axisSetpointsTable[ 1 ]->force;

  double dpRefForce = dpRefTorque / BALL_LENGTH;
  double ieRefForce = ieRefTorque / ( BALL_BALL_WIDTH / 2.0 );
  jointSetpointsTable[ 0 ]->force = ( -dpRefForce - ieRefForce ) / 2.0;
  jointSetpointsTable[ 1 ]->force = ( -dpRefForce + ieRefForce ) / 2.0;
}
