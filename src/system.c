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

#include "system.h"

#include "ipc/ipc.h"

#include "shared_robot_control.h"
#include "shared_dof_variables.h"

#include "robots.h"

#include "data_io/interface/data_io.h"

#include "debug/data_logging.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>


Robot robotController = NULL;
size_t axesNumber = 0, jointsNumber = 0;

DataHandle robotInfo = NULL;

IPCConnection robotEventsConnection = NULL;
IPCConnection robotAxesConnection = NULL;
IPCConnection robotJointsConnection = NULL;


void RefreshRobotsInfo( const char*, char* );


bool System_Init( const int argc, const char** argv )
{
  if( argc < 2 ) 
  {
    Log_PrintString( NULL, "wrong usage: type \"%s --help\" for instructions", argv[ 0 ] );
    return false;
  }
  
  if( strcmp( argv[ 1 ], "--help" ) == 0 )
  {
    Log_PrintString( NULL, "usage: %s [--root <root_dir>] [--addr <connection_address>] [--log <log_dir>] <robot_name>", argv[ 0 ] );
    return false;
  }
  
  const char* rootDirectory = ".";
  const char* connectionAddress = NULL;
  const char* logDirectory = "./log/";
  const char* robotConfigName = argv[ argc - 1 ];
  
  for( int optionIndex = 1; optionIndex < argc - 1; optionIndex+=2 )
  {
    if( optionIndex + 1 >= argc - 1 )
    {
      Log_PrintString( NULL, "missing value for option %s. type \"%s --help\" for instructions", argv[ optionIndex ], argv[ 0 ] );
      return false;
    }
    
    if( strcmp( argv[ optionIndex ], "--root" ) == 0 ) rootDirectory = argv[ optionIndex + 1 ];
    else if( strcmp( argv[ optionIndex ], "--log" ) == 0 ) logDirectory = argv[ optionIndex + 1 ];
    else if( strcmp( argv[ optionIndex ], "--addr" ) == 0 ) connectionAddress = argv[ optionIndex + 1 ];
    else
    {
      Log_PrintString( NULL, "unknown option %s. type \"%s --help\" for instructions", argv[ optionIndex ], argv[ 0 ] );
      return false;
    }
  }
  
  robotEventsConnection = IPC_OpenConnection( IPC_TCP | IPC_SERVER, connectionAddress, 50000 );
  robotAxesConnection = IPC_OpenConnection( IPC_UDP | IPC_SERVER, connectionAddress, 50001 );
  robotJointsConnection = IPC_OpenConnection( IPC_UDP | IPC_SERVER, connectionAddress, 50002 );
  
  Log_SetDirectory( logDirectory );
  
  chdir( rootDirectory );
  robotInfo = DataIO_CreateEmptyData();
  DEBUG_PRINT( "loading robot configuration from %s", robotConfigName );
  RefreshRobotsInfo( robotConfigName, NULL );

  return true;
}

void System_End()
{
  //DEBUG_PRINT( "Ending Robot Control" );

  IPC_CloseConnection( robotEventsConnection );
  IPC_CloseConnection( robotAxesConnection );
  IPC_CloseConnection( robotJointsConnection );

  DataIO_UnloadData( robotInfo );

  Robot_End( robotController );
  
  //DEBUG_PRINT( "Robot Control ended" );
}

void UpdateEvents()
{
  static Byte messageBuffer[ IPC_MAX_MESSAGE_LENGTH ];

  Byte* messageIn = (Byte*) messageBuffer;
  while( IPC_ReadMessage( robotEventsConnection, messageIn ) ) 
  {
    Byte robotCommand = (Byte) *(messageIn++);
    
    DEBUG_PRINT( "received robot command: %u", robotCommand );
    
    Byte* messageOut = (Byte*) messageBuffer;
    memset( messageOut, 0, IPC_MAX_MESSAGE_LENGTH );
      
    if( robotCommand == 0x00 ) RefreshRobotsInfo( NULL, (char*) ( messageOut + 1 ) );
    else if( robotCommand == ROBOT_CMD_DISABLE ) messageOut[ 0 ] = Robot_Disable( robotController ) ? ROBOT_ST_DISABLED : 0x00;
    else if( robotCommand == ROBOT_CMD_ENABLE ) messageOut[ 0 ] = Robot_Enable( robotController ) ? ROBOT_ST_ENABLED : 0x00;
    else if( robotCommand == ROBOT_CMD_PASSIVATE ) messageOut[ 0 ] = Robot_SetControlState( robotController, ROBOT_PASSIVE ) ? ROBOT_ST_PASSIVE : 0x00;
    else if( robotCommand == ROBOT_CMD_OFFSET ) messageOut[ 0 ] = Robot_SetControlState( robotController, ROBOT_OFFSET ) ? ROBOT_ST_OFFSETTING : 0x00;
    else if( robotCommand == ROBOT_CMD_CALIBRATE ) messageOut[ 0 ] = Robot_SetControlState( robotController, ROBOT_CALIBRATION ) ? ROBOT_ST_CALIBRATING : 0x00;
    else if( robotCommand == ROBOT_CMD_PREPROCESS ) messageOut[ 0 ] = Robot_SetControlState( robotController, ROBOT_PREPROCESSING ) ? ROBOT_ST_PREPROCESSING : 0x00;
    else if( robotCommand == ROBOT_CMD_OPERATE ) messageOut[ 0 ] = Robot_SetControlState( robotController, ROBOT_OPERATION ) ? ROBOT_ST_OPERATING : 0x00;
    else if( robotCommand == ROBOT_CMD_SET_USER )
    {
      char* userName = (char*) messageIn;
      Log_SetBaseName( userName );
      messageOut[ 0 ] = ROBOT_ST_USER_SET;
      //DEBUG_PRINT( "New user name: %s", userName );
    }
    else if( robotCommand == ROBOT_CMD_SET_CONFIG )
    {
      char* robotName = (char*) messageIn;
      RefreshRobotsInfo( robotName, (char*) ( messageOut + 1 ) );
      messageOut[ 0 ] = ROBOT_ST_CONFIG_SET;
    }
    
    IPC_WriteMessage( robotEventsConnection, messageOut );
  }   
}

void UpdateAxes()
{
  static Byte message[ IPC_MAX_MESSAGE_LENGTH ];

  Byte* messageIn = (Byte*) message;
  while( IPC_ReadMessage( robotAxesConnection, messageIn ) ) 
  {
    size_t setpointBlocksNumber = (size_t) *(messageIn++);
    //Log_PrintString( NULL, "received message for %lu axes", setpointBlocksNumber );
    for( size_t setpointBlockIndex = 0; setpointBlockIndex < setpointBlocksNumber; setpointBlockIndex++ )
    {
      size_t axisIndex = (size_t) *(messageIn++);
      
      if( axisIndex >= axesNumber ) continue;      
      
      float* axisSetpointsList = (float*) messageIn;
      RobotVariables axisSetpoints = { .position = axisSetpointsList[ DOF_POSITION ], .velocity = axisSetpointsList[ DOF_VELOCITY ],
                                       .acceleration = axisSetpointsList[ DOF_ACCELERATION ], .force = axisSetpointsList[ DOF_FORCE ],
                                       .inertia = axisSetpointsList[ DOF_INERTIA ],
                                       .stiffness = axisSetpointsList[ DOF_STIFFNESS ], .damping = axisSetpointsList[ DOF_DAMPING ] };
      //if( axisIndex == 0 ) Log_PrintString( NULL, "setpoints: p: %.3f - v: %.3f", axisSetpoints.position, axisSetpoints.velocity );
      Robot_SetAxisSetpoints( robotController, axisIndex, &axisSetpoints );

      messageIn += DOF_DATA_BLOCK_SIZE;
    }
  }
  
  memset( message, 0, IPC_MAX_MESSAGE_LENGTH * sizeof(Byte) );
  size_t axisdataOffset = 1;
  for( size_t axisIndex = 0; axisIndex < axesNumber; axisIndex++ )
  {    
    RobotVariables axisMeasures = { 0 };
    if( Robot_GetAxisMeasures( robotController, axisIndex, &axisMeasures ) )
    {
      message[ 0 ]++;
      message[ axisdataOffset++ ] = (Byte) axisIndex;
      
      float* axisMeasuresList = (float*) ( message + axisdataOffset );
      
      axisMeasuresList[ DOF_POSITION ] = (float) axisMeasures.position;
      axisMeasuresList[ DOF_VELOCITY ] = (float) axisMeasures.velocity;
      axisMeasuresList[ DOF_ACCELERATION ] = (float) axisMeasures.acceleration;
      axisMeasuresList[ DOF_FORCE ] = (float) axisMeasures.force;
      axisMeasuresList[ DOF_INERTIA ] = (float) axisMeasures.inertia;
      axisMeasuresList[ DOF_STIFFNESS ] = (float) axisMeasures.stiffness;
      axisMeasuresList[ DOF_DAMPING ] = (float) axisMeasures.damping;
      
      //if( axisIndex == 0 ) Log_PrintString( NULL, "measures: p: %+.5f, v: %+.5f, f: %+.5f", axisMeasuresList[ DOF_POSITION ], axisMeasuresList[ DOF_VELOCITY ], axisMeasuresList[ DOF_FORCE ] );
    
      axisdataOffset += DOF_DATA_BLOCK_SIZE;
    }
  }
  
  if( message[ 0 ] > 0 )
  {
    //Log_PrintString( NULL, "sending measures from %lu axes", message[ 0 ] );
    IPC_WriteMessage( robotAxesConnection, (const Byte*) message );
  }
}

void UpdateJoints()
{
  static Byte messageOut[ IPC_MAX_MESSAGE_LENGTH ];
  
  memset( messageOut, 0, IPC_MAX_MESSAGE_LENGTH * sizeof(Byte) );
  size_t jointDataOffset = 1;
  for( size_t jointIndex = 0; jointIndex < jointsNumber; jointIndex++ )
  {
    messageOut[ 0 ]++;
    messageOut[ jointDataOffset++ ] = (Byte) jointIndex;
    
    float* jointMeasuresList = (float*) ( messageOut + jointDataOffset );
    
    RobotVariables jointMeasures = { 0 };
    if( Robot_GetJointMeasures( robotController, jointIndex, &jointMeasures ) )
    {
      jointMeasuresList[ DOF_POSITION ] = (float) jointMeasures.position;
      jointMeasuresList[ DOF_VELOCITY ] = (float) jointMeasures.velocity;
      jointMeasuresList[ DOF_ACCELERATION ] = (float) jointMeasures.acceleration;
      jointMeasuresList[ DOF_FORCE ] = (float) jointMeasures.force;
      jointMeasuresList[ DOF_INERTIA ] = (float) jointMeasures.inertia;
      jointMeasuresList[ DOF_STIFFNESS ] = (float) jointMeasures.stiffness;
      jointMeasuresList[ DOF_DAMPING ] = (float) jointMeasures.damping;
    }
    
    jointDataOffset += DOF_DATA_BLOCK_SIZE;
  }
  
  if( messageOut[ 0 ] > 0 )
  {
    //DEBUG_UPDATE( "sending measures for %u joints", messageOut[ 0 ] );
    IPC_WriteMessage( robotJointsConnection, (const Byte*) messageOut );
  }
}

void System_Update()
{ 
  UpdateEvents();
  UpdateAxes();
  UpdateJoints();
}


void RefreshRobotsInfo( const char* robotName, char* sharedControlsString )
{ 
  if( robotName != NULL )
  {     
    if( robotController != NULL ) Robot_End( robotController );
    robotController = Robot_Init( robotName );
    if( robotController != NULL )
    {
      DataIO_SetStringValue( robotInfo, "id", robotName );   
      
      DataHandle sharedJointsList = DataIO_AddList( robotInfo, "joints" );
      DataHandle sharedAxesList = DataIO_AddList( robotInfo, "axes" );
      
      axesNumber = Robot_GetAxesNumber( robotController ); 

      for( size_t axisIndex = 0; axisIndex < axesNumber; axisIndex++ )
      {
        const char* axisName = Robot_GetAxisName( robotController, axisIndex );
        if( axisName != NULL ) DataIO_SetStringValue( sharedAxesList, NULL, axisName );
      }
      
      jointsNumber = Robot_GetJointsNumber( robotController );

      for( size_t jointIndex = 0; jointIndex < jointsNumber; jointIndex++ )
      {
        const char* jointName = Robot_GetJointName( robotController, jointIndex );
        if( jointName != NULL ) DataIO_SetStringValue( sharedJointsList, NULL, jointName );
      }
    }
  }
  
  if( sharedControlsString != NULL )
  {
    char* robotControlsString = DataIO_GetDataString( robotInfo );
    Log_PrintString( NULL, "robots info string: %s", robotControlsString );
    strncpy( sharedControlsString, robotControlsString, strlen( robotControlsString ) );
    free( robotControlsString );
  }
}
