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

#include "system.h"

#include "ipc.h"

#include "shared_robot_control.h"
#include "shared_axis_data.h"
#include "shared_joint_data.h"

#include "robots.h"

#include "data_io.h"

//#include "utils/debug/async_debug.h"
//#include "utils/debug/data_logging.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

const unsigned long UPDATE_INTERVAL_MS = 5;


Robot robotController = NULL;

DataHandle robotConfig = NULL;

IPCConnection robotEventsConnection = NULL;
IPCConnection robotAxesConnection = NULL;
IPCConnection robotJointsConnection = NULL;

Axis* axesList = NULL;
Joint* jointsList = NULL;
size_t dofsNumber = 0;


void RefreshRobotsInfo( const char* );


bool System_Init( const int argc, const char** argv )
{
  if( argc < 4 ) 
  {
    //DEBUG_PRINT( "usage: %s <config_dir> <config_type> <config_file>", argv[ 0 ] );
    return false;
  }
  
  const char* configDirectory = argv[ 1 ];
  const char* configType = argv[ 2 ];
  const char* configFile = argv[ 3 ];
  
  robotEventsConnection = IPC_OpenConnection( IPC_TCP, NULL, 50000 );
  robotAxesConnection = IPC_OpenConnection( IPC_UDP, NULL, 50001 );
  robotJointsConnection = IPC_OpenConnection( IPC_UDP, NULL, 50002 );
  
  //DEBUG_PRINT( "looking for %s robotsConfiguration", configType );
  
  DataIO_SetBaseFilePath( configDirectory );
  
  robotConfig = DataIO_CreateEmptyData();
  //DEBUG_PRINT( "loading robotsConfiguration from %s", configDirectory );
  RefreshRobotsInfo( configFile );

  return true;
}

void System_End()
{
  //DEBUG_PRINT( "Ending RobRehab Control on thread %lx", THREAD_ID );

  IPC_CloseConnection( robotEventsConnection );
  IPC_CloseConnection( robotAxesConnection );
  IPC_CloseConnection( robotJointsConnection );

  DataIO_UnloadData( robotConfig );
  
  if( axesList != NULL ) free( axesList );
  if( jointsList != NULL ) free( jointsList );

  Robot_End( robotController );
  
  //DEBUG_PRINT( "RobRehab Control ended on thread %lx", THREAD_ID );
}

void UpdateEvents()
{
  static Byte message[ IPC_MAX_MESSAGE_LENGTH ];
  static RemoteID messageSenderID;
  
  if( IPC_ReadMessage( robotAxesConnection, (Byte*) message, &messageSenderID ) ) 
  {
    Byte* messageIn = (Byte*) message;
    Byte robotCommand = (Byte) *(messageIn++);
    
    if( robotCommand == 0x00 )
    {
      //DEBUG_PRINT( "info string request: %u", commandBlocksNumber );
      RefreshRobotsInfo( NULL );
    }
    else
    {
      Byte robotState = 0x00;
      
      if( robotCommand == ROBOT_CMD_DISABLE ) robotState = Robot_Disable( robotController ) ? ROBOT_ST_DISABLED : 0x00;
      else if( robotCommand == ROBOT_CMD_ENABLE ) robotState = Robot_Enable( robotController ) ? ROBOT_ST_ENABLED : 0x00;
      else if( robotCommand == ROBOT_CMD_OFFSET ) robotState = Robot_SetControlState( robotController, ROBOT_OFFSET ) ? ROBOT_ST_OFFSETTING : 0x00;
      else if( robotCommand == ROBOT_CMD_CALIBRATE ) robotState = Robot_SetControlState( robotController, ROBOT_CALIBRATION ) ? ROBOT_ST_CALIBRATING : 0x00;
      else if( robotCommand == ROBOT_CMD_PREPROCESS ) robotState = Robot_SetControlState( robotController, ROBOT_PREPROCESSING ) ? ROBOT_ST_PREPROCESSING : 0x00;
      else if( robotCommand == ROBOT_CMD_OPERATE ) robotState = Robot_SetControlState( robotController, ROBOT_OPERATION ) ? ROBOT_ST_OPERATING : 0x00;
      else if( robotCommand == ROBOT_CMD_SET_USER )
      {
        char* userName = (char*) messageIn;
        //DataLogging.SetBaseDirectory( userName );
        //DEBUG_PRINT( "New user name: %s", userName );
      }
    }
  }   
}

void UpdateAxes()
{
  static Byte message[ IPC_MAX_MESSAGE_LENGTH ];

  if( IPC_ReadMessage( robotAxesConnection, (Byte*) message, NULL ) ) 
  {
    Byte* messageIn = (Byte*) message;
    size_t setpointBlocksNumber = (size_t) *(messageIn++);
    //DEBUG_PRINT( "received message from client %lu for %lu axes", clientID, setpointBlocksNumber );
    for( size_t setpointBlockIndex = 0; setpointBlockIndex < setpointBlocksNumber; setpointBlockIndex++ )
    {
      size_t axisIndex = (size_t) *(messageIn++);
      Axis axis = axesList[ axisIndex ];
      
      //DEBUG_PRINT( "receiving for axis %lu (of %lu)", axisIndex, kv_max( axisControlClientsList ) );

      if( axisIndex >= dofsNumber ) continue;      

      //if( kv_A( axisControlClientsList, axisIndex ) == IPC_INVALID_CONNECTION ) kv_A( axisControlClientsList, axisIndex ) = clientID;
      
      //if( kv_A( axisControlClientsList, axisIndex ) == clientID )  
      //{
        //DEBUG_PRINT( "receiving axis %lu setpoints", axisIndex );
        //SHMControl.SetControlByte( sharedRobotAxesData, axisIndex, axisMask );
        float* axisSetpointsList = (float*) messageIn;
        RobotVariables axisSetpoints = { .position = axisSetpointsList[ AXIS_POSITION ], .velocity = axisSetpointsList[ AXIS_VELOCITY ],
                                     .acceleration = axisSetpointsList[ AXIS_ACCELERATION ], .force = axisSetpointsList[ AXIS_FORCE ],
                                     .stiffness = axisSetpointsList[ AXIS_STIFFNESS ], .damping = axisSetpointsList[ AXIS_DAMPING ] };
        Robot_SetAxisSetpoints( axis, &axisSetpoints );
      //}

      messageIn += AXIS_DATA_BLOCK_SIZE;
    }
  }
  
  memset( message, 0, IPC_MAX_MESSAGE_LENGTH * sizeof(Byte) );
  size_t axisdataOffset = 1;
  for( size_t axisIndex = 0; axisIndex < dofsNumber; axisIndex++ )
  {
    message[ 0 ]++;
    message[ axisdataOffset++ ] = (Byte) axisIndex;
    
    Axis axis = axesList[ axisIndex ];
    
    float* axisMeasuresList = (float*) ( message + axisdataOffset );
    
    RobotVariables axisMeasures = { 0 };
    if( Robot_GetAxisMeasures( axis, &axisMeasures ) )
    {
      axisMeasuresList[ AXIS_POSITION ] = (float) axisMeasures.position;
      axisMeasuresList[ AXIS_VELOCITY ] = (float) axisMeasures.velocity;
      axisMeasuresList[ AXIS_ACCELERATION ] = (float) axisMeasures.acceleration;
      axisMeasuresList[ AXIS_FORCE ] = (float) axisMeasures.force;
      axisMeasuresList[ AXIS_STIFFNESS ] = (float) axisMeasures.stiffness;
      axisMeasuresList[ AXIS_DAMPING ] = (float) axisMeasures.damping;
      
      //DEBUG_PRINT( "measures: p: %.3f - v: %.3f - f: %.3f", axisMeasuresList[ AXIS_POSITION ], axisMeasuresList[ AXIS_VELOCITY ], axisMeasuresList[ AXIS_FORCE ] );
    }
    
    axisdataOffset += AXIS_DATA_BLOCK_SIZE;
  }
  
  //DEBUG_PRINT( "sending %u axes for client %lu", messageOut[ 0 ], clientID );
  IPC_WriteMessage( robotAxesConnection, (const Byte*) message, NULL );
}

void UpdateJoints()
{
  static Byte messageOut[ IPC_MAX_MESSAGE_LENGTH ];
  
  memset( messageOut, 0, IPC_MAX_MESSAGE_LENGTH * sizeof(Byte) );
  size_t jointDataOffset = 1;
  for( size_t jointIndex = 0; jointIndex < dofsNumber; jointIndex++ )
  {
    messageOut[ 0 ]++;
    messageOut[ jointDataOffset++ ] = (Byte) jointIndex;
    
    Joint joint = jointsList[ jointIndex ];
    
    float* jointMeasuresList = (float*) ( messageOut + jointDataOffset );
    
    RobotVariables jointMeasures = { 0 };
    if( Robot_GetJointMeasures( joint, &jointMeasures ) )
    {
      jointMeasuresList[ AXIS_POSITION ] = (float) jointMeasures.position;
      jointMeasuresList[ AXIS_VELOCITY ] = (float) jointMeasures.velocity;
      jointMeasuresList[ AXIS_ACCELERATION ] = (float) jointMeasures.acceleration;
      jointMeasuresList[ AXIS_FORCE ] = (float) jointMeasures.force;
      jointMeasuresList[ AXIS_STIFFNESS ] = (float) jointMeasures.stiffness;
      jointMeasuresList[ AXIS_DAMPING ] = (float) jointMeasures.damping;
    }
    
    jointDataOffset += JOINT_DATA_BLOCK_SIZE;
  }
  
  //DEBUG_UPDATE( "sending measures for %u joints", messageOut[ 0 ] );
  IPC_WriteMessage( robotJointsConnection, (const Byte*) messageOut, NULL );
}

void System_Update()
{ 
  UpdateAxes();
  UpdateEvents();
  UpdateJoints();
}


void RefreshRobotsInfo( const char* configFile )
{
  static char configFileName[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  DataHandle sharedJointsList = DataIO_AddList( robotConfig, "joints" );
  DataHandle sharedAxesList = DataIO_AddList( robotConfig, "axes" );
  
  if( configFile != NULL ) strncpy( configFileName, configFile, DATA_IO_MAX_FILE_PATH_LENGTH );
  DataHandle robotsConfiguration = DataIO_LoadFileData( configFileName );
  if( robotsConfiguration != NULL )
  {
    char* robotName = DataIO_GetStringValue( robotsConfiguration, NULL, "id" );
    if( robotName != NULL )
    {     
      robotController = Robot_Init( robotName );
      if( robotController != NULL )
      {
        DataIO_SetStringValue( robotConfig, "id", robotName );   
        //DEBUG_PRINT( "set string value %s to data list %p", robotName, sharedRobotsList );
        
        size_t axesNumber = Robot_GetAxesNumber( robotController ); 
        axesList = (Axis*) realloc( axesList, axesNumber * sizeof(Axis) );
        //DEBUG_PRINT( "got %lu axes for robot %lu", axesNumber, robotController );
        for( size_t axisIndex = 0; axisIndex < axesNumber; axisIndex++ )
        {
          const char* axisName = Robot_GetAxisName( robotController, axisIndex );
          if( axisName != NULL )
          {
            DataIO_SetStringValue( sharedAxesList, NULL, axisName );
            axesList[ axisIndex ] = Robot_GetAxis( robotController, axisIndex );
          }
        }
        
        size_t jointsNumber = Robot_GetJointsNumber( robotController );
        jointsList = (Joint*) realloc( jointsList, jointsNumber * sizeof(Joint) );
        for( size_t jointIndex = 0; jointIndex < jointsNumber; jointIndex++ )
        {
          const char* jointName = Robot_GetJointName( robotController, jointIndex );
          if( jointName != NULL )
          {
            DataIO_SetStringValue( sharedJointsList, NULL, jointName );
            jointsList[ jointIndex ] = Robot_GetJoint( robotController, jointIndex );
          }
        }
      }
    }

    DataIO_UnloadData( robotsConfiguration );
  }
  
  char* robotEventsConnectionString = DataIO_GetDataString( robotConfig );
  if( robotEventsConnectionString != NULL )
  {
    //DEBUG_PRINT( "robots info string: %s", robotEventsConnectionString );
    
    IPC_WriteMessage( robotEventsConnection, (const Byte*) robotEventsConnectionString, NULL );
    
    free( robotEventsConnectionString );
  }
}
