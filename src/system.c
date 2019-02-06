////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2019 Leonardo Consoni <consoni_2519@hotmail.com>       //
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

#include "robot.h"

#include "data_io/interface/data_io.h"

#include "debug/data_logging.h"
#include "timing/timing.h" 

#include "config_keys.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#ifdef _CVI_DLL_
#define chdir( dirName )
#elif WIN32
#include <dirent.h>
#define chdir _chdir
#else
#include <unistd.h>
#endif

const unsigned long NETWORK_UPDATE_MIN_INTERVAL_MS = 20;
static unsigned long lastUpdateTimeMS = 0;
static unsigned long lastNetworkUpdateElapsedTimeMS = NETWORK_UPDATE_MIN_INTERVAL_MS;


bool robotInitialized = false;
size_t axesNumber = 0, jointsNumber = 0;

DataHandle robotInfo = NULL;

IPCConnection robotEventsConnection = NULL;
IPCConnection robotAxesConnection = NULL;
IPCConnection robotJointsConnection = NULL;


void RefreshRobotsInfo( const char*, char* );


bool System_Init( const int argc, const char** argv )
{
  DEBUG_PRINT( "Starting Robot Control at time %g", Time_GetExecSeconds() );
  
  if( argc < 2 ) 
  {
    DEBUG_PRINT( "wrong usage: type \"%s --help\" for instructions", argv[ 0 ] );
    return false;
  }
  
  if( strcmp( argv[ 1 ], "--help" ) == 0 )
  {
    DEBUG_PRINT( "usage: %s [--root <root_dir>] [--addr <connection_address>] [--log <log_dir>] <robot_name>", argv[ 0 ] );
    return false;
  }
  
  const char* rootDirectory = ".";
  const char* connectionAddress = NULL;
  const char* logDirectory = "./" KEY_LOG "s/";
  const char* robotConfigName = argv[ argc - 1 ];
  
  for( int optionIndex = 1; optionIndex < argc - 1; optionIndex+=2 )
  {
    if( optionIndex + 1 >= argc - 1 )
    {
      DEBUG_PRINT( "missing value for option %s. type \"%s --help\" for instructions", argv[ optionIndex ], argv[ 0 ] );
      return false;
    }
    
    if( strcmp( argv[ optionIndex ], "--root" ) == 0 ) rootDirectory = argv[ optionIndex + 1 ];
    else if( strcmp( argv[ optionIndex ], "--log" ) == 0 ) logDirectory = argv[ optionIndex + 1 ];
    else if( strcmp( argv[ optionIndex ], "--addr" ) == 0 ) connectionAddress = argv[ optionIndex + 1 ];
    else
    {
      DEBUG_PRINT( "unknown option %s. type \"%s --help\" for instructions", argv[ optionIndex ], argv[ 0 ] );
      return false;
    }
  }
  
  robotEventsConnection = IPC_OpenConnection( IPC_TCP | IPC_SERVER, connectionAddress, 50000 );
  robotAxesConnection = IPC_OpenConnection( IPC_UDP | IPC_SERVER, connectionAddress, 50001 );
  robotJointsConnection = IPC_OpenConnection( IPC_UDP | IPC_SERVER, connectionAddress, 50002 );
  
  Log_SetDirectory( logDirectory );
  Log_SetTimeStamp();

  chdir( rootDirectory );
  robotInfo = DataIO_CreateEmptyData();
  DEBUG_PRINT( "loading robot configuration from %s", robotConfigName );
  RefreshRobotsInfo( robotConfigName, NULL );

  lastUpdateTimeMS = Time_GetExecMilliseconds();
  
  return true;
}

void System_End()
{
  DEBUG_PRINT( "Ending Robot Control at time %g", Time_GetExecSeconds() );

  IPC_CloseConnection( robotEventsConnection );
  IPC_CloseConnection( robotAxesConnection );
  IPC_CloseConnection( robotJointsConnection );

  DataIO_UnloadData( robotInfo );

  Robot_End();
  
  DEBUG_PRINT( "Robot Control ended at time %g", Time_GetExecSeconds() );
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
      
    if( robotCommand == ROBOT_REQ_GET_INFO ) 
    {
      messageOut[ 0 ] = ROBOT_REP_GOT_INFO;
      RefreshRobotsInfo( NULL, (char*) ( messageOut + 1 ) );
    }
    else if( robotCommand == ROBOT_REQ_DISABLE ) messageOut[ 0 ] = Robot_Disable() ? ROBOT_REP_DISABLED : 0x00;
    else if( robotCommand == ROBOT_REQ_ENABLE ) messageOut[ 0 ] = Robot_Enable() ? ROBOT_REP_ENABLED : 0x00;
    else if( robotCommand == ROBOT_REQ_PASSIVATE ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_PASSIVE ) ? ROBOT_REP_PASSIVE : 0x00;
    else if( robotCommand == ROBOT_REQ_OFFSET ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_OFFSET ) ? ROBOT_REP_OFFSETTING : 0x00;
    else if( robotCommand == ROBOT_REQ_CALIBRATE ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_CALIBRATION ) ? ROBOT_REP_CALIBRATING : 0x00;
    else if( robotCommand == ROBOT_REQ_PREPROCESS ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_PREPROCESSING ) ? ROBOT_REP_PREPROCESSING : 0x00;
    else if( robotCommand == ROBOT_REQ_OPERATE ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_OPERATION ) ? ROBOT_REP_OPERATING : 0x00;
    else if( robotCommand == ROBOT_REQ_SET_USER )
    {
      char* userName = (char*) messageIn;
      Log_SetBaseName( userName );
      messageOut[ 0 ] = ROBOT_REP_USER_SET;
      //DEBUG_PRINT( "New user name: %s", userName );
    }
    else if( robotCommand == ROBOT_REQ_SET_CONFIG )
    {
      char* robotName = (char*) messageIn;
      RefreshRobotsInfo( robotName, (char*) ( messageOut + 1 ) );
      messageOut[ 0 ] = ROBOT_REP_CONFIG_SET;
    }
    
    IPC_WriteMessage( robotEventsConnection, messageOut );
  }   
}

bool UpdateAxes( unsigned long lastNetworkUpdateElapsedTimeMS )
{
  static Byte message[ IPC_MAX_MESSAGE_LENGTH ];

  Byte* messageIn = (Byte*) message;
  while( IPC_ReadMessage( robotAxesConnection, messageIn ) ) 
  {
    size_t setpointBlocksNumber = (size_t) *(messageIn++);
    //DEBUG_PRINT( "received message for %lu axes", setpointBlocksNumber );
    for( size_t setpointBlockIndex = 0; setpointBlockIndex < setpointBlocksNumber; setpointBlockIndex++ )
    {
      size_t axisIndex = (size_t) *(messageIn++);
      
      if( axisIndex >= axesNumber ) continue;      
      
      float* axisSetpointsList = (float*) messageIn;
      RobotVariables axisSetpoints = { .position = axisSetpointsList[ DOF_POSITION ], .velocity = axisSetpointsList[ DOF_VELOCITY ],
                                       .acceleration = axisSetpointsList[ DOF_ACCELERATION ], .force = axisSetpointsList[ DOF_FORCE ],
                                       .inertia = axisSetpointsList[ DOF_INERTIA ],
                                       .stiffness = axisSetpointsList[ DOF_STIFFNESS ], .damping = axisSetpointsList[ DOF_DAMPING ] };
      //if( axisIndex == 0 ) DEBUG_PRINT( "setpoints: p: %.3f - v: %.3f", axisSetpoints.position, axisSetpoints.velocity );
      Robot_SetAxisSetpoints( axisIndex, &axisSetpoints );

      messageIn += DOF_DATA_BLOCK_SIZE;
    }
  }
  
  memset( message, 0, IPC_MAX_MESSAGE_LENGTH * sizeof(Byte) );
  size_t axisdataOffset = 1;
  for( size_t axisIndex = 0; axisIndex < axesNumber; axisIndex++ )
  {    
    RobotVariables axisMeasures = { 0 };
    if( Robot_GetAxisMeasures( axisIndex, &axisMeasures ) )
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
      
      //if( axisIndex == 0 ) DEBUG_PRINT( "measures: p: %+.5f, v: %+.5f, f: %+.5f", axisMeasuresList[ DOF_POSITION ], axisMeasuresList[ DOF_VELOCITY ], axisMeasuresList[ DOF_FORCE ] );
    
      axisdataOffset += DOF_DATA_BLOCK_SIZE;
    }
  }
  
  if( message[ 0 ] > 0 && lastNetworkUpdateElapsedTimeMS >= NETWORK_UPDATE_MIN_INTERVAL_MS )
  {
    //DEBUG_PRINT( "sending measures from %lu axes", message[ 0 ] );
    IPC_WriteMessage( robotAxesConnection, (const Byte*) message );
    return true;
  }
  
  return false;
}

bool UpdateJoints( unsigned long lastNetworkUpdateElapsedTimeMS )
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
    if( Robot_GetJointMeasures( jointIndex, &jointMeasures ) )
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
  
  if( messageOut[ 0 ] > 0 && lastNetworkUpdateElapsedTimeMS >= NETWORK_UPDATE_MIN_INTERVAL_MS )
  {
    //DEBUG_UPDATE( "sending measures for %u joints", messageOut[ 0 ] );
    IPC_WriteMessage( robotJointsConnection, (const Byte*) messageOut );
    return true;
  }
  
  return false;
}

void System_Update()
{
  unsigned long lastUpdateElapsedTimeMS = Time_GetExecMilliseconds() - lastUpdateTimeMS;
  lastUpdateTimeMS = Time_GetExecMilliseconds();
  
  UpdateEvents();
  
  lastNetworkUpdateElapsedTimeMS += lastUpdateElapsedTimeMS;
  if( UpdateAxes( lastNetworkUpdateElapsedTimeMS ) || UpdateJoints( lastNetworkUpdateElapsedTimeMS ) )
    lastNetworkUpdateElapsedTimeMS = 0;
}


void RefreshRobotsInfo( const char* robotName, char* sharedControlsString )
{ 
  if( robotName != NULL )
  {     
    if( robotInitialized ) Robot_End();
    
    robotInfo = DataIO_CreateEmptyData();
    
    if( (robotInitialized = Robot_Init( robotName )) )
    {
      DataIO_SetStringValue( robotInfo, "id", robotName );   
      
      DataHandle sharedJointsList = DataIO_AddList( robotInfo, "joints" );
      DataHandle sharedAxesList = DataIO_AddList( robotInfo, "axes" );
      
      axesNumber = Robot_GetAxesNumber(); 

      for( size_t axisIndex = 0; axisIndex < axesNumber; axisIndex++ )
      {
        const char* axisName = Robot_GetAxisName( axisIndex );
        if( axisName != NULL ) DataIO_SetStringValue( sharedAxesList, NULL, axisName );
      }
      
      jointsNumber = Robot_GetJointsNumber();

      for( size_t jointIndex = 0; jointIndex < jointsNumber; jointIndex++ )
      {
        const char* jointName = Robot_GetJointName( jointIndex );
        if( jointName != NULL ) DataIO_SetStringValue( sharedJointsList, NULL, jointName );
      }
    }
  }
  
  if( sharedControlsString != NULL )
  {
    char* robotControlsString = DataIO_GetDataString( robotInfo );
    DEBUG_PRINT( "robots info string: %s", robotControlsString );
    strncpy( sharedControlsString, robotControlsString, strlen( robotControlsString ) );
    free( robotControlsString );
  }
}
