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

#include "ipc/interface/ipc.h"

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
#include "getopt.h"
#define chdir _chdir
#else
#include <unistd.h>
#include <dirent.h>
#include <getopt.h>
#endif

const unsigned long NETWORK_UPDATE_MIN_INTERVAL_MS = 20;
static unsigned long lastUpdateTimeMS = 0;
static unsigned long lastNetworkUpdateElapsedTimeMS = NETWORK_UPDATE_MIN_INTERVAL_MS;


bool robotInitialized = false;
size_t axesNumber = 0, jointsNumber = 0;

DataHandle robotConfig = NULL;

IPCConnection robotEventsConnection = NULL;
IPCConnection robotAxesConnection = NULL;


void ListRobotConfigs( char* );
DataHandle ReloadRobotConfig( const char* );
void GetRobotConfigString( DataHandle, char* );


bool System_Init( const int argc, const char** argv )
{
  DEBUG_PRINT( "Starting Robot Control at time %g", Time_GetExecSeconds() );
  
  const char* rootDirectory = ".";
  const char* connectionAddress = NULL;
  const char* logDirectory = "./" KEY_LOGS "/";
  const char* robotConfigName = NULL;
  
  static struct option longOptions[] =
  {
    { "help", no_argument, NULL, 'h' },
    { "root", required_argument, NULL, 'r' },
    { "log", required_argument, NULL, 'l' },
    { "addr", required_argument, NULL, 'a' },
    { "config", required_argument, NULL, 'c' },
    { NULL, 0, NULL, 0 }
  };
  
  int optionChar;
  int optionIndex;
  while( (optionChar = getopt_long( argc, (char* const*) argv, "hr:l:a:c:", longOptions, &optionIndex )) != -1 )
  {
    DEBUG_PRINT( "option %s(%c) set with argument %s", longOptions[ optionIndex ].name, optionChar, optarg );
    if( optionChar == 'h' )
    {
      printf( "usage: %s [--root <root_dir>] [--addr <connection_address>] [--log <log_dir>] [--config <robot_name>]\n", argv[ 0 ] );
      return false;
    }
    else if( optionChar == 'r' ) rootDirectory = optarg;
    else if( optionChar == 'l' ) logDirectory = optarg;
    else if( optionChar == 'a' ) connectionAddress = optarg;
    else if( optionChar == 'c' ) robotConfigName = optarg;
  }
  
  const char* connectionHost = connectionAddress;
  char* connectionChannel = ( connectionAddress != NULL ) ? strrchr( connectionAddress, ':' ) : NULL;
  if( connectionChannel != NULL ) *(connectionChannel++) = '\0';
  robotEventsConnection = IPC_OpenConnection( IPC_REP, connectionHost, connectionChannel );
  robotAxesConnection = IPC_OpenConnection( IPC_SERVER, connectionHost, connectionChannel );
  
  Log_SetDirectory( logDirectory );

  chdir( rootDirectory );
  DEBUG_PRINT( "loading robot configuration from %s", robotConfigName );
  robotConfig = ReloadRobotConfig( robotConfigName );

  lastUpdateTimeMS = Time_GetExecMilliseconds();
  
  return true;
}

void System_End()
{
  DEBUG_PRINT( "Ending Robot Control at time %g", Time_GetExecSeconds() );

  IPC_CloseConnection( robotEventsConnection );
  IPC_CloseConnection( robotAxesConnection );
//   IPC_CloseConnection( robotJointsConnection );

  DataIO_UnloadData( robotConfig );

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
    if( robotCommand == ROBOT_REQ_LIST_CONFIGS ) 
    {
      messageOut[ 0 ] = ROBOT_REP_CONFIGS_LISTED;
      ListRobotConfigs( (char*) ( messageOut + 1 ) );
    }
    else if( robotCommand == ROBOT_REQ_GET_CONFIG ) 
    {
      messageOut[ 0 ] = ROBOT_REP_GOT_CONFIG;
      GetRobotConfigString( robotConfig, (char*) ( messageOut + 1 ) );
    }
    else if( robotCommand == ROBOT_REQ_SET_CONFIG )
    {
      char* robotName = (char*) messageIn;
      DEBUG_PRINT( "robot config %s set", robotName );
      robotConfig = ReloadRobotConfig( robotName );
      messageOut[ 0 ] = ROBOT_REP_CONFIG_SET;
      GetRobotConfigString( robotConfig, (char*) ( messageOut + 1 ) );
    }
    else 
    {
      if( robotCommand == ROBOT_REQ_SET_USER )
      {
        char* userName = (char*) messageIn;
        DEBUG_PRINT( "new user name: %s", userName );
        Log_SetBaseName( userName );
        messageOut[ 0 ] = ROBOT_REP_USER_SET;
      }
      else if( robotCommand == ROBOT_REQ_DISABLE ) messageOut[ 0 ] = Robot_Disable() ? ROBOT_REP_DISABLED : 0x00;
      else if( robotCommand == ROBOT_REQ_ENABLE ) messageOut[ 0 ] = Robot_Enable() ? ROBOT_REP_ENABLED : 0x00;
      else if( robotCommand == ROBOT_REQ_PASSIVATE ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_PASSIVE ) ? ROBOT_REP_PASSIVE : 0x00;
      else if( robotCommand == ROBOT_REQ_OFFSET ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_OFFSET ) ? ROBOT_REP_OFFSETTING : 0x00;
      else if( robotCommand == ROBOT_REQ_CALIBRATE ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_CALIBRATION ) ? ROBOT_REP_CALIBRATING : 0x00;
      else if( robotCommand == ROBOT_REQ_PREPROCESS ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_PREPROCESSING ) ? ROBOT_REP_PREPROCESSING : 0x00;
      else if( robotCommand == ROBOT_REQ_OPERATE ) messageOut[ 0 ] = Robot_SetControlState( ROBOT_OPERATION ) ? ROBOT_REP_OPERATING : 0x00;
      memset( messageOut + 1, 0, IPC_MAX_MESSAGE_LENGTH - 1 );
    }
    DEBUG_PRINT( "sending robot state: %u", messageOut[ 0 ] );
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

void System_Update()
{
  unsigned long lastUpdateElapsedTimeMS = Time_GetExecMilliseconds() - lastUpdateTimeMS;
  lastUpdateTimeMS = Time_GetExecMilliseconds();
  
  UpdateEvents();
  
  lastNetworkUpdateElapsedTimeMS += lastUpdateElapsedTimeMS;
  if( UpdateAxes( lastNetworkUpdateElapsedTimeMS ) /*|| UpdateJoints( lastNetworkUpdateElapsedTimeMS )*/ )
    lastNetworkUpdateElapsedTimeMS = 0;
}


void ListRobotConfigs( char* sharedRobotsString )
{
  DataHandle robotsList = DataIO_CreateEmptyData();
  
  DataHandle sharedRobotsList = DataIO_AddList( robotsList, KEY_ROBOTS );
  DEBUG_PRINT( "searching robots config in: %s", "./" KEY_CONFIG "/" KEY_ROBOTS "/" );
  const char** dataList = DataIO_ListStorageDataEntries( "./" KEY_CONFIG "/" KEY_ROBOTS "/" );
  for( size_t dataIndex = 0; dataList[ dataIndex ] != NULL; dataIndex++ )
    DataIO_SetStringValue( sharedRobotsList, NULL, dataList[ dataIndex ] );
  
  char* robotsListString = DataIO_GetDataString( robotsList );
  DEBUG_PRINT( "robots info string: %s", robotsListString );
  strncpy( sharedRobotsString, robotsListString, strlen( robotsListString ) );
  free( robotsListString );
  
  DataIO_UnloadData( robotsList );
}

DataHandle ReloadRobotConfig( const char* robotName )
{ 
  DataHandle robotConfig = DataIO_CreateEmptyData();
  
  if( robotName != NULL )
  {     
    Log_SetTimeStamp();
    
    if( robotInitialized ) Robot_End();
    
    if( (robotInitialized = Robot_Init( robotName )) )
    {
      DataIO_SetStringValue( robotConfig, KEY_ID, robotName );   
      
      DataHandle sharedJointsList = DataIO_AddList( robotConfig, KEY_JOINTS );
      DataHandle sharedAxesList = DataIO_AddList( robotConfig, KEY_AXES );
      
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
  
  return robotConfig;
}

void GetRobotConfigString( DataHandle robotConfig, char* sharedControlsString )
{
  if( sharedControlsString != NULL )
  {
    char* robotConfigString = DataIO_GetDataString( robotConfig );
    DEBUG_PRINT( "robots info string: %s", robotConfigString );
    strncpy( sharedControlsString, robotConfigString, strlen( robotConfigString ) );
    free( robotConfigString );
  }
}
