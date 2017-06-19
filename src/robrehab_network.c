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


#include <stdint.h>
#include <time.h>

#include "shm_control.h"
#include "shared_axis_data.h"
#include "shared_joint_data.h"

#include "utils/klib/kvec.h"
#include "utils/async_ip_network.h"
#include "utils/debug/async_debug.h"

#include "robrehab_subsystem.h"


const unsigned long UPDATE_INTERVAL_MS = 10;


static unsigned long eventServerConnectionID = IP_CONNECTION_INVALID_ID;
static unsigned long axisServerConnectionID = IP_CONNECTION_INVALID_ID;
static unsigned long jointServerConnectionID = IP_CONNECTION_INVALID_ID;

static kvec_t( unsigned long ) eventClientsList;
const size_t INFO_BLOCK_SIZE = 2;

static kvec_t( unsigned long ) axisClientsList;
static kvec_t( unsigned long ) jointClientsList;

SHMController sharedRobotsInfo;
SHMController sharedRobotAxesData;
SHMController sharedRobotJointsData;

static kvec_t( unsigned long ) axisControlClientsList;
static size_t jointControlClientsNumber = 0;

DEFINE_NAMESPACE_INTERFACE( SubSystem, ROBREHAB_SUBSYSTEM_INTERFACE );

static void RefreshRobotsInfo( char* );

bool SubSystem_Init( const int argc, const char** argv )
{
  const char* multicastHost = /*( argc > 2 ) ? argv[ 1 ] :*/ NULL;
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "Initializing RobRehab Network on thread %lx", THREAD_ID );
  
  if( (eventServerConnectionID = AsyncIPNetwork.OpenConnection( IP_SERVER | IP_TCP, NULL, 50000 )) == IP_CONNECTION_INVALID_ID )
    return false;
  if( (axisServerConnectionID = AsyncIPNetwork.OpenConnection( IP_SERVER | IP_UDP, multicastHost, 50001 )) == IP_CONNECTION_INVALID_ID )
    return false;
  if( (jointServerConnectionID = AsyncIPNetwork.OpenConnection( IP_SERVER | IP_UDP, multicastHost, 50002 )) == IP_CONNECTION_INVALID_ID )
    return false;
  
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "Received server connection IDs: %lu (Info) - %lu (Data) - %lu(joint)", eventServerConnectionID, axisServerConnectionID, jointServerConnectionID );
  
  kv_init( eventClientsList );
  kv_init( axisClientsList );
  kv_init( jointClientsList );
  
  kv_init( axisControlClientsList );
  for( size_t axisIndex = 0; axisIndex < kv_size( axisControlClientsList ); axisIndex++ )
    kv_A( axisControlClientsList, axisIndex ) = IP_CONNECTION_INVALID_ID;
  
  sharedRobotsInfo = SHMControl.InitData( "robots_info", SHM_CONTROL_OUT );
  sharedRobotAxesData = SHMControl.InitData( "robot_axes_data", SHM_CONTROL_OUT );
  sharedRobotJointsData = SHMControl.InitData( "robot_joints_data", SHM_CONTROL_OUT );
  
  char robotsInfoString[ SHM_CONTROL_MAX_DATA_SIZE ];
  RefreshRobotsInfo( robotsInfoString );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network initialized on thread %lx", THREAD_ID );
  
  return true;
}

void SubSystem_End()
{
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "ending RobRehab Network on thread %lx", THREAD_ID );
  
  for( size_t eventClientIndex = 0; eventClientIndex < kv_size( eventClientsList ); eventClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( eventClientsList, eventClientIndex ) );
  AsyncIPNetwork.CloseConnection( eventServerConnectionID );
  /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "info server %lu closed", eventServerConnectionID );
  
  for( size_t axisClientIndex = 0; axisClientIndex < kv_size( axisClientsList ); axisClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( axisClientsList, axisClientIndex ) );
  AsyncIPNetwork.CloseConnection( axisServerConnectionID );
  /*DEBUG_EVENT( 2,*/DEBUG_PRINT( "data server %lu closed", axisServerConnectionID );
  
  for( size_t emgClientIndex = 0; emgClientIndex < kv_size( jointClientsList ); emgClientIndex++ )
    AsyncIPNetwork.CloseConnection( kv_A( jointClientsList, emgClientIndex ) );
  AsyncIPNetwork.CloseConnection( jointServerConnectionID );
  /*DEBUG_EVENT( 3,*/DEBUG_PRINT( "joint server %lu closed", jointServerConnectionID );
  
  SHMControl.EndData( sharedRobotsInfo );
  SHMControl.EndData( sharedRobotAxesData );
  SHMControl.EndData( sharedRobotJointsData );
  
  kv_destroy( eventClientsList );
  DEBUG_EVENT( 6, "info clients list %p destroyed", eventClientsList );
  kv_destroy( axisClientsList );
  DEBUG_EVENT( 7, "data clients list %p destroyed", axisClientsList );
  kv_destroy( jointClientsList );
  DEBUG_EVENT( 8, "joint clients list %p destroyed", jointClientsList );
  
  kv_destroy( axisControlClientsList );
  
  /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "RobRehab Network ended on thread %lx", THREAD_ID );
}

static void UpdateClientEvent( unsigned long );
static void UpdateClientAxis( unsigned long );
static void UpdateClientJoint( unsigned long );

void SubSystem_Update()
{
  DEBUG_UPDATE( "updating connections on thread %lx", THREAD_ID );
  
  unsigned long newEventClientID = AsyncIPNetwork.GetClient( eventServerConnectionID );
  if( newEventClientID != IP_CONNECTION_INVALID_ID )
  {
    /*DEBUG_EVENT( 0,*/DEBUG_PRINT( "new info client found: %lu", newEventClientID );
    kv_push( unsigned long, eventClientsList, newEventClientID );
  }
  
  unsigned long newAxisClientID = AsyncIPNetwork.GetClient( axisServerConnectionID );
  if( newAxisClientID != IP_CONNECTION_INVALID_ID )
  {
    /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "new axis client found: %lu", newAxisClientID );
    kv_push( unsigned long, axisClientsList, newAxisClientID );
  }
  
  unsigned long newjointClientID = AsyncIPNetwork.GetClient( jointServerConnectionID );
  if( newjointClientID != IP_CONNECTION_INVALID_ID )
  {
    /*DEBUG_EVENT( 1,*/DEBUG_PRINT( "new joint client found: %lu", newjointClientID );
    kv_push( unsigned long, jointClientsList, newjointClientID );
  }
  
  for( size_t clientIndex = 0; clientIndex < kv_size( eventClientsList ); clientIndex++ )
    UpdateClientEvent( kv_A( eventClientsList, clientIndex ) );
  
  for( size_t clientIndex = 0; clientIndex < kv_size( axisClientsList ); clientIndex++ )
    UpdateClientAxis( kv_A( axisClientsList, clientIndex ) );

  for( size_t clientIndex = 0; clientIndex < kv_size( jointClientsList ); clientIndex++ )
    UpdateClientJoint( kv_A( jointClientsList, clientIndex ) );
}

void RefreshRobotsInfo( char* infoBuffer )
{
  static uint8_t listRequestsCount;

  SHMControl.SetControlByte( sharedRobotsInfo, SHM_CONTROL_MASK_SIZE - 1, ++listRequestsCount );

  Timing.Delay( 100 );

  SHMControl.GetData( sharedRobotsInfo, (void*) infoBuffer, 0, SHM_CONTROL_MAX_DATA_SIZE );
  memset( infoBuffer + strlen(infoBuffer), 0, SHM_CONTROL_MAX_DATA_SIZE - strlen(infoBuffer) );

  uint8_t axesNumber = SHMControl.GetControlByte( sharedRobotAxesData, SHM_CONTROL_MASK_SIZE - 1, SHM_CONTROL_PEEK );
  kv_resize( unsigned long, axisControlClientsList, (size_t) axesNumber );
  
  jointControlClientsNumber = SHMControl.GetControlByte( sharedRobotJointsData, SHM_CONTROL_MASK_SIZE - 1, SHM_CONTROL_PEEK );  

  //DEBUG_PRINT( "received info string: %s (%u axes, %u joints)", infoBuffer, axesNumber, jointsNumber );
}

void UpdateClientEvent( unsigned long clientID )
{
  static char messageOut[ IP_MAX_MESSAGE_LENGTH ];
  
  char* messageIn = AsyncIPNetwork.ReadMessage( clientID );
  if( messageIn != NULL ) 
  {
    uint8_t commandBlocksNumber = (uint8_t) *(messageIn++);
    
    if( commandBlocksNumber == 0x00 )
    {
      //DEBUG_PRINT( "info string request: %u", commandBlocksNumber );
      memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );      
      RefreshRobotsInfo( messageOut );
      AsyncIPNetwork.WriteMessage( clientID, messageOut );
    }
    else
    {
      for( uint8_t commandBlockIndex = 0; commandBlockIndex < commandBlocksNumber; commandBlockIndex++ )
      {
        uint8_t robotIndex = (uint8_t) *(messageIn++);
        uint8_t command = (uint8_t) *(messageIn++);
        DEBUG_PRINT( "received robot %u command: %u", robotIndex, command );
      
        SHMControl.SetControlByte( sharedRobotsInfo, robotIndex, command );
      }
    }
  }
}

void UpdateClientAxis( unsigned long clientID )
{
  static char messageOut[ IP_MAX_MESSAGE_LENGTH ];
  
  char* messageIn = AsyncIPNetwork.ReadMessage( clientID );
  if( messageIn != NULL ) 
  {
    size_t setpointBlocksNumber = (size_t) *(messageIn++);
    //DEBUG_PRINT( "received message from client %lu for %lu axes", clientID, setpointBlocksNumber );
    for( size_t setpointBlockIndex = 0; setpointBlockIndex < setpointBlocksNumber; setpointBlockIndex++ )
    {
      size_t axisIndex = (size_t) *(messageIn++);
      
      DEBUG_PRINT( "receiving for axis %lu (of %lu)", axisIndex, kv_max( axisControlClientsList ) );

      if( axisIndex >= kv_max( axisControlClientsList ) ) continue;      

      if( kv_A( axisControlClientsList, axisIndex ) == IP_CONNECTION_INVALID_ID ) kv_A( axisControlClientsList, axisIndex ) = clientID;
      
      //if( kv_A( axisControlClientsList, axisIndex ) == clientID )  
      //{
        //DEBUG_PRINT( "receiving axis %lu setpoints", axisIndex );
        //SHMControl.SetControlByte( sharedRobotAxesData, axisIndex, axisMask );
        SHMControl.SetData( sharedRobotAxesData, messageIn, axisIndex * AXIS_DATA_BLOCK_SIZE, AXIS_DATA_BLOCK_SIZE );
      //}

      messageIn += AXIS_DATA_BLOCK_SIZE;
    }
  }
  
  memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
  size_t axisdataOffset = 1;
  for( size_t axisIndex = 0; axisIndex < kv_max( axisControlClientsList ); axisIndex++ )
  {
    if( SHMControl.GetControlByte( sharedRobotAxesData, axisIndex, SHM_CONTROL_PEEK ) )
    {
      messageOut[ 0 ]++;
      messageOut[ axisdataOffset++ ] = (uint8_t) axisIndex;
    
      SHMControl.GetData( sharedRobotAxesData, messageOut + axisdataOffset, axisIndex * AXIS_DATA_BLOCK_SIZE, AXIS_DATA_BLOCK_SIZE );
      axisdataOffset += AXIS_DATA_BLOCK_SIZE;
    }
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    //DEBUG_PRINT( "sending %u axes for client %lu", messageOut[ 0 ], clientID );
    AsyncIPNetwork.WriteMessage( clientID, messageOut );
  }
}


void UpdateClientJoint( unsigned long clientID )
{
  static char messageOut[ IP_MAX_MESSAGE_LENGTH ];
  
  memset( messageOut, 0, IP_MAX_MESSAGE_LENGTH * sizeof(char) );
  size_t jointDataOffset = 1;
  for( size_t jointIndex = 0; jointIndex < jointControlClientsNumber; jointIndex++ )
  {
    if( SHMControl.GetControlByte( sharedRobotJointsData, jointIndex, SHM_CONTROL_REMOVE ) )
    {
      messageOut[ 0 ]++;
      messageOut[ jointDataOffset++ ] = (uint8_t) jointIndex;
    
      SHMControl.GetData( sharedRobotJointsData, messageOut + jointDataOffset, jointIndex * JOINT_DATA_BLOCK_SIZE, JOINT_DATA_BLOCK_SIZE );
      jointDataOffset += JOINT_DATA_BLOCK_SIZE;
    }
  }
  
  if( messageOut[ 0 ] > 0 ) 
  {
    DEBUG_UPDATE( "sending measures for %u joints", messageOut[ 0 ] );
    AsyncIPNetwork.WriteMessage( clientID, messageOut );
  }
}
