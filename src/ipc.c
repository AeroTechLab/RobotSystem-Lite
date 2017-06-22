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


#include "ipc.h"

#include <zmq.h>

#include <stdlib.h>
#include <string.h>

struct _IPCConnectionData
{
  zmq_pollitem_t poller;
  RemoteID* remoteIDsList;
  size_t remotesCount;
  size_t identityLength;
  size_t messageLength;
};

void* context = NULL;

int activeConnectionsNumber = 0;

size_t IPC_GetActivesNumber()
{
  return (size_t) activeConnectionsNumber;
}

IPCConnection IPC_OpenConnection( Byte flags, const char* host, uint16_t channel )
{
  const Byte TRANSPORT_MASK = 0xF0, ROLE_MASK = 0x0F;
  
  static char zmqAddress[ 256 ];
  
  if( context == NULL ) context = zmq_ctx_new();
  
  IPCConnection newConnection = (IPCConnection) malloc( sizeof(IPCConnectionData) );
  
  newConnection->poller.socket = NULL;
  if( (flags & TRANSPORT_MASK) == IPC_TCP )
  {
    sprintf( zmqAddress, "tcp://%s:%u", ( host != NULL ) ? host : "*", channel );
    newConnection->poller.socket = zmq_socket( context, ZMQ_STREAM );
  }
#ifdef ZMQ_BUILD_DRAFT_API
  else if( (flags & TRANSPORT_MASK) == IPC_UDP )
  {
    sprintf( zmqAddress, "udp://%s:%u", ( host != NULL ) ? host : "*", channel );
    newConnection->poller.socket = zmq_socket( context, ZMQ_DGRAM );    
  }
#endif
  
  printf( "socket created on port %u: %p\n", channel, newConnection->poller.socket );
  
  if( newConnection->poller.socket == NULL ) 
  {
    free( newConnection );
    return NULL;
  }
  
  int status = 0;
  if( (flags & ROLE_MASK) == IPC_SERVER ) status = zmq_bind( newConnection->poller.socket, zmqAddress );
  else status = zmq_connect( newConnection->poller.socket, zmqAddress );
  
  printf( "socket %p binding status: %d\n", newConnection->poller.socket, status );
  
  if( status == -1 ) 
  {
    free( newConnection );
    return NULL;
  }
  
  newConnection->poller.events = ZMQ_POLLIN;
  
  newConnection->remoteIDsList = NULL;
  newConnection->remotesCount = 0;
  
  newConnection->messageLength = IPC_MAX_MESSAGE_LENGTH;
  
  int multicastHops;
  size_t optionLength;
  bool isMulticast = (bool) ( zmq_getsockopt( newConnection->poller.socket, ZMQ_MULTICAST_HOPS, &multicastHops, &optionLength ) == 0 );
  
  if( (flags & ROLE_MASK) == IPC_CLIENT || isMulticast ) 
  {
    newConnection->remoteIDsList = (RemoteID*) malloc( sizeof(RemoteID) );
    newConnection->remotesCount = 1;
    
    zmq_getsockopt( newConnection->poller.socket, ZMQ_IDENTITY, &(newConnection->remoteIDsList[ 0 ]), &(newConnection->identityLength) );
  }
  
  char address[ 256 ];
  size_t addrLength = 256;
  zmq_getsockopt( newConnection->poller.socket, ZMQ_LAST_ENDPOINT, address, &addrLength );
  printf( "socket %p address: %s\n", newConnection->poller.socket, address );
  
  activeConnectionsNumber++;
  
  return newConnection;
}

void IPC_CloseConnection( IPCConnection connection )
{
   if( connection == NULL ) return;
   
   for( size_t remoteIndex = 0; remoteIndex < connection->remotesCount; remoteIndex++ )
   {
     zmq_send( connection->poller.socket, connection->remoteIDsList[ remoteIndex ], connection->identityLength, ZMQ_SNDMORE );
     zmq_send( connection->poller.socket, NULL, 0, 0 );
   }
   
   zmq_close( connection->poller.socket );
   
   if( connection->remoteIDsList != NULL ) free( connection->remoteIDsList );
   
   free( connection );
   
   if( --activeConnectionsNumber <= 0 ) 
   {
     if( context != NULL ) zmq_ctx_destroy( context );
     context = NULL;
   }
}

size_t IPC_SetMessageLength( IPCConnection connection, size_t messageLength )
{
  if( connection == NULL ) return 0;
  
  if( messageLength > IPC_MAX_MESSAGE_LENGTH ) messageLength = IPC_MAX_MESSAGE_LENGTH;
  
  connection->messageLength = messageLength;
  
  return connection->messageLength;
}

bool IPC_ReadMessage( IPCConnection connection, Byte* message, RemoteID* ref_remoteID )
{
  static RemoteID remoteID;
  
  if( connection == NULL ) return false;
  
  if( ref_remoteID == NULL ) ref_remoteID = &remoteID;
  
  if( zmq_poll( &(connection->poller), 1, 0 ) <= 0 ) return false;
  
  if( connection->poller.revents != ZMQ_POLLIN ) return false;
  
  printf( "reading from socket %p\r", connection->poller.socket );
  
  if( zmq_recv( connection->poller.socket, ref_remoteID, connection->identityLength, 0 ) <= 0 ) return false;
  if( zmq_recv( connection->poller.socket, message, connection->messageLength, 0 ) <= 0 ) return false;
  
  printf( "%x\r", message[ 0 ] );
  
  for( size_t remoteIndex = 0; remoteIndex < connection->remotesCount; remoteIndex++ )
  {
    RemoteID* currentRemoteID = &(connection->remoteIDsList[ remoteIndex ]);
    if( memcmp( ref_remoteID, currentRemoteID, connection->identityLength ) == 0 )
    {
      connection->remoteIDsList = realloc( connection->remoteIDsList, ( connection->remotesCount + 1 ) * sizeof(RemoteID) );
      memcpy( &(connection->remoteIDsList[ connection->remotesCount++ ] ), ref_remoteID, connection->identityLength );
      
      break;
    }
  }
  
  return true;
}

bool IPC_WriteMessage( IPCConnection connection, const Byte* message, const RemoteID* ref_remoteID )
{ 
  if( connection == NULL ) return false;
  
  if( ref_remoteID != NULL )
  {
    if( zmq_send( connection->poller.socket, ref_remoteID, connection->identityLength, ZMQ_SNDMORE ) < 0 ) return false;
    if( zmq_send( connection->poller.socket, message, strlen( (const char*) message ), 0 ) < 0 ) return false;
  }
  else
  {
    for( size_t remoteIndex = 0; remoteIndex < connection->remotesCount; remoteIndex++ )
    {
      zmq_send( connection->poller.socket, connection->remoteIDsList[ remoteIndex ], connection->identityLength, ZMQ_SNDMORE );
      zmq_send( connection->poller.socket, message, strlen( (const char*) message ), 0 );
    }
  }
  
  return true;
}
