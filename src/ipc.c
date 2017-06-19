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
  void* socket;
  RemoteID* remoteIDsList;
  size_t remotesCount;
  size_t messageLength;
};

void* context = NULL;

int activeConnectionsNumber = 0;

size_t IPC_GetActivesNumber()
{
  return (size_t) activeConnectionsNumber;
}

IPCConnection IPC_OpenConnection( enum IPCTransport transport, const char* host, uint16_t channel )
{
  static char zmqAddress[ 256 ];
  
  if( context == NULL ) context = zmq_ctx_new();
  
  IPCConnection newConnection = (IPCConnection) malloc( sizeof(IPCConnectionData) );
  
  newConnection->socket = NULL;
  if( transport == IPC_TCP )
  {
    sprintf( zmqAddress, "tcp://%s:%u", host, channel );
    newConnection->socket = zmq_socket( context, ZMQ_STREAM );
  }
#ifdef ZMQ_BUILD_DRAFT_API
  else if( transport == IPC_UDP )
  {
    sprintf( zmqAddress, "udp://%s:%u", host, channel );
    newConnection->socket = zmq_socket( context, ZMQ_DGRAM );    
  }
#endif
  
  if( host == NULL ) zmq_bind( newConnection->socket, zmqAddress );
  else zmq_connect( newConnection->socket, zmqAddress );
  
  newConnection->remoteIDsList = NULL;
  newConnection->remotesCount = 0;
  
  newConnection->messageLength = IPC_MAX_MESSAGE_LENGTH;
  
  activeConnectionsNumber++;
  
  return newConnection;
}

void IPC_CloseConnection( IPCConnection connection )
{
   if( connection == NULL ) return;
   
   for( size_t remoteIndex = 0; remoteIndex < connection->remotesCount; remoteIndex++ )
   {
     zmq_send( connection->socket, connection->remoteIDsList[ remoteIndex ], IPC_ID_LENGTH, ZMQ_SNDMORE );
     zmq_send( connection->socket, NULL, 0, 0 );
   }
   
   zmq_close( connection->socket );
   
   free( connection->remoteIDsList );
   
   free( connection );
   
   if( --activeConnectionsNumber <= 0 ) 
   {
     zmq_ctx_destroy( context );
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
  if( connection == NULL ) return false;
  
  if( zmq_recv( connection->socket, ref_remoteID, IPC_ID_LENGTH, 0 ) < 0 ) return false;
  if( zmq_recv( connection->socket, message, connection->messageLength, 0 ) < 0 ) return false;
  
  return true;
}

bool IPC_WriteMessage( IPCConnection connection, const Byte* message, RemoteID remoteID )
{
  if( connection == NULL ) return false;
  
  if( remoteID != NULL )
  {
    if( zmq_send( connection->socket, remoteID, IPC_ID_LENGTH, ZMQ_SNDMORE ) < 0 ) return false;
    if( zmq_send( connection->socket, message, strlen( (const char*) message ), 0 ) < 0 ) return false;
  }
  else
  {
    for( size_t remoteIndex = 0; remoteIndex < connection->remotesCount; remoteIndex++ )
    {
      zmq_send( connection->socket, connection->remoteIDsList[ remoteIndex ], IPC_ID_LENGTH, ZMQ_SNDMORE );
      zmq_send( connection->socket, message, strlen( (const char*) message ), 0 );
    }
  }
  
  return true;
}
