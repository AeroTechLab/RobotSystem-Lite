////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2025 Leonardo Consoni <leonardojc@protonmail.com>      //
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


#include "output.h"

#include "signal_io/signal_io.h"
#include "debug/data_logging.h"
      
#include "config_keys.h" 

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
      
struct _OutputData
{
  DECLARE_MODULE_INTERFACE_REF( SIGNAL_IO_INTERFACE );
  long int deviceID;
  unsigned int channel;
};


Output Output_Init( DataHandle configuration )
{
  if( configuration == NULL ) return NULL;
  
  Output newOutput = (Output) malloc( sizeof(OutputData) );
  memset( newOutput, 0, sizeof(OutputData) );

  newOutput->deviceID = SIGNAL_IO_DEVICE_INVALID_ID;
  
  bool loadSuccess = true;
  char filePath[ DATA_IO_MAX_PATH_LENGTH ];
  sprintf( filePath, KEY_MODULES "/" KEY_SIGNAL_IO "/%s", DataIO_GetStringValue( configuration, "", KEY_INTERFACE "." KEY_TYPE ) );
  //DEBUG_PRINT( "trying to read signal IO module %s", filePath );
  LOAD_MODULE_IMPLEMENTATION( SIGNAL_IO_INTERFACE, filePath, newOutput, &loadSuccess );
  if( loadSuccess )
  {
    //PRINT_PLUGIN_FUNCTIONS( SIGNAL_IO_INTERFACE, newOutput );
    newOutput->deviceID = newOutput->InitDevice( DataIO_GetStringValue( configuration, "", KEY_INTERFACE "." KEY_CONFIG ) );
    if( newOutput->deviceID != SIGNAL_IO_DEVICE_INVALID_ID ) 
    {
      newOutput->channel = (unsigned int) DataIO_GetNumericValue( configuration, -1, KEY_INTERFACE "." KEY_CHANNEL );
      //DEBUG_PRINT( "trying to aquire channel %u from interface %d", newOutput->channel, newOutput->deviceID );
      //loadSuccess = newOutput->AcquireOutputChannel( newOutput->deviceID, newOutput->channel );
    }
    else loadSuccess = false;
  }
  
  if( !loadSuccess )
  {
    Output_End( newOutput );
    return NULL;
  }
  
  return newOutput;
}

void Output_End( Output output )
{
  if( output == NULL ) return;
  
  if( output->EndDevice != NULL ) output->EndDevice( output->deviceID );
  
  free( output );
}

bool Output_Enable( Output output )
{
  if( output == NULL ) return false;
  DEBUG_PRINT( "acquiring output %u from interface %d", output->channel, output->deviceID );  
  return output->AcquireOutputChannel( output->deviceID, output->channel );
}

void Output_Disable( Output output )
{
  if( output == NULL ) return;
  
  output->ReleaseOutputChannel( output->deviceID, output->channel );
}

void Output_Reset( Output output )
{
  if( output == NULL ) return;
  DEBUG_PRINT( "resetting interface %d", output->deviceID );
  output->Reset( output->deviceID );
}

bool Output_HasError( Output output )
{
  if( output == NULL ) return true;
  
  return output->HasError( output->deviceID );
}

void Output_Update( Output output, double value )
{
  if( output == NULL ) return;
  //DEBUG_PRINT( "evaluating transform function %p", output->transformFunction );
  output->Write( output->deviceID, output->channel, value );
}
