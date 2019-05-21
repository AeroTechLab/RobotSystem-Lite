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


#include "input.h"

#include "signal_io/signal_io.h"
#include "debug/data_logging.h"

#include "config_keys.h"

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

struct _InputData
{
  DECLARE_MODULE_INTERFACE_REF( SIGNAL_IO_INTERFACE );
  int deviceID;
  unsigned int channel;
  double* buffer;
  double value;
  SignalProcessor processor;
};


Input Input_Init( DataHandle configuration )
{
  if( configuration == NULL ) return NULL;
  
  //DEBUG_PRINT( "input configuration found on data handle %p", configuration );
  
  Input newInput = (Input) malloc( sizeof(InputData) );
  memset( newInput, 0, sizeof(InputData) ); 
  
  newInput->deviceID = SIGNAL_IO_DEVICE_INVALID_ID;
  
  bool loadSuccess;
  char filePath[ DATA_IO_MAX_PATH_LENGTH ];
  sprintf( filePath, KEY_MODULES "/" KEY_SIGNAL_IO "/%s", DataIO_GetStringValue( configuration, "", KEY_INTERFACE "." KEY_TYPE ) );
  //DEBUG_PRINT( "trying to read signal IO module %s", filePath );
  LOAD_MODULE_IMPLEMENTATION( SIGNAL_IO_INTERFACE, filePath, newInput, &loadSuccess );
  if( loadSuccess )
  {
    //PRINT_PLUGIN_FUNCTIONS( SIGNAL_IO_INTERFACE, newInput );
    newInput->deviceID = newInput->InitDevice( DataIO_GetStringValue( configuration, "", KEY_INTERFACE "." KEY_CONFIG ) );
    if( newInput->deviceID != SIGNAL_IO_DEVICE_INVALID_ID )
    {
      newInput->channel = (unsigned int) DataIO_GetNumericValue( configuration, -1, KEY_INTERFACE "." KEY_CHANNEL );
      loadSuccess = newInput->CheckInputChannel( newInput->deviceID, newInput->channel );
      
      size_t maxInputSamplesNumber = newInput->GetMaxInputSamplesNumber( newInput->deviceID );
      newInput->buffer = (double*) calloc( maxInputSamplesNumber, sizeof(double) );
      
      uint8_t signalProcessingFlags = 0x00;
      if( DataIO_GetBooleanValue( configuration, false, KEY_SIGNAL_PROCESSING "." KEY_RECTIFIED ) ) signalProcessingFlags |= SIG_PROC_RECTIFY;
      if( DataIO_GetBooleanValue( configuration, false, KEY_SIGNAL_PROCESSING "." KEY_NORMALIZED ) ) signalProcessingFlags |= SIG_PROC_NORMALIZE;
      newInput->processor = SignalProcessor_Create( signalProcessingFlags );
          
      double relativeMinCutFrequency = DataIO_GetNumericValue( configuration, 0.0, KEY_SIGNAL_PROCESSING "." KEY_MIN_FREQUENCY );
      SignalProcessor_SetMinFrequency( newInput->processor, relativeMinCutFrequency );
      double relativeMaxCutFrequency = DataIO_GetNumericValue( configuration, 0.0, KEY_SIGNAL_PROCESSING "." KEY_MAX_FREQUENCY );
      SignalProcessor_SetMaxFrequency( newInput->processor, relativeMaxCutFrequency );
      
      newInput->Reset( newInput->deviceID );
    }
  }
  
  if( !loadSuccess )
  {
    Input_End( newInput );
    return NULL;
  } 
  
  return newInput;
}

void Input_End( Input input )
{
  if( input == NULL ) return;
  
  if( input->EndDevice != NULL ) input->EndDevice( input->deviceID );
  
  SignalProcessor_Discard( input->processor );
  
  free( input->buffer );

  free( input );
}

double Input_Update( Input input )
{
  if( input == NULL ) return 0.0;
  
  size_t aquiredSamplesNumber = input->Read( input->deviceID, input->channel, input->buffer );
    
  return SignalProcessor_UpdateSignal( input->processor, input->buffer, aquiredSamplesNumber );
}
  
bool Input_HasError( Input input )
{
  if( input == NULL ) return true;
  
  return input->HasError( input->deviceID );
}

void Input_Reset( Input input )
{
  if( input == NULL ) return;
  
  SignalProcessor_SetState( input->processor, SIG_PROC_STATE_MEASUREMENT );
  input->Reset( input->deviceID );
}

void Input_SetState( Input input, enum SigProcState newProcessingState )
{
  if( input == NULL ) return;
  
  SignalProcessor_SetState( input->processor, newProcessingState );
}
 
