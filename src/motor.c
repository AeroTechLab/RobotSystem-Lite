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


#include "motor.h"

#include "sensor.h"

#include "signal_io/signal_io.h"
#include "debug/data_logging.h"
#include "timing/timing.h"
      
#include "config_keys.h" 

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
      
struct _MotorData
{
  DECLARE_MODULE_INTERFACE_REF( SIGNAL_IO_INTERFACE );
  int interfaceID;
  unsigned int outputChannel;
  double outputGain;
  Sensor reference;
  double outputOffset;
  bool isOffsetting;
  Log log;
};


Motor Motor_Init( DataHandle configuration )
{
  static char filePath[ DATA_IO_MAX_PATH_LENGTH ];
  
  if( configuration == NULL ) return NULL;
  
  const char* motorName = DataIO_GetStringValue( configuration, NULL, "" );
  if( motorName != NULL )
  {
    DEBUG_PRINT( "trying to create motor %s", motorName );
    sprintf( filePath, KEY_CONFIG "/" KEY_MOTOR "/%s", motorName );
    if( (configuration = DataIO_LoadStorageData( filePath )) == NULL ) return NULL;
  }
  
  Motor newMotor = (Motor) malloc( sizeof(MotorData) );
  memset( newMotor, 0, sizeof(MotorData) );

  bool loadSuccess = true;
  sprintf( filePath, KEY_MODULES "/" KEY_SIGNAL_IO "/%s", DataIO_GetStringValue( configuration, "", KEY_OUTPUT_INTERFACE "." KEY_TYPE ) );
  LOAD_MODULE_IMPLEMENTATION( SIGNAL_IO_INTERFACE, filePath, newMotor, &loadSuccess );
  if( loadSuccess )
  {
    newMotor->interfaceID = newMotor->InitDevice( DataIO_GetStringValue( configuration, "", KEY_OUTPUT_INTERFACE "." KEY_CONFIG ) );
    if( newMotor->interfaceID != SIGNAL_IO_DEVICE_INVALID_ID ) 
    {
      newMotor->outputChannel = (unsigned int) DataIO_GetNumericValue( configuration, -1, KEY_OUTPUT_INTERFACE "." KEY_CHANNEL );
      //DEBUG_PRINT( "trying to aquire channel %u from interface %d", newMotor->outputChannel, newMotor->interfaceID );
      //loadSuccess = newMotor->AcquireOutputChannel( newMotor->interfaceID, newMotor->outputChannel );
    }
    else loadSuccess = false;
  }
  
  newMotor->outputGain = DataIO_GetNumericValue( configuration, 1.0, KEY_OUTPUT_GAIN "." KEY_MULTIPLIER );
  newMotor->outputGain /= DataIO_GetNumericValue( configuration, 1.0, KEY_OUTPUT_GAIN "." KEY_DIVISOR );
  
  DataHandle referenceConfiguration = DataIO_GetSubData( configuration, KEY_REFERENCE );
  newMotor->reference = Sensor_Init( referenceConfiguration );
  if( referenceConfiguration != NULL ) DataIO_UnloadData( referenceConfiguration );
  
  newMotor->isOffsetting = false;
  
  if( DataIO_HasKey( configuration, KEY_LOG ) )
  {
    const char* logFileName = DataIO_GetStringValue( configuration, "", KEY_LOG "." KEY_FILE );
    newMotor->log = Log_Init( logFileName, (size_t) DataIO_GetNumericValue( configuration, 3, KEY_LOG "." KEY_PRECISION ) );
  }
  
  if( motorName != NULL ) DataIO_UnloadData( configuration );
  
  if( !loadSuccess )
  {
    Motor_End( newMotor );
    return NULL;
  }
  
  return newMotor;
}

void Motor_End( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->EndDevice( motor->interfaceID );
  
  Log_End( motor->log );
  
  free( motor );
}

bool Motor_Enable( Motor motor )
{
  if( motor == NULL ) return false;
  
  motor->Reset( motor->interfaceID );
  return motor->AcquireOutputChannel( motor->interfaceID, motor->outputChannel );
}

void Motor_Disable( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->ReleaseOutputChannel( motor->interfaceID, motor->outputChannel );
}

void Motor_Reset( Motor motor )
{
  if( motor == NULL ) return;
  
  motor->Reset( motor->interfaceID );
}

bool Motor_HasError( Motor motor )
{
  if( motor == NULL ) return false;
  
  return motor->HasError( motor->interfaceID );
}

void Motor_SetOffset( Motor motor, bool enabled )
{
  if( motor == NULL ) return;
  
  motor->outputOffset = 0.0;
  if( motor->isOffsetting ) motor->outputOffset = Sensor_Update( motor->reference );
  motor->isOffsetting = enabled;
  
  Sensor_SetState( motor->reference, enabled ? SENSOR_STATE_OFFSET : SENSOR_STATE_MEASUREMENT );
  
  Motor_WriteControl( motor, 0.0 );
}

void Motor_WriteControl( Motor motor, double setpoint )
{
  if( motor == NULL ) return;
  
  Log_EnterNewLine( motor->log, Time_GetExecSeconds() );
  Log_RegisterValues( motor->log, 2, setpoint, setpoint * motor->outputGain );
  
  setpoint = ( setpoint + motor->outputOffset ) * motor->outputGain;

  if( ! motor->isOffsetting ) motor->Write( motor->interfaceID, motor->outputChannel, setpoint );
}
