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

#include "input.h"
#include "output.h"
#include "tinyexpr/tinyexpr.h"

#include "data_io/interface/data_io.h"
#include "signal_io/signal_io.h"
#include "debug/data_logging.h"
#include "timing/timing.h"
      
#include "config_keys.h" 

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
      
const char* SETPOINT_VARIABLE_NAME = "set";
const char* REFERENCE_VARIABLE_NAME = "ref";
      
struct _MotorData
{
  Output output;
  Input reference;
  double setpoint, offset;
  te_variable inputVariables[ 2 ];
  te_expr* transformFunction;
  bool isOffsetting;
  Log log;
};


Motor Motor_Init( const char* configName )
{
  char filePath[ DATA_IO_MAX_PATH_LENGTH ];
  DEBUG_PRINT( "trying to create motor %s", configName );
  sprintf( filePath, KEY_CONFIG "/" KEY_MOTORS "/%s", configName );
  DataHandle configuration = DataIO_LoadStorageData( filePath );
  if( configuration == NULL ) return NULL;
  
  Motor newMotor = (Motor) malloc( sizeof(MotorData) );
  memset( newMotor, 0, sizeof(MotorData) );

  newMotor->output = Output_Init( configuration );
  
  bool loadSuccess = ( newMotor->output != NULL ) ? true : false;
  
  newMotor->reference = Input_Init( DataIO_GetSubData( configuration, KEY_REFERENCE ) );
  newMotor->isOffsetting = false;
  DEBUG_PRINT( "reference input: %p", newMotor->reference );
  int expressionError;
  newMotor->inputVariables[ 0 ].name = SETPOINT_VARIABLE_NAME;
  newMotor->inputVariables[ 0 ].address = &(newMotor->setpoint);
  newMotor->inputVariables[ 1 ].name = REFERENCE_VARIABLE_NAME;
  newMotor->inputVariables[ 1 ].address = &(newMotor->offset);
  const char* transformExpression = DataIO_GetStringValue( configuration, SETPOINT_VARIABLE_NAME, KEY_OUTPUT );
  newMotor->transformFunction = te_compile( transformExpression, newMotor->inputVariables, 2, &expressionError ); 
  if( expressionError > 0 ) loadSuccess = false;
  DEBUG_PRINT( "transform function: out= %s (error: %d)", transformExpression, expressionError );
  if( DataIO_HasKey( configuration, KEY_LOG ) )
    newMotor->log = Log_Init( DataIO_GetBooleanValue( configuration, false, KEY_LOG "." KEY_FILE ) ? configName : "", 
                              (size_t) DataIO_GetNumericValue( configuration, 3, KEY_LOG "." KEY_PRECISION ) );
  
  DataIO_UnloadData( configuration );
  
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
  
  Output_End( motor->output );
  
  Input_End( motor->reference );
  
  if( motor->transformFunction != NULL ) te_free( motor->transformFunction );
  
  Log_End( motor->log );
  
  free( motor );
}

bool Motor_Enable( Motor motor )
{
  if( motor == NULL ) return false;

  Output_Reset( motor->output );

  bool enabled = Output_Enable( motor->output );
  
  enabled = ! Output_HasError( motor->output );
  
  return enabled;
}

void Motor_Disable( Motor motor )
{
  if( motor == NULL ) return;
  
  Output_Disable( motor->output );
}

void Motor_SetOffset( Motor motor )
{
  if( motor == NULL ) return;

  motor->offset = 0.0;

  motor->isOffsetting = true;
  DEBUG_PRINT( "setting motor %p reference state to offset", motor );
  Input_SetState( motor->reference, SIG_PROC_STATE_OFFSET );
}

void Motor_SetOperation( Motor motor )
{
  if( motor == NULL ) return;

  if( motor->isOffsetting ) motor->offset = Input_Update( motor->reference );
  
  motor->isOffsetting = false;
  DEBUG_PRINT( "setting motor %p reference state to operation", motor );
  Input_SetState( motor->reference, SIG_PROC_STATE_MEASUREMENT );
  DEBUG_PRINT( "setting motor %p to initial position", motor );
  Motor_WriteControl( motor, 0.0 );
}

void Motor_WriteControl( Motor motor, double setpoint )
{
  if( motor == NULL ) return;
  motor->setpoint = setpoint;
  DEBUG_PRINT( "evaluating transform function %p (set=%g, ref=%g)", motor->transformFunction, *((double*) motor->inputVariables[ 0 ].address), *((double*) motor->inputVariables[ 1 ].address) );
  double outputValue = te_eval( motor->transformFunction );
  DEBUG_PRINT( "logging motor data to %p", motor->log );
  //Log_EnterNewLine( motor->log, Time_GetExecSeconds() );
  //Log_RegisterValues( motor->log, 3, motor->setpoint, motor->offset, output );
  DEBUG_PRINT( "writing %g to output %p", outputValue, motor->output );
  if( ! motor->isOffsetting ) Output_Update( motor->output, outputValue );
}
