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


#include "actuators.h"

#include "motors.h"
#include "sensors.h"
#include "kalman/kalman_filters.h"

#include "debug/data_logging.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


enum ControlVariable { POSITION, VELOCITY, FORCE, ACCELERATION, CONTROL_VARS_NUMBER };

struct _ActuatorData
{
  enum ActuatorState controlState;
  enum ControlVariable controlMode;
  Motor motor;
  Sensor* sensorsList;
  size_t sensorsNumber;
  KFilter sensorFilter;
};


const char* CONTROL_MODE_NAMES[ CONTROL_VARS_NUMBER ] = { [ POSITION ] = "POSITION", [ VELOCITY ] = "VELOCITY", 
                                                          [ FORCE ] = "FORCE", [ ACCELERATION ] = "ACCELERATION" };
Actuator Actuator_Init( DataHandle configuration )
{
  static char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  if( configuration == NULL ) return NULL;
  
  const char* actuatorName = DataIO_GetStringValue( configuration, NULL, "" );
  if( actuatorName != NULL )
  {
    sprintf( filePath, "actuators/%s", actuatorName );
    if( (configuration = DataIO_LoadFileData( filePath )) == NULL ) return NULL;
  }
  
  Actuator newActuator = (Actuator) malloc( sizeof(ActuatorData) );
  memset( newActuator, 0, sizeof(ActuatorData) );
  
  bool loadSuccess = true;
  
  if( (newActuator->sensorsNumber = DataIO_GetListSize( configuration, "sensors" )) > 0 )
  {
    newActuator->sensorFilter = Kalman_CreateFilter( CONTROL_VARS_NUMBER );
    
    newActuator->sensorsList = (Sensor*) calloc( newActuator->sensorsNumber, sizeof(Sensor) );
    for( size_t sensorIndex = 0; sensorIndex < newActuator->sensorsNumber; sensorIndex++ )
    {
      DataHandle sensorConfiguration = DataIO_GetSubData( configuration, "sensors.%lu.config", sensorIndex );
      newActuator->sensorsList[ sensorIndex ] = Sensor_Init( sensorConfiguration ); 
      const char* sensorType = DataIO_GetStringValue( configuration, "", "sensors.%lu.input_variable", sensorIndex );
      for( int controlModeIndex = 0; controlModeIndex < CONTROL_VARS_NUMBER; controlModeIndex++ )
      {
        if( strcmp( sensorType, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) 
          Kalman_AddInput( newActuator->sensorFilter, controlModeIndex );
      }
    }
  }
  
  DataHandle motorConfiguration = DataIO_GetSubData( configuration, "motor.config" );
  if( (newActuator->motor = Motor_Init( motorConfiguration )) == NULL ) loadSuccess = false;
  
  const char* controlModeName = DataIO_GetStringValue( configuration, (char*) CONTROL_MODE_NAMES[ 0 ], "motor.output_variable" );
  for( newActuator->controlMode = 0; newActuator->controlMode < CONTROL_VARS_NUMBER; newActuator->controlMode++ )
  {
    if( strcmp( controlModeName, CONTROL_MODE_NAMES[ newActuator->controlMode ] ) == 0 ) break;
  }
  
  newActuator->controlState = ACTUATOR_OPERATION;
  
  if( actuatorName != NULL ) DataIO_UnloadData( configuration );
  
  if( !loadSuccess )
  {
    Actuator_End( newActuator );
    return NULL;
  }
  
  Actuator_Reset( newActuator );
  
  return newActuator;
}

void Actuator_End( Actuator actuator )
{
  if( actuator == NULL ) return;
  
  Kalman_DiscardFilter( actuator->sensorFilter );
  
  Motor_End( actuator->motor );
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
    Sensor_End( actuator->sensorsList[ sensorIndex ] );
}

void Actuator_Enable( Actuator actuator )
{
  if( actuator == NULL ) return;
  
  Motor_Enable( actuator->motor );     
}

void Actuator_Disable( Actuator actuator )
{
  if( actuator == NULL ) return;
    
  Motor_Disable( actuator->motor );
}

void Actuator_Reset( Actuator actuator )
{
  if( actuator == NULL ) return;
    
  Motor_Reset( actuator->motor );
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
    Sensor_Reset( actuator->sensorsList[ sensorIndex ] );
  
  Kalman_Reset( actuator->sensorFilter );
}

bool Actuator_SetControlState( Actuator actuator, enum ActuatorState newState )
{
  if( actuator == NULL ) return false;
  
  if( newState == actuator->controlState ) return false;
  
  if( newState >= ACTUATOR_STATES_NUMBER ) return false;

  enum SigProcState sensorsState = SIG_PROC_STATE_MEASUREMENT;
  if( newState == ACTUATOR_OFFSET ) sensorsState = SIG_PROC_STATE_OFFSET;
  else if( newState == ACTUATOR_CALIBRATION ) sensorsState = SIG_PROC_STATE_CALIBRATION;
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
    Sensor_SetState( actuator->sensorsList[ sensorIndex ], sensorsState );
  
  actuator->controlState = newState;
  
  return true;
}

bool Actuator_IsEnabled( Actuator actuator )
{
  if( actuator == NULL ) return false;
    
  return Motor_IsEnabled( actuator->motor );
}

bool Actuator_HasError( Actuator actuator )
{
  if( actuator == NULL ) return false;
    
  if( Motor_HasError( actuator->motor ) ) return true;
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    if( Sensor_HasError( actuator->sensorsList[ sensorIndex ] ) ) return true;
  }
  
  return false;
}


ActuatorVariables* Actuator_GetMeasures( Actuator actuator, ActuatorVariables* ref_measures, double timeDelta )
{
  if( actuator == NULL ) return NULL;
  
  //DEBUG_UPDATE( "reading measures from actuator %p", actuator );
  
  Kalman_SetPredictionFactor( actuator->sensorFilter, POSITION, VELOCITY, timeDelta );
  Kalman_SetPredictionFactor( actuator->sensorFilter, POSITION, ACCELERATION, timeDelta * timeDelta / 2.0 );
  Kalman_SetPredictionFactor( actuator->sensorFilter, VELOCITY, ACCELERATION, timeDelta );
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    double sensorMeasure = Sensor_Update( actuator->sensorsList[ sensorIndex ], NULL );
    Kalman_SetInput( actuator->sensorFilter, sensorIndex, sensorMeasure );
  }
  (void) Kalman_Predict( actuator->sensorFilter, (double*) ref_measures );
  (void) Kalman_Update( actuator->sensorFilter, NULL, (double*) ref_measures );
  
  //Log_PrintString( NULL, "p: %.3f, v: %.3f, f: %.3f", ref_measures->position, ref_measures->velocity, ref_measures->force );
  
  return ref_measures;
}

double Actuator_SetSetpoints( Actuator actuator, ActuatorVariables* ref_setpoints )
{
  if( actuator == NULL ) return 0.0;
  
  double motorSetpoint = ( (double*) ref_setpoints )[ actuator->controlMode ];
  
  // If the motor is being actually controlled, write its control output
  if( Motor_IsEnabled( actuator->motor ) && actuator->controlState != ACTUATOR_OFFSET ) 
    Motor_WriteControl( actuator->motor, motorSetpoint );
  
  return motorSetpoint;
}
