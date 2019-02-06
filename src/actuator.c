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


#include "actuator.h"

#include "config_keys.h"

#include "motor.h"
#include "sensor.h"

#include "kalman/kalman_filters.h"
#include "debug/data_logging.h"
#include "timing/timing.h"

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
  KFilter motionFilter;
  Log log;
};


const char* CONTROL_MODE_NAMES[ CONTROL_VARS_NUMBER ] = { [ POSITION ] = "POSITION", [ VELOCITY ] = "VELOCITY", 
                                                          [ FORCE ] = "FORCE", [ ACCELERATION ] = "ACCELERATION" };
Actuator Actuator_Init( DataHandle configuration )
{
  static char filePath[ DATA_IO_MAX_PATH_LENGTH ];
  
  if( configuration == NULL ) return NULL;
  
  const char* actuatorName = DataIO_GetStringValue( configuration, NULL, "" );
  if( actuatorName != NULL )
  {
    DEBUG_PRINT( "trying to create actuator %s", actuatorName );
    sprintf( filePath, KEY_CONFIG "/" KEY_ACTUATOR "/%s", actuatorName );
    if( (configuration = DataIO_LoadStorageData( filePath )) == NULL ) return NULL;
  }
  
  Actuator newActuator = (Actuator) malloc( sizeof(ActuatorData) );
  memset( newActuator, 0, sizeof(ActuatorData) );
  
  bool loadSuccess = true;
  
  if( (newActuator->sensorsNumber = DataIO_GetListSize( configuration, KEY_SENSOR "s" )) > 0 )
  {
    newActuator->motionFilter = Kalman_CreateFilter( CONTROL_VARS_NUMBER );
    
    newActuator->sensorsList = (Sensor*) calloc( newActuator->sensorsNumber, sizeof(Sensor) );
    for( size_t sensorIndex = 0; sensorIndex < newActuator->sensorsNumber; sensorIndex++ )
    {
      DataHandle sensorConfiguration = DataIO_GetSubData( configuration, KEY_SENSOR "s.%lu." KEY_CONFIG, sensorIndex );
      newActuator->sensorsList[ sensorIndex ] = Sensor_Init( sensorConfiguration );
      Sensor_Reset( newActuator->sensorsList[ sensorIndex ] );
      const char* sensorType = DataIO_GetStringValue( configuration, "", KEY_SENSOR "s.%lu." KEY_VARIABLE, sensorIndex );
      for( int controlModeIndex = 0; controlModeIndex < CONTROL_VARS_NUMBER; controlModeIndex++ )
      {
        if( strcmp( sensorType, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) 
          Kalman_AddInput( newActuator->motionFilter, controlModeIndex );
      }
      Kalman_SetInputMaxError( newActuator->motionFilter, sensorIndex, DataIO_GetNumericValue( configuration, 1.0, KEY_SENSOR "s.%lu." KEY_DEVIATION ) );
    }
  }
  
  DataHandle motorConfiguration = DataIO_GetSubData( configuration, KEY_MOTOR "." KEY_CONFIG );
  if( (newActuator->motor = Motor_Init( motorConfiguration )) == NULL ) loadSuccess = false;
  
  const char* controlModeName = DataIO_GetStringValue( configuration, (char*) CONTROL_MODE_NAMES[ 0 ], KEY_MOTOR "." KEY_VARIABLE );
  for( newActuator->controlMode = 0; newActuator->controlMode < CONTROL_VARS_NUMBER; newActuator->controlMode++ )
  {
    if( strcmp( controlModeName, CONTROL_MODE_NAMES[ newActuator->controlMode ] ) == 0 ) break;
  }
  
  if( DataIO_HasKey( configuration, KEY_LOG ) )
  {
    const char* logFileName = DataIO_GetStringValue( configuration, "", KEY_LOG "." KEY_FILE );
    newActuator->log = Log_Init( logFileName, (size_t) DataIO_GetNumericValue( configuration, 3, KEY_LOG "." KEY_PRECISION ) );
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
  
  Kalman_DiscardFilter( actuator->motionFilter );
  
  Motor_End( actuator->motor );
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
    Sensor_End( actuator->sensorsList[ sensorIndex ] );
  
  Log_End( actuator->log );
}

bool Actuator_Enable( Actuator actuator )
{
  if( actuator == NULL ) return false;
  
  return Motor_Enable( actuator->motor );     
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
  
  Kalman_Reset( actuator->motionFilter );
}

bool Actuator_SetControlState( Actuator actuator, enum ActuatorState newState )
{
  if( actuator == NULL ) return false;
  
  if( newState == actuator->controlState ) return false;
  
  if( newState >= ACTUATOR_STATES_NUMBER ) return false;

  enum SensorState sensorsState = SENSOR_STATE_MEASUREMENT;
  if( newState == ACTUATOR_OFFSET ) sensorsState = SENSOR_STATE_OFFSET;
  else if( newState == ACTUATOR_CALIBRATION ) sensorsState = SENSOR_STATE_CALIBRATION;
  
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
    Sensor_SetState( actuator->sensorsList[ sensorIndex ], sensorsState );
  
  Motor_SetOffset( actuator->motor, ( newState == ACTUATOR_OFFSET ) );
  
  actuator->controlState = newState;
  
  return true;
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


bool Actuator_GetMeasures( Actuator actuator, ActuatorVariables* ref_measures, double timeDelta )
{
  if( actuator == NULL ) return false;
  
  //DEBUG_UPDATE( "reading measures from actuator %p", actuator );
  
  Kalman_SetPredictionFactor( actuator->motionFilter, POSITION, VELOCITY, timeDelta );
  Kalman_SetPredictionFactor( actuator->motionFilter, POSITION, ACCELERATION, timeDelta * timeDelta / 2.0 );
  Kalman_SetPredictionFactor( actuator->motionFilter, VELOCITY, ACCELERATION, timeDelta );
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    double sensorMeasure = Sensor_Update( actuator->sensorsList[ sensorIndex ] );
    Kalman_SetInput( actuator->motionFilter, sensorIndex, sensorMeasure );
  }
  (void) Kalman_Predict( actuator->motionFilter, (double*) ref_measures );
  (void) Kalman_Update( actuator->motionFilter, NULL, (double*) ref_measures );
  
  //DEBUG_PRINT( "p=%.5f, v=%.5f, f=%.5f", ref_measures->position, ref_measures->velocity, ref_measures->force );
  
  Log_EnterNewLine( actuator->log, Time_GetExecSeconds() );
  Log_RegisterList( actuator->log, 4, (double*) ref_measures );
  
  return true;
}

double Actuator_SetSetpoints( Actuator actuator, ActuatorVariables* ref_setpoints )
{
  if( actuator == NULL ) return 0.0;
  
  double motorSetpoint = ( (double*) ref_setpoints )[ actuator->controlMode ];
  
  // If the motor is being actually controlled, write its control output
  if( actuator->controlState != ACTUATOR_OFFSET ) 
    Motor_WriteControl( actuator->motor, motorSetpoint );
  
  return motorSetpoint;
}
