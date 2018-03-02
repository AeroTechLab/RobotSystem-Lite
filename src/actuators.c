////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2018 Leonardo Consoni <consoni_2519@hotmail.com>       //
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
  ActuatorVariables offset;
  Motor motor;
  Sensor* sensorsList;
  size_t sensorsNumber;
  KFilter motionFilter;
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
    newActuator->motionFilter = Kalman_CreateFilter( CONTROL_VARS_NUMBER );
    
    newActuator->sensorsList = (Sensor*) calloc( newActuator->sensorsNumber, sizeof(Sensor) );
    for( size_t sensorIndex = 0; sensorIndex < newActuator->sensorsNumber; sensorIndex++ )
    {
      DataHandle sensorConfiguration = DataIO_GetSubData( configuration, "sensors.%lu.config", sensorIndex );
      newActuator->sensorsList[ sensorIndex ] = Sensor_Init( sensorConfiguration );
      Sensor_Reset( newActuator->sensorsList[ sensorIndex ] );
      const char* sensorType = DataIO_GetStringValue( configuration, "", "sensors.%lu.input_variable", sensorIndex );
      for( int controlModeIndex = 0; controlModeIndex < CONTROL_VARS_NUMBER; controlModeIndex++ )
      {
        if( strcmp( sensorType, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) 
          Kalman_AddInput( newActuator->motionFilter, controlModeIndex );
      }
      Kalman_SetInputMaxError( newActuator->motionFilter, sensorIndex, DataIO_GetNumericValue( configuration, 1.0, "sensors.%lu.deviation" ) );
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
  
  Kalman_DiscardFilter( actuator->motionFilter );
  
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


bool Actuator_GetMeasures( Actuator actuator, ActuatorVariables* ref_measures, double timeDelta )
{
  if( actuator == NULL ) return false;
  
  //DEBUG_UPDATE( "reading measures from actuator %p", actuator );
  
  Kalman_SetPredictionFactor( actuator->motionFilter, POSITION, VELOCITY, timeDelta );
  Kalman_SetPredictionFactor( actuator->motionFilter, POSITION, ACCELERATION, timeDelta * timeDelta / 2.0 );
  Kalman_SetPredictionFactor( actuator->motionFilter, VELOCITY, ACCELERATION, timeDelta );
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    double sensorMeasure = Sensor_Update( actuator->sensorsList[ sensorIndex ], NULL );
    Kalman_SetInput( actuator->motionFilter, sensorIndex, sensorMeasure );
  }
  (void) Kalman_Predict( actuator->motionFilter, (double*) ref_measures );
  (void) Kalman_Update( actuator->motionFilter, NULL, (double*) ref_measures );
  
  //DEBUG_PRINT( "p=%.5f, v=%.5f, f=%.5f", ref_measures->position, ref_measures->velocity, ref_measures->force );
  
  if( actuator->controlState == ACTUATOR_OFFSET ) 
  {
    actuator->offset = *ref_measures;
    return false;
  }
  
  ref_measures->position -= actuator->offset.position;
  ref_measures->velocity -= actuator->offset.velocity;
  ref_measures->acceleration -= actuator->offset.acceleration;
  ref_measures->force -= actuator->offset.force;
    
  return true;
}

double Actuator_SetSetpoints( Actuator actuator, ActuatorVariables* ref_setpoints )
{
  if( actuator == NULL ) return 0.0;
  
  double motorSetpoint = ( (double*) ref_setpoints )[ actuator->controlMode ];
  motorSetpoint += ( (double*) &(actuator->offset) )[ actuator->controlMode ];
  
  // If the motor is being actually controlled, write its control output
  if( Motor_IsEnabled( actuator->motor ) && actuator->controlState != ACTUATOR_OFFSET ) 
    Motor_WriteControl( actuator->motor, motorSetpoint );
  
  return motorSetpoint;
}
