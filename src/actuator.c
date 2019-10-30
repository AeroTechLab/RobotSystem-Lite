////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2019 Leonardo Consoni <leonardojc@protonmail.com>      //
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

#include "data_io/interface/data_io.h"
#include "kalman/kalman_filters.h"
#include "debug/data_logging.h"
#include "timing/timing.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


enum ControlVariable { POSITION, VELOCITY, ACCELERATION, FORCE, CONTROL_VARS_NUMBER };

struct _ActuatorData
{
  enum ControlState controlState;
  enum ControlVariable controlMode;
  Motor motor;
  Sensor* sensorsList;
  size_t sensorsNumber;
  KFilter motionFilter;
  Log log;
};


const char* CONTROL_MODE_NAMES[ CONTROL_VARS_NUMBER ] = { [ POSITION ] = "POSITION", [ VELOCITY ] = "VELOCITY", 
                                                          [ ACCELERATION ] = "ACCELERATION", [ FORCE ] = "FORCE" };
Actuator Actuator_Init( const char* configName )
{
  char filePath[ DATA_IO_MAX_PATH_LENGTH ];  
  DEBUG_PRINT( "trying to create actuator %s", configName );
  sprintf( filePath, KEY_CONFIG "/" KEY_ACTUATORS "/%s", configName );
  DataHandle configuration = DataIO_LoadStorageData( filePath );
  if( configuration == NULL ) return NULL;
  DEBUG_PRINT( "found actuator %s config in handle %p", configName, configuration );
  Actuator newActuator = (Actuator) malloc( sizeof(ActuatorData) );
  memset( newActuator, 0, sizeof(ActuatorData) );
  
  bool loadSuccess = true;
  DEBUG_PRINT( "found %lu sensors", DataIO_GetListSize( configuration, KEY_SENSORS ) );
  if( (newActuator->sensorsNumber = DataIO_GetListSize( configuration, KEY_SENSORS )) > 0 )
  {
    newActuator->motionFilter = Kalman_CreateFilter( CONTROL_VARS_NUMBER, newActuator->sensorsNumber, 0 );
    
    newActuator->sensorsList = (Sensor*) calloc( newActuator->sensorsNumber, sizeof(Sensor) );
    for( size_t sensorIndex = 0; sensorIndex < newActuator->sensorsNumber; sensorIndex++ )
    {
      const char* sensorName = DataIO_GetStringValue( configuration, "", KEY_SENSORS ".%lu." KEY_CONFIG, sensorIndex );
      if( (newActuator->sensorsList[ sensorIndex ] = Sensor_Init( sensorName )) == NULL ) loadSuccess = false;
      DEBUG_PRINT( "loading sensor %s success: %s", sensorName, loadSuccess ? "true" : "false" );
      const char* sensorType = DataIO_GetStringValue( configuration, "", KEY_SENSORS ".%lu." KEY_VARIABLE, sensorIndex );
      double measurementDeviation = DataIO_GetNumericValue( configuration, 1.0, KEY_SENSORS ".%lu." KEY_DEVIATION, sensorIndex );
      for( int controlModeIndex = 0; controlModeIndex < CONTROL_VARS_NUMBER; controlModeIndex++ )
        if( strcmp( sensorType, CONTROL_MODE_NAMES[ controlModeIndex ] ) == 0 ) 
          Kalman_SetMeasureWeight( newActuator->motionFilter, sensorIndex, controlModeIndex, measurementDeviation );
    }
  }
  
  const char* motorName = DataIO_GetStringValue( configuration, "", KEY_MOTOR "." KEY_CONFIG );
  if( (newActuator->motor = Motor_Init( motorName )) == NULL ) loadSuccess = false;
  DEBUG_PRINT( "loading motor %s success: %s", motorName, loadSuccess ? "true" : "false" ); 
  const char* controlModeName = DataIO_GetStringValue( configuration, (char*) CONTROL_MODE_NAMES[ 0 ], KEY_MOTOR "." KEY_VARIABLE );
  for( newActuator->controlMode = 0; newActuator->controlMode < CONTROL_VARS_NUMBER; newActuator->controlMode++ )
    if( strcmp( controlModeName, CONTROL_MODE_NAMES[ newActuator->controlMode ] ) == 0 ) break;
  DEBUG_PRINT( "control mode: %s", CONTROL_MODE_NAMES[ newActuator->controlMode ] );
  if( DataIO_HasKey( configuration, KEY_LOG ) )
    newActuator->log = Log_Init( DataIO_GetBooleanValue( configuration, false, KEY_LOG "." KEY_FILE ) ? configName : "", 
                                 (size_t) DataIO_GetNumericValue( configuration, 3, KEY_LOG "." KEY_PRECISION ) );
  //DEBUG_PRINT( "log created with handle %p", newActuator->log );
  newActuator->controlState = CONTROL_PASSIVE;
  //DEBUG_PRINT( "loading success: %s", loadSuccess ? "true" : "false" );
  DataIO_UnloadData( configuration );
  //DEBUG_PRINT( "data on handle %p unloaded", configuration );
  if( !loadSuccess )
  {
    Actuator_End( newActuator );
    return NULL;
  }
  //DEBUG_PRINT( "reseting actuator %s", configName );
  Kalman_Reset( newActuator->motionFilter );
  //DEBUG_PRINT( "actuator %s ready", configName );
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

bool Actuator_SetControlState( Actuator actuator, enum ControlState newState )
{
  if( actuator == NULL ) return false;
  
  if( newState == actuator->controlState ) return false;
  
  if( newState >= CONTROL_STATES_NUMBER ) return false;

  DEBUG_PRINT( "setting actuator state to %s", ( newState == CONTROL_OFFSET ) ? "offset" : ( ( newState == CONTROL_CALIBRATION ) ? "calibration" : "operation" ) );
  if( newState == CONTROL_OFFSET )
  {
    for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
      Sensor_SetOffset( actuator->sensorsList[ sensorIndex ] );
    Motor_SetOffset( actuator->motor );
  }
  else if( newState == CONTROL_CALIBRATION )
  {
    for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
      Sensor_SetCalibration( actuator->sensorsList[ sensorIndex ] );
    Motor_SetOperation( actuator->motor );
  }
  else
  {
    for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
      Sensor_SetMeasurement( actuator->sensorsList[ sensorIndex ] );
    Motor_SetOperation( actuator->motor );
  }
  
  actuator->controlState = newState;
  
  return true;
}

bool Actuator_GetMeasures( Actuator actuator, DoFVariables* ref_measures, double timeDelta )
{
  if( actuator == NULL ) return false;
  
  //DEBUG_PRINT( "reading measures from %lu sensors", actuator->sensorsNumber );
  double filteredMeasures[ CONTROL_VARS_NUMBER ];
  
  Kalman_SetTransitionFactor( actuator->motionFilter, POSITION, VELOCITY, timeDelta );
  Kalman_SetTransitionFactor( actuator->motionFilter, POSITION, ACCELERATION, timeDelta * timeDelta / 2.0 );
  Kalman_SetTransitionFactor( actuator->motionFilter, VELOCITY, ACCELERATION, timeDelta );
  for( size_t sensorIndex = 0; sensorIndex < actuator->sensorsNumber; sensorIndex++ )
  {
    double sensorMeasure = Sensor_Update( actuator->sensorsList[ sensorIndex ] );
    Kalman_SetMeasure( actuator->motionFilter, sensorIndex, sensorMeasure );
  }
  (void) Kalman_Predict( actuator->motionFilter, NULL, (double*) filteredMeasures );
  (void) Kalman_Update( actuator->motionFilter, NULL, (double*) filteredMeasures );
  
  DEBUG_PRINT( "p=%.5f, v=%.5f, f=%.5f", filteredMeasures[ POSITION ], filteredMeasures[ VELOCITY ], filteredMeasures[ FORCE ] );
  ref_measures->position = filteredMeasures[ POSITION ];
  ref_measures->velocity = filteredMeasures[ VELOCITY ];
  ref_measures->acceleration = filteredMeasures[ ACCELERATION ];
  ref_measures->force = filteredMeasures[ FORCE ];
  
  Log_EnterNewLine( actuator->log, Time_GetExecSeconds() );
  Log_RegisterList( actuator->log, CONTROL_VARS_NUMBER, (double*) filteredMeasures );
  
  return true;
}

double Actuator_SetSetpoints( Actuator actuator, DoFVariables* ref_setpoints )
{
  if( actuator == NULL ) return 0.0;
  
  double motorSetpoint = ( (double*) ref_setpoints )[ actuator->controlMode ];
  //DEBUG_PRINT( "writing setpoint %g to motor", motorSetpoint );
  // If the motor is being actually controlled, write its control output
  if( actuator->controlState == CONTROL_OPERATION ) Motor_WriteControl( actuator->motor, motorSetpoint );
  //DEBUG_PRINT( "setpoint %g written to motor", motorSetpoint );
  return motorSetpoint;
}
