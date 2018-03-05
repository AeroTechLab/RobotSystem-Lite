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


#include "sensors.h"

#include "signal_processing/signal_processing.h" 

#include "signal_io_interface.h"
#include "curve_loader.h"

//#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include "data_io.h"

#include "timing/timing.h"

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


struct _SensorData
{
  DECLARE_MODULE_INTERFACE_REF( SIGNAL_IO_INTERFACE );
  int deviceID;
  unsigned int channel;
  double* inputBuffer;
  SignalProcessor processor;
  Sensor reference;
  double differentialGain;
  Log log;
};


Curve LoadMeasurementCurve( DataHandle curveData );

Sensor Sensor_Init( DataHandle configuration )
{
  static char filePath[ DATA_IO_MAX_PATH_LENGTH ];
  
  if( configuration == NULL ) return NULL;
  
  const char* sensorName = DataIO_GetStringValue( configuration, NULL, "" );
  if( sensorName != NULL )
  {
    sprintf( filePath, "sensors/%s", sensorName );
    if( (configuration = DataIO_LoadStorageData( filePath )) == NULL ) return NULL;
  }
  
  //DEBUG_PRINT( "sensor configuration found on data handle %p", configuration );
  
  Sensor newSensor = (Sensor) malloc( sizeof(SensorData) );
  memset( newSensor, 0, sizeof(SensorData) );  
  
  bool loadSuccess;
  sprintf( filePath, NAME_STRING( SIGNAL_IO_MODULES_PATH ) "/%s", DataIO_GetStringValue( configuration, "", "input_interface.type" ) );
  LOAD_MODULE_IMPLEMENTATION( SIGNAL_IO_INTERFACE, filePath, newSensor, &loadSuccess );
  if( loadSuccess )
  {
    newSensor->deviceID = newSensor->InitDevice( DataIO_GetStringValue( configuration, "", "input_interface.config" ) );
    if( newSensor->deviceID != SIGNAL_IO_DEVICE_INVALID_ID )
    {
      newSensor->channel = (unsigned int) DataIO_GetNumericValue( configuration, -1, "input_interface.channel" );
      loadSuccess = newSensor->CheckInputChannel( newSensor->deviceID, newSensor->channel );
      
      size_t maxInputSamplesNumber = newSensor->GetMaxInputSamplesNumber( newSensor->deviceID );
      newSensor->inputBuffer = (double*) calloc( maxInputSamplesNumber, sizeof(double) );
      
      uint8_t signalProcessingFlags = 0;
      if( DataIO_GetBooleanValue( configuration, false, "signal_processing.rectified" ) ) signalProcessingFlags |= SIG_PROC_RECTIFY;
      if( DataIO_GetBooleanValue( configuration, false, "signal_processing.normalized" ) ) signalProcessingFlags |= SIG_PROC_NORMALIZE;
      newSensor->processor = SignalProcessor_Create( signalProcessingFlags );
      
      double inputGain = DataIO_GetNumericValue( configuration, 1.0, "input_gain.multiplier" );
      inputGain /= DataIO_GetNumericValue( configuration, 1.0, "input_gain.divisor" );
      SignalProcessor_SetInputGain( newSensor->processor, inputGain );
      
      double relativeMinCutFrequency = DataIO_GetNumericValue( configuration, 0.0, "signal_processing.min_frequency" );
      SignalProcessor_SetMinFrequency( newSensor->processor, relativeMinCutFrequency );
      double relativeMaxCutFrequency = DataIO_GetNumericValue( configuration, 0.0, "signal_processing.max_frequency" );
      SignalProcessor_SetMaxFrequency( newSensor->processor, relativeMaxCutFrequency );
      
      newSensor->differentialGain = DataIO_GetNumericValue( configuration, 1.0, "differential_gain.multiplier" );
      newSensor->differentialGain /= DataIO_GetNumericValue( configuration, 1.0, "differential_gain.divisor" );
      
      const char* logFileName = DataIO_GetStringValue( configuration, NULL, "log" );
      if( logFileName != NULL )
      {
        sprintf( filePath, "sensors/%s", logFileName );
        newSensor->log = Log_Init( filePath, 4 );
      }
      
      DataHandle referenceConfiguration = DataIO_GetSubData( configuration, "reference" );
      newSensor->reference = Sensor_Init( referenceConfiguration );
      if( referenceConfiguration != NULL ) DataIO_UnloadData( referenceConfiguration );
      
      newSensor->Reset( newSensor->deviceID );
    }
    else loadSuccess = false;
  }
  
  if( sensorName != NULL ) DataIO_UnloadData( configuration );
  
  if( !loadSuccess )
  {
    Sensor_End( newSensor );
    return NULL;
  }    
  
  return newSensor;
}

void Sensor_End( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  sensor->EndDevice( sensor->deviceID );
  
  SignalProcessor_Discard( sensor->processor );
  
  free( sensor->inputBuffer );
  
  Log_End( sensor->log );
  
  Sensor_End( sensor->reference );
  
  free( sensor );
}

double Sensor_Update( Sensor sensor )
{
  if( sensor == NULL ) return 0.0;
  
  size_t aquiredSamplesNumber = sensor->Read( sensor->deviceID, sensor->channel, sensor->inputBuffer );
    
  double sensorOutput = SignalProcessor_UpdateSignal( sensor->processor, sensor->inputBuffer, aquiredSamplesNumber );
  
  double referenceOutput = Sensor_Update( sensor->reference );
  
  double sensorMeasure = sensor->differentialGain * ( sensorOutput - referenceOutput );
  
  //if( sensor->reference != NULL && sensor->measurementCurve != NULL ) DEBUG_PRINT( "sensor=%g, reference=%g, measure=%g", sensorOutput, referenceOutput, sensorMeasure );
  
  //Log_EnterNewLine( sensor->log, Time_GetExecSeconds() );
  //Log_RegisterList( sensor->log, sensor->maxInputSamplesNumber, sensor->inputBuffer );
  //Log_RegisterValues( sensor->log, 3, sensorOutput, referenceOutput, sensorMeasure );
  
  return sensorMeasure;
}
  
bool Sensor_HasError( Sensor sensor )
{
  if( sensor == NULL ) return false;
  
  return sensor->HasError( sensor->deviceID );
}

void Sensor_Reset( Sensor sensor )
{
  if( sensor == NULL ) return;
  
  SignalProcessor_SetState( sensor->processor, SIG_PROC_STATE_MEASUREMENT );
  sensor->Reset( sensor->deviceID );
}

void Sensor_SetState( Sensor sensor, enum SensorState newState )
{
  if( sensor == NULL ) return;
  
  enum SigProcState newProcessingState = SIG_PROC_STATE_MEASUREMENT;
  if( newState == SENSOR_STATE_OFFSET ) newProcessingState = SIG_PROC_STATE_OFFSET;
  else if( newState == SENSOR_STATE_CALIBRATION ) newProcessingState = SIG_PROC_STATE_CALIBRATION;
  
  SignalProcessor_SetState( sensor->processor, newProcessingState );
  Sensor_SetState( sensor->reference, newState );
}
