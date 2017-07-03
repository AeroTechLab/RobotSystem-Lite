////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016 Leonardo José Consoni                                  //
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


#include <math.h>
#include <stdbool.h>

#include "sensors.h"
#include "signal_processing.h"
#include "curve_interpolation.h"

#include "utils/data_io.h"

#include "klib/khash.h"

#include "debug/async_debug.h"
#include "debug/data_logging.h"

#include "emg_processing.h"


typedef struct _NeuronData
{
  double* inputWeightsList;
  double* outputWeightsList;
  double activation;
}
NeuronData;

typedef NeuronData* Neuron;

typedef struct _EMGMuscleData
{
  Sensor emgSensor;
  double* emgRawBuffer;
  size_t emgRawBufferLength;
}
EMGMuscleData;

typedef EMGMuscleData* EMGMuscle;

typedef struct _EMGJointData
{
  EMGMuscleData* musclesList;
  size_t musclesNumber;
  NeuronData* neuronsList;
  size_t neuronsNumber;
  double inputAngleMax, inputAngleMin;
  double inputTorqueMax, inputTorqueMin;
  double outputTorque, outputStiffness;
  double outputTorqueMax, outputTorqueMin;
  double outputStiffnessMax, outputStiffnessMin;
  Log offsetLog, calibrationLog, samplingLog;
  Log currentLog;
  Log emgRawLog;
}
EMGJointData;

typedef EMGJointData* EMGJoint;

KHASH_MAP_INIT_INT( JointInt, EMGJoint )
static khash_t( JointInt )* jointsList = NULL;


DEFINE_NAMESPACE_INTERFACE( EMGProcessing, EMG_PROCESSING_FUNCTIONS );

static EMGJoint LoadEMGJointData( const char* );
static void UnloadEMGJointData( EMGJoint );


JointID EMGProcessing_InitJoint( const char* configFileName )
{
  if( jointsList == NULL ) jointsList = kh_init( JointInt );
  
  int configKey = (int) kh_str_hash_func( configFileName );
  
  int insertionStatus;
  khint_t newJointIndex = kh_put( JointInt, jointsList, configKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( jointsList, newJointIndex ) = LoadEMGJointData( configFileName );
    if( kh_value( jointsList, newJointIndex ) == NULL )
    {
      DEBUG_PRINT( "EMG joint controller %s configuration failed", configFileName );
      EMGProcessing_EndJoint( (int) newJointIndex );
      return EMG_JOINT_INVALID_ID;
    }
    
    DEBUG_PRINT( "new key %d inserted (iterator: %u - total: %u)", kh_key( jointsList, newJointIndex ), newJointIndex, kh_size( jointsList ) );
  }
  else if( insertionStatus == 0 ) DEBUG_PRINT( "joint key %d already exists", configKey );
  
  return (int) kh_key( jointsList, newJointIndex );
}

void EMGProcessing_EndJoint( JointID jointID )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  UnloadEMGJointData( kh_value( jointsList, jointIndex ) );
  
  kh_del( JointInt, jointsList, jointIndex );
  
  if( kh_size( jointsList ) == 0 )
  {
    kh_destroy( JointInt, jointsList );
    jointsList = NULL;
  }
}

void EMGProcessing_GetJointMuscleSignals( JointID jointID, double* normalizedSignalsList )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  //DEBUG_PRINT( "updating sensor %d-%lu (%p)", jointID, muscleIndex, joint->musclesList[ muscleIndex ]->emgSensor );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesNumber; muscleIndex++ )
    normalizedSignalsList[ muscleIndex ] = Sensors.Update( joint->musclesList[ muscleIndex ].emgSensor, joint->musclesList[ muscleIndex ].emgRawBuffer );
  
  if( joint->currentLog != NULL && joint->emgRawLog != NULL )
  {
    DataLogging.RegisterValues( joint->emgRawLog, 1, Timing.GetExecTimeSeconds() );
    for( size_t muscleIndex = 0; muscleIndex < joint->musclesNumber; muscleIndex++ )
      DataLogging.RegisterList( joint->emgRawLog, joint->musclesList[ muscleIndex ].emgRawBufferLength, joint->musclesList[ muscleIndex ].emgRawBuffer );
  }
}

void EMGProcessing_GetJointMuscleTorques( JointID jointID, double* normalizedSignalsList, double jointAngle, double jointExternalTorque, double* muscleTorquesList )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  if( joint->currentLog != NULL )
  {
    DataLogging.RegisterValues( joint->currentLog, 3, Timing.GetExecTimeSeconds(), jointAngle, jointExternalTorque );
    DataLogging.RegisterList( joint->currentLog, joint->musclesNumber, normalizedSignalsList );
  }
  
  //DEBUG_PRINT( "Muscle signals: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", normalizedSignalsList[ 0 ], normalizedSignalsList[ 1 ], normalizedSignalsList[ 2 ], normalizedSignalsList[ 3 ], normalizedSignalsList[ 4 ], normalizedSignalsList[ 5 ] );
  
  for( size_t inputIndex = 0; inputIndex < joint->musclesNumber; inputIndex++ )
    muscleTorquesList[ inputIndex ] = normalizedSignalsList[ inputIndex ];
      
  muscleTorquesList[ joint->musclesNumber ] = ( jointAngle - joint->inputAngleMin ) / ( joint->inputAngleMax - joint->inputAngleMin );
  muscleTorquesList[ joint->musclesNumber + 1 ] = ( jointExternalTorque - joint->inputTorqueMin ) / ( joint->inputTorqueMax - joint->inputTorqueMin );
}

double EMGProcessing_GetJointTorque( JointID jointID, double* muscleTorquesList )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  for( size_t neuronIndex = 0; neuronIndex < joint->neuronsNumber; neuronIndex++ )
  {
    Neuron neuron = (Neuron) &(joint->neuronsList[ neuronIndex ]);
      
    neuron->activation = 0.0;
    for( size_t inputIndex = 0; inputIndex < joint->musclesNumber + 2; inputIndex++ )
      neuron->activation += neuron->inputWeightsList[ inputIndex ] * muscleTorquesList[ inputIndex ];
    
    neuron->activation = 1.0 / ( 1.0 + exp( -neuron->activation ) );
  }
  
  joint->neuronsList[ joint->neuronsNumber ].activation = -1.0;
  
  double outputActivation = 0.0;
  for( size_t neuronIndex = 0; neuronIndex < joint->neuronsNumber + 1; neuronIndex++ )
  {
    Neuron neuron = (Neuron) &(joint->neuronsList[ neuronIndex ]);
    outputActivation += neuron->outputWeightsList[ 0 ] * neuron->activation;
  }
  outputActivation = 1.0 / ( 1.0 + exp( -outputActivation ) );
  
  return outputActivation * ( joint->outputTorqueMax - joint->outputTorqueMin ) + joint->outputTorqueMin;
}

double EMGProcessing_GetJointStiffness( JointID jointID, double* muscleTorquesList )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0.0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  for( size_t neuronIndex = 0; neuronIndex < joint->neuronsNumber; neuronIndex++ )
  {
    Neuron neuron = (Neuron) &(joint->neuronsList[ neuronIndex ]);
      
    neuron->activation = 0.0;
    for( size_t inputIndex = 0; inputIndex < joint->musclesNumber + 2; inputIndex++ )
      neuron->activation += neuron->inputWeightsList[ inputIndex ] * muscleTorquesList[ inputIndex ];
    
    neuron->activation = 1.0 / ( 1.0 + exp( -neuron->activation ) );
  }
  
  joint->neuronsList[ joint->neuronsNumber ].activation = -1.0;
  
  double outputActivation = 0.0;
  for( size_t neuronIndex = 0; neuronIndex < joint->neuronsNumber + 1; neuronIndex++ )
  {
    Neuron neuron = (Neuron) &(joint->neuronsList[ neuronIndex ]);
    outputActivation += neuron->outputWeightsList[ 1 ] * neuron->activation;
  }
  outputActivation = 1.0 / ( 1.0 + exp( -outputActivation ) );
  
  return outputActivation * ( joint->outputStiffnessMax - joint->outputStiffnessMin ) + joint->outputStiffnessMin;
}

void EMGProcessing_SetJointProcessingPhase( JointID jointID, enum EMGProcessingPhase processingPhase )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  enum SignalProcessingPhase signalProcessingPhase = SIGNAL_PROCESSING_PHASE_MEASUREMENT;
  
  if( processingPhase == EMG_PROCESSING_OFFSET )
  {
    signalProcessingPhase = SIGNAL_PROCESSING_PHASE_OFFSET;
    joint->currentLog = joint->offsetLog;
  }
  else if( processingPhase == EMG_PROCESSING_CALIBRATION )
  {
    signalProcessingPhase = SIGNAL_PROCESSING_PHASE_CALIBRATION;
    joint->currentLog = joint->calibrationLog;
  }
  else if( processingPhase == EMG_PROCESSING_SAMPLING )
    joint->currentLog = joint->samplingLog;
  else // if( processingPhase == EMG_PROCESSING_MEASUREMENT )
    joint->currentLog = NULL;
  
  DEBUG_PRINT( "new EMG processing phase: %d (log: %p)", processingPhase, joint->currentLog );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesNumber; muscleIndex++ )
    Sensors.SetState( joint->musclesList[ muscleIndex ].emgSensor, signalProcessingPhase );
}

size_t EMGProcessing_GetJointMusclesCount( JointID jointID )
{
  khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return 0;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  return joint->musclesNumber;
}

void EMGProcessing_FitJointParameters( JointID jointID, EMGJointSampler samplingData )
{
  /*khint_t jointIndex = kh_get( JointInt, jointsList, (khint_t) jointID );
  if( jointIndex == kh_end( jointsList ) ) return;
  
  EMGJoint joint = kh_value( jointsList, jointIndex );
  
  double squaredErrorSum = 0;
  size_t musclesNumber = joint->musclesListLength;
  
  double squaredErrorMean = 1000.0;
  while( squaredErrorMean > 0.1 )
  {
    for( size_t muscleIndex = 0; muscleIndex < musclesNumber; muscleIndex++ )
    {
      for( size_t muscleParameterIndex = 0; muscleParameterIndex < MUSCLE_GAINS_NUMBER; muscleParameterIndex++ )
      {
        double currentValue = parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ];
        currentValue = EMGProcessing.SetJointMuscleGain( sampler->jointID, muscleIndex, muscleParameterIndex, currentValue );
        parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ] = currentValue;
      }
    
      double* muscleSignalList = sampler->muscleSignalsList[ muscleIndex ];
    
      for( size_t sampleIndex = 0; sampleIndex < sampler->samplesCount; sampleIndex++ )
      {
        double normalizedSample = muscleSignalList[ sampleIndex ];
        double jointAngle = sampler->jointAnglesList[ sampleIndex ];
        double jointIDTorque = sampler->jointIDTorquesList[ sampleIndex ];
    
        double jointEMGTorque = EMGProcessing.GetJointMuscleTorque( sampler->jointID, muscleIndex, normalizedSample, jointAngle );
      
        double sampleError = jointEMGTorque - jointIDTorque;
      
        squaredErrorSum += ( sampleError * sampleError );
      }
    }
  
    squaredErrorMean = squaredErrorSum / sampler->samplesCount;
  }*/
}


static EMGJoint LoadEMGJointData( const char* configFileName )
{
  static char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  DEBUG_PRINT( "Trying to load joint %s EMG data", configFileName );
  
  EMGJoint newJoint = NULL;
  
  sprintf( filePath, "joints/%s", configFileName );
  DataHandle configuration = DataIO.GetDataHandler()->LoadFileData( filePath );
  if( configuration != NULL )
  {
    newJoint = (EMGJoint) malloc( sizeof(EMGJointData) );
    memset( newJoint, 0, sizeof(EMGJointData) );
    
    bool loadError = false;
    size_t emgRawSamplesNumber = 0;
    newJoint->musclesNumber = (size_t) DataIO.GetDataHandler()->GetListSize( configuration, "emg_sensors" );
    newJoint->neuronsNumber = (size_t) DataIO.GetDataHandler()->GetListSize( configuration, "output_weights" );
    if( newJoint->musclesNumber > 0 && newJoint->neuronsNumber > 0 )
    {
      DEBUG_PRINT( "%u neurons found for joint %s network", newJoint->neuronsNumber, configFileName );
      
      newJoint->neuronsList = (NeuronData*) calloc( newJoint->neuronsNumber + 1, sizeof(NeuronData) );
      for( size_t neuronIndex = 0; neuronIndex < newJoint->neuronsNumber; neuronIndex++ )
      {
        Neuron newNeuron = (Neuron) &(newJoint->neuronsList[ neuronIndex ]);
        
        newNeuron->inputWeightsList = (double*) calloc( newJoint->musclesNumber + 2, sizeof(double) );
        for( size_t inputIndex = 0; inputIndex < newJoint->musclesNumber + 2; inputIndex++ )
          newNeuron->inputWeightsList[ inputIndex ] = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "input_weights.%lu.%lu", neuronIndex, inputIndex );
        
        for( size_t outputIndex = 0; outputIndex < 2; outputIndex++ )
          newNeuron->outputWeightsList[ outputIndex ] = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "output_weights.%lu.%lu", neuronIndex, outputIndex );
      }
      
      newJoint->inputAngleMin = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "inputs_min.%lu", newJoint->musclesNumber );
      newJoint->inputAngleMax = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "inputs_max.%lu", newJoint->musclesNumber );
      newJoint->inputTorqueMin = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "inputs_min.%lu", newJoint->musclesNumber + 1 );
      newJoint->inputTorqueMax = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "inputs_max.%lu", newJoint->musclesNumber + 1 );
      newJoint->outputTorqueMin = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "outputs_min.0" );
      newJoint->outputTorqueMax = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "outputs_max.0" );
      newJoint->outputStiffnessMin = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "outputs_min.1" );
      newJoint->outputStiffnessMax = DataIO.GetDataHandler()->GetNumericValue( configuration, 0.0, "outputs_max.1" );
      
      for( size_t muscleIndex = 0; muscleIndex < newJoint->musclesNumber; muscleIndex++ )
      {
        DataHandle sensorData = DataIO.GetDataHandler()->GetSubData( configuration, "emg_sensors.%lu", muscleIndex );
        EMGMuscle newMuscle = &(newJoint->musclesList[ muscleIndex ]);
        newMuscle->emgSensor = Sensors.Init( sensorData );
        if( newJoint->musclesList[ muscleIndex ].emgSensor != NULL )
        {
          newMuscle->emgRawBufferLength = Sensors.GetInputBufferLength( newMuscle->emgSensor );
          newMuscle->emgRawBuffer = (double*) calloc( newMuscle->emgRawBufferLength, sizeof(double) );
          emgRawSamplesNumber += newMuscle->emgRawBufferLength;
        }
        else
          loadError = true;
      }
      
      char* logName = DataIO.GetDataHandler()->GetStringValue( configuration, NULL, "log" );
      if( logName != NULL )
      {
        size_t jointSampleValuesNumber = newJoint->musclesNumber + 3;
        
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_offset", logName );                                    
        newJoint->offsetLog = DataLogging.InitLog( filePath, jointSampleValuesNumber, DATA_LOG_MAX_PRECISION );
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_calibration", logName );
        newJoint->calibrationLog = DataLogging.InitLog( filePath, jointSampleValuesNumber, DATA_LOG_MAX_PRECISION );
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_sampling", logName );
        newJoint->samplingLog = DataLogging.InitLog( filePath, jointSampleValuesNumber, DATA_LOG_MAX_PRECISION );
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_raw", logName );
        newJoint->emgRawLog = DataLogging.InitLog( filePath, emgRawSamplesNumber + 1, 6 );
      }
    }
    else loadError = true;
    
    DataIO.GetDataHandler()->UnloadData( configuration );
    
    if( loadError )
    {
      UnloadEMGJointData( newJoint );
      return NULL;
    }
  }
  else
    DEBUG_PRINT( "configuration for joint %s not found", configFileName );
  
  return newJoint;
}

static void UnloadEMGJointData( EMGJoint joint )
{
  if( joint == NULL ) return;
  
  for( size_t neuronIndex = 0; neuronIndex < joint->neuronsNumber; neuronIndex++ )
    free( joint->neuronsList[ neuronIndex ].inputWeightsList );
  
  for( size_t muscleIndex = 0; muscleIndex < joint->musclesNumber; muscleIndex++ )
  {
    Sensors.End( joint->musclesList[ muscleIndex ].emgSensor );
    free( joint->musclesList[ muscleIndex ].emgRawBuffer );
  }
  
  DataLogging.EndLog( joint->offsetLog );
  DataLogging.EndLog( joint->calibrationLog );
  DataLogging.EndLog( joint->samplingLog );
  DataLogging.EndLog( joint->emgRawLog );
  
  free( joint->neuronsList );
  free( joint->musclesList );
  
  free( joint );
}
