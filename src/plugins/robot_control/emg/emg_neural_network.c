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


#include "emg_processing.h" 

#include "data_io.h"

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#define EMG_ID_MAX_LENGTH 32

const double LEARNING_RATE = 0.1;
const double MOMENTUM_FACTOR = 0.5;
const double PRECISION = 10e-6;

struct _EMGModelData
{
  char** muscleNamesList;
  char** jointNamesList;
  size_t musclesNumber, jointsNumber;
  double* inputWeightsTable;
  double* outputWeightsTable;
  double* inputsList;
  double* inputsMinList;
  double* inputsMaxList;
  double* outputsList;
  double* outputsMinList;
  double* outputsMaxList;
  double* hiddenOutputsList;
  size_t inputsNumber, outputsNumber, hiddenNeuronsNumber;
};


bool LoadVariableNames( DataHandle modelData, EMGModel model )
{
  if( (model->musclesNumber = DataIO_GetListSize( modelData, "muscles" )) > 0 )
  {
    model->muscleNamesList = (char**) calloc( model->musclesNumber , sizeof(char*) );
    for( size_t muscleIndex = 0; muscleIndex < model->musclesNumber; muscleIndex++ )
    {
      model->muscleNamesList[ muscleIndex ] = (char*) calloc( EMG_ID_MAX_LENGTH, sizeof(char) );
      strncpy( model->muscleNamesList[ muscleIndex ], DataIO_GetStringValue( modelData, "", "muscles.%lu.id", muscleIndex ), EMG_ID_MAX_LENGTH );
    }
    
    if( (model->jointsNumber = DataIO_GetListSize( modelData, "joints" )) > 0 )
    {
      model->jointNamesList = (char**) calloc( model->jointsNumber , sizeof(char*) );
      for( size_t jointIndex = 0; jointIndex < model->jointsNumber; jointIndex++ )
      {
        model->jointNamesList[ jointIndex ] = (char*) calloc( EMG_ID_MAX_LENGTH, sizeof(char) );
        strncpy( model->jointNamesList[ jointIndex ], DataIO_GetStringValue( modelData, "", "joints.%lu", jointIndex ), EMG_ID_MAX_LENGTH );
      }
        
      return true;
    }
  }
  
  return false;
}

EMGModel EMGProcessing_InitModel( const char* configFileName )
{
  //DEBUG_PRINT( "Trying to load joint %s EMG data", configFileName );
  
  EMGModel newNetwork = NULL;
  
  char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  sprintf( filePath, "emg_models/%s", configFileName );
  DataHandle networkData = DataIO_LoadFileData( filePath );
  if( networkData != NULL )
  {
    newNetwork = (EMGModel) malloc( sizeof(EMGModelData) );
    memset( newNetwork, 0, sizeof(EMGModelData) );
    
    bool loadError = ! LoadVariableNames( networkData, newNetwork );

    newNetwork->inputsNumber = DataIO_GetListSize( networkData, "inputs_min" );
    newNetwork->outputsNumber = DataIO_GetListSize( networkData, "output_weights" );
    newNetwork->hiddenNeuronsNumber = DataIO_GetListSize( networkData, "input_weights" );
    if( newNetwork->outputsNumber > 0 && newNetwork->hiddenNeuronsNumber > 0 )
    {      
      //DEBUG_PRINT( "%u neurons found for joint %s network", newNetwork->neuronsNumber, configFileName );
      
      newNetwork->inputsList = (double*) calloc( newNetwork->inputsNumber + 1, sizeof(double) );
      newNetwork->inputWeightsTable = (double*) calloc( newNetwork->hiddenNeuronsNumber * ( newNetwork->inputsNumber + 1 ), sizeof(double) );
      for( size_t neuronIndex = 0; neuronIndex < newNetwork->hiddenNeuronsNumber; neuronIndex++ )
      {
        double* neuronInputWeightsList = newNetwork->inputWeightsTable + neuronIndex * ( newNetwork->inputsNumber + 1 );
        for( size_t inputIndex = 0; inputIndex < newNetwork->inputsNumber + 1; inputIndex++ )
          neuronInputWeightsList[ inputIndex ] = DataIO_GetNumericValue( networkData, 0.0, "input_weights.%lu.%lu", neuronIndex, inputIndex );
      }
      
      newNetwork->outputsList = (double*) calloc( newNetwork->outputsNumber, sizeof(double) );
      newNetwork->outputWeightsTable = (double*) calloc( newNetwork->outputsNumber * ( newNetwork->hiddenNeuronsNumber + 1 ), sizeof(double) );
      for( size_t outputIndex = 0; outputIndex < newNetwork->outputsNumber; outputIndex++ )
      {
        double* neuronOutputWeightsList = newNetwork->outputWeightsTable + outputIndex * ( newNetwork->hiddenNeuronsNumber + 1 );
        for( size_t neuronIndex = 0; neuronIndex < newNetwork->hiddenNeuronsNumber + 1; neuronIndex++ )
          neuronOutputWeightsList[ neuronIndex ] = DataIO_GetNumericValue( networkData, 0.0, "output_weights.%lu.%lu", neuronIndex, outputIndex );
      }
      
      newNetwork->hiddenOutputsList = (double*) calloc( newNetwork->hiddenNeuronsNumber + 1, sizeof(double) );
      
      newNetwork->inputsMinList = (double*) calloc( newNetwork->inputsNumber, sizeof(double) );
      newNetwork->inputsMaxList = (double*) calloc( newNetwork->inputsNumber, sizeof(double) );
      for( size_t inputIndex = 0; inputIndex < newNetwork->inputsNumber + 1; inputIndex++ )
      {
        newNetwork->inputsMinList[ inputIndex ] = DataIO_GetNumericValue( networkData, 0.0, "inputs_min.%lu", inputIndex );
        newNetwork->inputsMaxList[ inputIndex ] = DataIO_GetNumericValue( networkData, 0.0, "inputs_max.%lu", inputIndex );
      }
      
      newNetwork->outputsMinList = (double*) calloc( newNetwork->inputsNumber, sizeof(double) );
      newNetwork->outputsMaxList = (double*) calloc( newNetwork->inputsNumber, sizeof(double) );
      for( size_t outputIndex = 0; outputIndex < newNetwork->inputsNumber + 1; outputIndex++ )
      {
        newNetwork->outputsMinList[ outputIndex ] = DataIO_GetNumericValue( networkData, 0.0, "outputs_min.%lu", outputIndex );
        newNetwork->outputsMaxList[ outputIndex ] = DataIO_GetNumericValue( networkData, 0.0, "outputs_max.%lu", outputIndex );
      }
    }
    else loadError = true;
    
    DataIO_UnloadData( networkData );
    
    if( loadError )
    {
      EMGProcessing_EndModel( newNetwork );
      return NULL;
    }
  }
  //else
  //  DEBUG_PRINT( "configuration for joint %s not found", configFileName );
  
  return newNetwork;
}

void EMGProcessing_EndModel( EMGModel network )
{
  if( network == NULL ) return;
  
  for( size_t muscleIndex = 0; muscleIndex < network->musclesNumber; muscleIndex++ )
    free( network->muscleNamesList[ muscleIndex ] );
  free( network->muscleNamesList );
  for( size_t jointIndex = 0; jointIndex < network->jointsNumber; jointIndex++ )
    free( network->jointNamesList[ jointIndex ] );
  free( network->jointNamesList );
  free( network->inputWeightsTable );
  free( network->outputWeightsTable );
  free( network->inputsList );
  free( network->inputsMinList );
  free( network->inputsMaxList );
  free( network->outputsList );
  free( network->outputsMinList );
  free( network->outputsMaxList );
  free( network->hiddenOutputsList );
  
  free( network );
}

const char** EMGProcessing_GetMuscleNames( EMGModel model )
{
  if( model == NULL ) return NULL;
  
  return (const char**) model->muscleNamesList;
}

size_t EMGProcessing_GetMusclesCount( EMGModel model )
{
  if( model == NULL ) return 0;
  
  return model->musclesNumber;
}

void EMGProcessing_GetJointTorques( EMGModel model, double* jointTorquesList )
{
  if( model == NULL ) return;
  
  for( size_t jointIndex = 0; jointIndex < model->jointsNumber; jointIndex++ )
    jointTorquesList[ jointIndex ] = model->outputsList[ jointIndex ];
}

void EMGProcessing_GetJointStiffnesses( EMGModel model, double* jointStiffnessesList )
{
  if( model == NULL ) return;
  
  for( size_t jointIndex = model->jointsNumber; jointIndex < 2 * model->jointsNumber; jointIndex++ )
    jointStiffnessesList[ jointIndex ] = model->outputsList[ jointIndex ];
}

const char** EMGProcessing_GetJointNames( EMGModel model )
{
  if( model == NULL ) return NULL;
  
  return (const char**) model->jointNamesList;
}

size_t EMGProcessing_GetJointsCount( EMGModel model )
{
  if( model == NULL ) return 0;
  
  return model->jointsNumber;
}

void EMGProcessing_FitParameters( EMGModel network, EMGSamplingData* sampler )
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

void EMGProcessing_RunStep( EMGModel network, double* muscleActivationsList, double* jointAnglesList, double* externalTorquesList )
{
  if( network == NULL ) return;  
  
  //DEBUG_PRINT( "Muscle signals: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", normalizedSignalsList[ 0 ], normalizedSignalsList[ 1 ], normalizedSignalsList[ 2 ], normalizedSignalsList[ 3 ], normalizedSignalsList[ 4 ], normalizedSignalsList[ 5 ] );
  
  memcpy( network->inputsList, muscleActivationsList, network->musclesNumber * sizeof(double) );
  memcpy( network->inputsList + network->musclesNumber, jointAnglesList, network->jointsNumber * sizeof(double) );
  memcpy( network->inputsList + network->musclesNumber + network->jointsNumber, externalTorquesList, network->jointsNumber * sizeof(double) );
  
  for( size_t inputIndex = 0; inputIndex < network->inputsNumber; inputIndex++ )
  {
    network->inputsList[ inputIndex ] -= network->inputsMinList[ inputIndex ];
    network->inputsList[ inputIndex ] /= ( network->inputsMaxList[ inputIndex ] - network->inputsMinList[ inputIndex ] );
  }
  
  network->inputsList[ network->inputsNumber ] = -1.0;
  
  for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber; neuronIndex++ )
  {
    double neuronActivation = 0.0;
    double* neuronInputWeightsList = network->inputWeightsTable + neuronIndex * ( network->inputsNumber + 1 );
    for( size_t inputIndex = 0; inputIndex < network->inputsNumber + 1; inputIndex++ )
      neuronActivation += neuronInputWeightsList[ inputIndex ] * network->inputsList[ inputIndex ];
    network->hiddenOutputsList[ neuronIndex ] = 1.0 / ( 1.0 + exp( -neuronActivation ) );
  }
  network->hiddenOutputsList[ network->hiddenNeuronsNumber ] = -1.0;
  
  for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
  {
    double outputActivation = 0.0;
    double* neuronOutputWeightsList = network->outputWeightsTable + outputIndex * ( network->hiddenNeuronsNumber + 1 );
    for( size_t neuronIndex = 0; neuronIndex < network->hiddenNeuronsNumber + 1; neuronIndex++ )
      outputActivation += neuronOutputWeightsList[ neuronIndex ] * network->hiddenOutputsList[ neuronIndex ];
    network->outputsList[ outputIndex ] = 1.0 / ( 1.0 + exp( -outputActivation ) );
  }
  
  for( size_t outputIndex = 0; outputIndex < network->outputsNumber; outputIndex++ )
  {
    network->outputsList[ outputIndex ] -= network->outputsMinList[ outputIndex ];
    network->outputsList[ outputIndex ] /= ( network->outputsMaxList[ outputIndex ] - network->outputsMinList[ outputIndex ] );
  }
}
