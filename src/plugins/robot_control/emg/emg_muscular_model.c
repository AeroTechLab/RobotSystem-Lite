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

#include "curve_loader.h" 

#include "data_io.h"

#include "debug/data_logging.h"

#include <math.h>
#include <stdbool.h>

enum { MUSCLE_ACTIVE_FORCE, MUSCLE_PASSIVE_FORCE, MUSCLE_MOMENT_ARM, MUSCLE_NORM_LENGTH, MUSCLE_CURVES_NUMBER };

typedef struct _EMGMuscleData
{
  Curve curvesList[ MUSCLE_CURVES_NUMBER ];
  double activationFactor;
  double scalingFactor;
  double initialPenationAngle;
  double fibersForce;
}
EMGMuscleData;

typedef EMGMuscleData* EMGMuscle;

typedef struct _EMGJointData
{
  double torque;
  double stiffness;
}
EMGJointData;

typedef EMGJointData* EMGJoint;

typedef char EMGID[ 32 ];

typedef struct _EMGModelData
{
  EMGMuscle* musclesList;
  EMGID* muscleNamesList;
  size_t musclesNumber;
  EMGJoint* jointsList;
  EMGID* jointNamesList;
  size_t jointsNumber;
}
EMGModelData;

typedef EMGModelData* EMGModel;


const char* MUSCLE_CURVE_NAMES[ MUSCLE_CURVES_NUMBER ] = { "active_force", "passive_force", "moment_arm", "normalized_length" };
static EMGMuscle LoadEMGMuscleData( DataHandle modelData )
{
  if( modelData == NULL ) return NULL;

  EMGMuscle newMuscle = (EMGMuscle) malloc( sizeof(EMGMuscleData) );
    
  for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
  {
    DataHandle curveData = DataIO_GetSubData( modelData, "%s", MUSCLE_CURVE_NAMES[ curveIndex ] );
    newMuscle->curvesList[ curveIndex ] = Curve_Load( curveData );
  }

  newMuscle->activationFactor = DataIO_GetNumericValue( modelData, -2.0, "activation_factor" );
  newMuscle->scalingFactor = DataIO_GetNumericValue( modelData, 1.0, "scaling_factor" );
  newMuscle->initialPenationAngle = DataIO_GetNumericValue( modelData, 0.0, "initial_penation_angle" );
    
  return newMuscle;
}


EMGModel EMGProcessing_InitModel( const char* configFileName )
{
  static char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  
  //DEBUG_PRINT( "Trying to load joint %s EMG data", configFileName );
  
  EMGModel newModel = NULL;
  
  sprintf( filePath, "joints/%s", configFileName );
  DataHandle modelData = DataIO_LoadFileData( filePath );
  if( modelData != NULL )
  {
    newModel = (EMGModel) malloc( sizeof(EMGModelData) );
    memset( newModel, 0, sizeof(EMGModelData) );
    
    bool loadError = false;
    size_t emgRawSamplesNumber = 0;
    if( (newModel->musclesNumber = (size_t) DataIO_GetListSize( modelData, "muscles" )) > 0 )
    {
      //DEBUG_PRINT( "%u muscles found for joint %s", newModel->musclesNumber, configFileName );
      
      newModel->musclesList = (EMGMuscle*) calloc( newModel->musclesNumber , sizeof(EMGMuscle) );
      newModel->muscleNamesList = (EMGID*) calloc( newModel->musclesNumber , sizeof(EMGID) );
      for( size_t muscleIndex = 0; muscleIndex < newModel->musclesNumber; muscleIndex++ )
      {
        newModel->musclesList[ muscleIndex ] = LoadEMGMuscleData( DataIO_GetSubData( modelData, "muscles.%lu.model_functions", muscleIndex ) );
        strncpy( (char*) &(newModel->muscleNamesList[ muscleIndex ]), DataIO_GetStringValue( modelData, "", "muscles.%lu.id", muscleIndex ), sizeof(EMGID) ); 
      }
    }
    else loadError = true;
    
    if( (newModel->jointsNumber = (size_t) DataIO_GetListSize( modelData, "joints" )) > 0 )
    {
      newModel->jointsList = (EMGJoint*) calloc( newModel->jointsNumber , sizeof(EMGJoint) );
      newModel->jointNamesList = (EMGID*) calloc( newModel->jointsNumber , sizeof(EMGID) );
      for( size_t jointIndex = 0; jointIndex < newModel->jointsNumber; jointIndex++ )
      {
        newModel->jointsList[ jointIndex ] = (EMGJoint) malloc( sizeof(EMGJointData) );
        strncpy( (char*) &(newModel->jointNamesList[ jointIndex ]), DataIO_GetStringValue( modelData, "", "joints.%lu", jointIndex ), sizeof(EMGID) );
    }
    else loadError = true;
    
    DataIO_UnloadData( modelData );
    
    if( loadError )
    {
      EMGProcessing_EndModel( newModel );
      return NULL;
    }
  }
  
  return newModel;
}

void EMGProcessing_EndModel( EMGModel model )
{
  if( model == NULL ) return;
  
  for( size_t muscleIndex = 0; muscleIndex < model->musclesNumber; muscleIndex++ )
  {
    
    for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
      Curve_Discard( model->musclesList[ muscleIndex ]->curvesList[ curveIndex ] );
    free( model->musclesList[ muscleIndex ]->emgRawBuffer );
  }
  
  
  
  free( model->musclesList );
  
  free( model );
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

//void EMGProcessing_GetMuscleForces( EMGModel model, double* normalizedSignalsList, double modelAngle, double modelExternalTorque, double* emgSamplesList )
//{
//  
//}

void EMGProcessing_GetJointTorques( EMGModel model, double* jointTorquesList )
{
  if( model == NULL ) return;
  
  double modelTorque = 0.0;

  for( size_t muscleIndex = 0; muscleIndex < model->musclesNumber; muscleIndex++ )
    modelTorque += muscleTorquesList[ muscleIndex ];
}

void EMGProcessing_GetJointStiffnesses( EMGModel model, double* jointStiffnessesList )
{
  if( model == NULL ) return;
  
  double modelStiffness = 0.0;
  
  for( size_t muscleIndex = 0; muscleIndex < model->musclesNumber; muscleIndex++ )
    modelStiffness += fabs( muscleTorquesList[ muscleIndex ] );
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

void EMGProcessing_FitJointParameters( EMGModel model, EMGSamplingData* samplingData )
{
  /*khint_t modelIndex = kh_get( JointInt, modelsList, (khint_t) model );
  if( modelIndex == kh_end( modelsList ) ) return;
  
  EMGModel model = kh_value( modelsList, modelIndex );
  
  double squaredErrorSum = 0;
  size_t musclesNumber = model->musclesNumber;
  
  double squaredErrorMean = 1000.0;
  while( squaredErrorMean > 0.1 )
  {
    for( size_t muscleIndex = 0; muscleIndex < musclesNumber; muscleIndex++ )
    {
      for( size_t muscleParameterIndex = 0; muscleParameterIndex < MUSCLE_GAINS_NUMBER; muscleParameterIndex++ )
      {
        double currentValue = parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ];
        currentValue = EMGProcessing.SetJointMuscleGain( sampler->model, muscleIndex, muscleParameterIndex, currentValue );
        parametersList[ muscleIndex * MUSCLE_GAINS_NUMBER + muscleParameterIndex ] = currentValue;
      }
    
      double* muscleSignalList = sampler->muscleSignalsList[ muscleIndex ];
    
      for( size_t sampleIndex = 0; sampleIndex < sampler->samplesCount; sampleIndex++ )
      {
        double normalizedSample = muscleSignalList[ sampleIndex ];
        double modelAngle = sampler->modelAnglesList[ sampleIndex ];
        double modelTorque = sampler->modelTorquesList[ sampleIndex ];
    
        double modelEMGTorque = EMGProcessing.GetJointMuscleTorque( sampler->model, muscleIndex, normalizedSample, modelAngle );
      
        double sampleError = modelEMGTorque - modelTorque;
      
        squaredErrorSum += ( sampleError * sampleError );
      }
    }
  
    squaredErrorMean = squaredErrorSum / sampler->samplesCount;
  }*/
}


void EMGProcessing_RunStep( EMGModel model, double* muscleActivationsList, double* jointAnglesList, double* externalTorquesList )
{
  if( model == NULL ) return NULL;  
  
//  if( model->currentLog != NULL )
//  {
//    Log_RegisterValues( model->currentLog, 3, Time_GetExecSeconds(), modelAngle, modelExternalTorque );
//    Log_RegisterList( model->currentLog, model->musclesNumber, normalizedSignalsList );
//  }
  
//  if( model->currentLog != NULL && model->emgRawLog != NULL )
//  {
//    Log_RegisterValues( model->emgRawLog, 1, Time_GetExecSeconds() );
//    for( size_t muscleIndex = 0; muscleIndex < model->musclesNumber; muscleIndex++ )
//      Log_RegisterList( model->emgRawLog, model->musclesList[ muscleIndex ]->emgRawBufferLength, model->musclesList[ muscleIndex ]->emgRawBuffer );
//  }
  
  //DEBUG_PRINT( "Muscle signals: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", normalizedSignalsList[ 0 ], normalizedSignalsList[ 1 ], normalizedSignalsList[ 2 ], normalizedSignalsList[ 3 ], normalizedSignalsList[ 4 ], normalizedSignalsList[ 5 ] );
  
  for( size_t muscleIndex = 0; muscleIndex < model->musclesNumber; muscleIndex++ )
  {
    EMGMuscle muscle = model->musclesList[ muscleIndex ];
    
    double activation = ( exp( muscle->activationFactor * muscleActivationsList[ muscleIndex ] ) - 1 ) / ( exp( muscle->activationFactor ) - 1 );
  
    double activeForce = Curve_GetValue( muscle->curvesList[ MUSCLE_ACTIVE_FORCE ], jointAnglesList[ 0 ], 0.0 );
    double passiveForce = Curve_GetValue( muscle->curvesList[ MUSCLE_PASSIVE_FORCE ], jointAnglesList[ 0 ], 0.0 );
  
    double normalizedLength = Curve_GetValue( muscle->curvesList[ MUSCLE_NORM_LENGTH ], jointAnglesList[ 0 ], 0.0 );
    double momentArm = Curve_GetValue( muscle->curvesList[ MUSCLE_MOMENT_ARM ], jointAnglesList[ 0 ], 0.0 );
  
    if( normalizedLength == 0.0 ) normalizedLength = 1.0;
    double penationAngle = asin( sin( muscle->initialPenationAngle ) / normalizedLength );
  
    double normalizedForce = activeForce * activation + passiveForce;
    double resultingForce = muscle->scalingFactor * cos( penationAngle ) * normalizedForce;
  
    muscle->fibersForce = resultingForce * momentArm;
  }
}
