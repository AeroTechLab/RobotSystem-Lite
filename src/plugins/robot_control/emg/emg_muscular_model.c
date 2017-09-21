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

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

enum { MUSCLE_ACTIVE_FORCE, MUSCLE_PASSIVE_FORCE, MUSCLE_MOMENT_ARM, MUSCLE_NORM_LENGTH, MUSCLE_VELOCITY_FORCE, MUSCLE_CURVES_NUMBER };

typedef struct _EMGMuscleData
{
  Curve curvesList[ MUSCLE_CURVES_NUMBER ];
  double activationFactor;
  double scalingFactor;
  double initialPenationAngle;
  double fibersForce, fibersTorque;
}
EMGMuscleData;

typedef EMGMuscleData* EMGMuscle;

typedef struct _EMGJointData
{
  double torque, stiffness;
  double stiffnessScalingFactor, stiffnessOffset;
}
EMGJointData;

typedef EMGJointData* EMGJoint;

const size_t EMG_ID_MAX_LENGTH = 32;

struct _EMGModelData
{
  EMGMuscle* musclesList;
  char** muscleNamesList;
  size_t musclesNumber;
  EMGJoint* jointsList;
  char** jointNamesList;
  size_t jointsNumber;
};


const char* MUSCLE_CURVE_NAMES[ MUSCLE_CURVES_NUMBER ] = { "active_force", "passive_force", "moment_arm", "normalized_length", "force_vel" };
static EMGMuscle LoadEMGMuscleData( DataHandle muscleData )
{
  if( muscleData == NULL ) return NULL;

  EMGMuscle newMuscle = (EMGMuscle) malloc( sizeof(EMGMuscleData) );
    
  for( size_t curveIndex = 0; curveIndex < MUSCLE_CURVES_NUMBER; curveIndex++ )
  {
    DataHandle curveData = DataIO_GetSubData( muscleData, "%s", MUSCLE_CURVE_NAMES[ curveIndex ] );
    newMuscle->curvesList[ curveIndex ] = Curve_Load( curveData );
  }

  newMuscle->activationFactor = DataIO_GetNumericValue( muscleData, -2.0, "activation_factor" );
  newMuscle->scalingFactor = DataIO_GetNumericValue( muscleData, 1.0, "scaling_factor" );
  newMuscle->initialPenationAngle = DataIO_GetNumericValue( muscleData, 0.0, "initial_penation_angle" );
    
  return newMuscle;
}

static EMGJoint LoadEMGJointData( DataHandle jointData )
{
  if( jointData == NULL ) return NULL;

  EMGJoint newJoint = (EMGJoint) malloc( sizeof(EMGJointData) );
    
  newJoint->stiffnessScalingFactor = DataIO_GetNumericValue( jointData, 1.0, "stiffness_scale" );
  newJoint->stiffnessOffset = DataIO_GetNumericValue( jointData, 0.0, "stiffness_offset" );
    
  return newJoint;
}


EMGModel EMGProcessing_InitModel( const char* configFileName )
{
  Log_PrintString( NULL, "Trying to load EMG model %s data", configFileName );
  
  EMGModel newModel = NULL;
  
  char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
  sprintf( filePath, "./config/emg_models/%s", configFileName );
  DataHandle modelData = DataIO_LoadFileData( filePath );
  Log_PrintString( NULL, "model %s data handle: %p", filePath, modelData );
  if( modelData != NULL )
  {
    newModel = (EMGModel) malloc( sizeof(EMGModelData) );
    memset( newModel, 0, sizeof(EMGModelData) );
    
    bool loadError = false;
    if( (newModel->musclesNumber = DataIO_GetListSize( modelData, "muscles" )) > 0 )
    {
      newModel->musclesList = (EMGMuscle*) calloc( newModel->musclesNumber, sizeof(EMGMuscle) );
      newModel->muscleNamesList = (char**) calloc( newModel->musclesNumber, sizeof(char*) );
      for( size_t muscleIndex = 0; muscleIndex < newModel->musclesNumber; muscleIndex++ )
      {
        newModel->musclesList[ muscleIndex ] = LoadEMGMuscleData( DataIO_GetSubData( modelData, "muscles.%lu", muscleIndex ) );
        newModel->muscleNamesList[ muscleIndex ] = (char*) calloc( EMG_ID_MAX_LENGTH, sizeof(char) );
        strncpy( newModel->muscleNamesList[ muscleIndex ], DataIO_GetStringValue( modelData, "", "muscles.%lu.id", muscleIndex ), EMG_ID_MAX_LENGTH );
      }
    }
    else loadError = true;
    
    if( (newModel->jointsNumber = DataIO_GetListSize( modelData, "joints" )) > 0 )
    {
      Log_PrintString( NULL, "%lu joints found", newModel->jointsNumber ); 
      
      newModel->jointsList = (EMGJoint*) calloc( newModel->jointsNumber, sizeof(EMGJoint) );
      newModel->jointNamesList = (char**) calloc( newModel->jointsNumber, sizeof(char*) );
      for( size_t jointIndex = 0; jointIndex < newModel->jointsNumber; jointIndex++ )
      {
        newModel->jointsList[ jointIndex ] = LoadEMGJointData( DataIO_GetSubData( modelData, "joints.%lu", jointIndex ) );
        newModel->jointNamesList[ jointIndex ] = (char*) calloc( EMG_ID_MAX_LENGTH, sizeof(char) );
        strncpy( newModel->jointNamesList[ jointIndex ], DataIO_GetStringValue( modelData, "", "joints.%lu.id", jointIndex ), EMG_ID_MAX_LENGTH );
      }
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
    free( model->musclesList[ muscleIndex ] );
    free( model->muscleNamesList[ muscleIndex ] );
  }
  
  free( model->musclesList );
  free( model->muscleNamesList );
  
  for( size_t jointIndex = 0; jointIndex < model->jointsNumber; jointIndex++ )
  {
    free( model->jointsList[ jointIndex ] );
    free( model->jointNamesList[ jointIndex ] );
  }
    
  free( model->jointsList );
  free( model->jointNamesList );
  
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
  
  for( size_t muscleIndex = 0; muscleIndex < model->musclesNumber; muscleIndex++ )
    jointTorquesList[ 0 ] += model->musclesList[ muscleIndex ]->fibersTorque;
}

void EMGProcessing_GetJointStiffnesses( EMGModel model, double* jointStiffnessesList )
{
  if( model == NULL ) return;
  
  double jointStiffness = 0.0;
  for( size_t muscleIndex = 0; muscleIndex < model->musclesNumber; muscleIndex++ )
    jointStiffness += fabs( model->musclesList[ muscleIndex ]->fibersTorque );
  
  jointStiffnessesList[ 0 ] = jointStiffness * model->jointsList[ 0 ]->stiffnessScalingFactor + model->jointsList[ 0 ]->stiffnessOffset;
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

void EMGProcessing_FitParameters( EMGModel model, EMGSamplingData* samplingData )
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
  if( model == NULL ) return;  
  
  //DEBUG_PRINT( "Muscle signals: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f", normalizedSignalsList[ 0 ], normalizedSignalsList[ 1 ], normalizedSignalsList[ 2 ], normalizedSignalsList[ 3 ], normalizedSignalsList[ 4 ], normalizedSignalsList[ 5 ] );
  
  for( size_t muscleIndex = 0; muscleIndex < model->musclesNumber; muscleIndex++ )
  {
    EMGMuscle muscle = model->musclesList[ muscleIndex ];
    
    double activation = ( exp( muscle->activationFactor * muscleActivationsList[ muscleIndex ] ) - 1 ) / ( exp( muscle->activationFactor ) - 1 );
  
    double activeForce = Curve_GetValue( muscle->curvesList[ MUSCLE_ACTIVE_FORCE ], jointAnglesList[ 0 ], 0.0 );
    double passiveForce = Curve_GetValue( muscle->curvesList[ MUSCLE_PASSIVE_FORCE ], jointAnglesList[ 0 ], 0.0 );
    double velocityForce = Curve_GetValue( muscle->curvesList[ MUSCLE_VELOCITY_FORCE ], jointAnglesList[ 0 ], 0.0 );
  
    double normalizedLength = Curve_GetValue( muscle->curvesList[ MUSCLE_NORM_LENGTH ], jointAnglesList[ 0 ], 0.0 );
    double momentArm = Curve_GetValue( muscle->curvesList[ MUSCLE_MOMENT_ARM ], jointAnglesList[ 0 ], 0.0 );
  
    if( normalizedLength == 0.0 ) normalizedLength = 1.0;
    double penationAngle = asin( sin( muscle->initialPenationAngle ) / normalizedLength );
  
    double normalizedForce = activeForce * velocityForce * activation + passiveForce;
    muscle->fibersForce = muscle->scalingFactor * cos( penationAngle ) * normalizedForce;
  
    muscle->fibersTorque = muscle->fibersForce * momentArm;
  }
}
