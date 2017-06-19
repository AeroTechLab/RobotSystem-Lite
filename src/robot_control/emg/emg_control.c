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


#include "utils/data_io.h"

#include "utils/debug/async_debug.h"

#include <stdlib.h>
#include <math.h> 

#include "emg_processing.h"

#include "robot_control/interface.h"

const unsigned long UPDATE_INTERVAL_MS = 5;


typedef struct _ControlData 
{
  enum RobotState currentControlState;
  JointID* jointIDsList;
  EMGJointSampler* jointSamplersList;
  char** jointNamesList;
  size_t jointsNumber;
}
ControlData;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


RobotController InitController( const char* configurationString )
{
  DEBUG_PRINT( "Trying to load robot control config %s", configurationString );
  
  DataIO.SetDataType( "JSON" );

  DataLogging.SetBaseDirectory( "test" );
  
  DataHandle configuration = DataIO.GetDataHandler()->LoadStringData( configurationString );
  if( configuration != NULL )
  {
    ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
    memset( newController, 0, sizeof(ControlData) );
    
    newController->currentControlState = ROBOT_PASSIVE;
  
    newController->jointsNumber = DataIO.GetDataHandler()->GetListSize( configuration, "joints" );
    newController->jointIDsList = (JointID*) calloc( newController->jointsNumber, sizeof(JointID) );
    newController->jointSamplersList = (EMGJointSampler*) calloc( newController->jointsNumber, sizeof(EMGJointSampler) );
    newController->jointNamesList = (char**) calloc( newController->jointsNumber, sizeof(char*) );
    
    for( size_t jointIndex = 0; jointIndex < newController->jointsNumber; jointIndex++ )
    {
      newController->jointSamplersList[ jointIndex ] = (EMGJointSampler) malloc( sizeof(EMGJointSamplingData) );
      EMGJointSampler newSampler = newController->jointSamplersList[ jointIndex ];
      
      newController->jointNamesList[ jointIndex ] = DataIO.GetDataHandler()->GetStringValue( configuration, "", "joints.%lu", jointIndex );
      newController->jointIDsList[ jointIndex ] = EMGProcessing.InitJoint( newController->jointNamesList[ jointIndex ] );
      if( newController->jointIDsList[ jointIndex ] != EMG_JOINT_INVALID_ID )
      {
        newSampler->musclesCount = EMGProcessing.GetJointMusclesCount( newController->jointIDsList[ jointIndex ] );

        char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
        snprintf( filePath, LOG_FILE_PATH_MAX_LEN, "joints/%s_protocol", newController->jointNamesList[ jointIndex ] );
        newSampler->protocolLog = DataLogging.InitLog( filePath, 1 + 2 * newSampler->musclesCount + 5, 6 );
        newSampler->isLogging = false;
        
        DEBUG_PRINT( "Got new shared joint with ID %d and %u muscles", newController->jointIDsList[ jointIndex ], newSampler->musclesCount );
      }
    }
    
    DataIO.GetDataHandler()->UnloadData( configuration );

    DEBUG_PRINT( "robot control config %s loaded", configurationString );
    
    return (RobotController) newController;
  }
  
  return NULL;
}

void EndController( RobotController genericController )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  DEBUG_PRINT( "ending robot controller %p", controller );
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    DEBUG_PRINT( "ending joint control %lu", jointIndex ); 
      
    EMGProcessing.EndJoint( controller->jointIDsList[ jointIndex ] );
    
    DataLogging.EndLog( controller->jointSamplersList[ jointIndex ]->protocolLog );
    free( controller->jointSamplersList[ jointIndex ] );
    
    free( controller->jointNamesList[ jointIndex ] );
  }
  
  free( controller->jointNamesList );
  free( controller->jointIDsList );
  free( controller->jointSamplersList );
}

size_t GetJointsNumber( RobotController genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return controller->jointsNumber;
}

const char** GetJointNamesList( RobotController genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return (const char**) controller->jointNamesList;
}

size_t GetAxesNumber( RobotController genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return controller->jointsNumber;
}

const char** GetAxisNamesList( RobotController genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return (const char**) controller->jointNamesList;
}

void SetControlState( RobotController genericController, enum RobotState newControlState )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  DEBUG_PRINT( "setting new control state: %d", newControlState );
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    JointID jointID = controller->jointIDsList[ jointIndex ];
    EMGJointSampler sampler = controller->jointSamplersList[ jointIndex ];

    if( newControlState == ROBOT_OFFSET )
    {
      DEBUG_PRINT( "starting offset phase for joint %d", jointID );
      EMGProcessing.SetJointProcessingPhase( jointID, EMG_PROCESSING_OFFSET );
    }
    else if( newControlState == ROBOT_CALIBRATION )
    {
      DEBUG_PRINT( "starting calibration for joint %d", jointID );
      EMGProcessing.SetJointProcessingPhase( jointID, EMG_PROCESSING_CALIBRATION );
    }
    else if( newControlState == ROBOT_PREPROCESSING )
    {
      DEBUG_PRINT( "reseting sampling count for joint %d", jointID );
      EMGProcessing.SetJointProcessingPhase( jointID, EMG_PROCESSING_SAMPLING );
      sampler->samplesCount = 0;
    }
    else 
    {
      if( newControlState == ROBOT_OPERATION )
      {
        if( controller->currentControlState == ROBOT_PREPROCESSING && sampler->samplesCount > 0 )
        {
          DEBUG_PRINT( "starting optimization for shared joint %d", jointID );
          EMGProcessing.FitJointParameters( jointID, sampler );
        }
      }
      
      EMGProcessing.SetJointProcessingPhase( jointID, EMG_PROCESSING_MEASUREMENT );
    }
  }
  
  controller->currentControlState = newControlState;
}

void RunControlStep( RobotController genericController, RobotVariables** jointMeasuresTable, RobotVariables** axisMeasuresTable, RobotVariables** jointSetpointsTable, RobotVariables** axisSetpointsTable, double timeDelta )
{
  static double jointMuscleSignalsList[ EMG_JOINT_MAX_MUSCLES_NUMBER ];
  static double jointMuscleTorquesList[ EMG_JOINT_MAX_MUSCLES_NUMBER ];
  
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    double jointMeasuredAngle = jointMeasuresTable[ jointIndex ]->position * 360.0;
    double jointExternalTorque = jointMeasuresTable[ jointIndex ]->force;

    axisMeasuresTable[ jointIndex ]->position = jointMeasuresTable[ jointIndex ]->position;
    axisMeasuresTable[ jointIndex ]->velocity = jointMeasuresTable[ jointIndex ]->velocity;
    axisMeasuresTable[ jointIndex ]->acceleration = jointMeasuresTable[ jointIndex ]->acceleration;
    
    jointSetpointsTable[ jointIndex ]->position = axisSetpointsTable[ jointIndex ]->position;
    jointSetpointsTable[ jointIndex ]->velocity = axisSetpointsTable[ jointIndex ]->velocity;
    jointSetpointsTable[ jointIndex ]->acceleration = axisSetpointsTable[ jointIndex ]->acceleration;
    
    double jointSetpointAngle = jointMeasuresTable[ jointIndex ]->position * 360.0;
    
    JointID jointID = controller->jointIDsList[ jointIndex ];
    EMGJointSampler sampler = controller->jointSamplersList[ jointIndex ];
    
    EMGProcessing.GetJointMuscleSignals( jointID, jointMuscleSignalsList );
    //for( size_t muscleIndex = 0; muscleIndex < sampler->musclesCount; muscleIndex++ )
    //  jointMeasuresTable[ jointIndex ][ SHM_JOINT_EMG_1 + muscleIndex ] = jointMuscleSignalsList[ muscleIndex ];
    
    if( controller->currentControlState == ROBOT_PREPROCESSING )
    {
      if( sampler->samplesCount < EMG_MAX_SAMPLES_NUMBER )
      {
        double* currentSamplesList = sampler->muscleSignalsList[ sampler->samplesCount ];

        sampler->sampleTimesList[ sampler->samplesCount ] = Timing.GetExecTimeSeconds();

        for( size_t muscleIndex = 0; muscleIndex < sampler->musclesCount; muscleIndex++ ) 
          currentSamplesList[ muscleIndex ] = jointMuscleSignalsList[ muscleIndex ];

        sampler->measuredAnglesList[ sampler->samplesCount ] = jointMeasuredAngle;
        sampler->setpointAnglesList[ sampler->samplesCount ] = jointSetpointAngle;
        sampler->externalTorquesList[ sampler->samplesCount ] = jointExternalTorque;

        //DEBUG_PRINT( "Saving sample %u: %.3f, %.3f", sampler->samplesCount, jointMeasuredAngle, jointExternalTorque );

        sampler->samplesCount++;
      }
    }
    
    EMGProcessing.GetJointMuscleTorques( jointID, jointMuscleSignalsList, jointMeasuredAngle, jointExternalTorque, jointMuscleTorquesList );

    //DEBUG_PRINT( "Joint pos: %.3f - force: %.3f - stiff: %.3f", jointMeasuredAngle, jointMeasuresTable[ jointIndex ][ CONTROL_FORCE ], jointMeasuresTable[ jointIndex ][ CONTROL_STIFFNESS ] );
    
    jointSetpointsTable[ jointIndex ]->force = axisSetpointsTable[ jointIndex ]->force;
    jointSetpointsTable[ jointIndex ]->stiffness = axisSetpointsTable[ jointIndex ]->stiffness ;
    jointSetpointsTable[ jointIndex ]->damping = axisSetpointsTable[ jointIndex ]->damping;

    double setpointStiffness = jointSetpointsTable[ jointIndex ]->stiffness ;
    double positionError = jointSetpointsTable[ jointIndex ]->position - jointMeasuresTable[ jointIndex ]->position;
    
    if( controller->currentControlState != ROBOT_PREPROCESSING && controller->currentControlState != ROBOT_OPERATION ) setpointStiffness = 0.0;
    else if( controller->currentControlState == ROBOT_OPERATION )
    {
       double jointEMGTorque = EMGProcessing.GetJointTorque( jointID, jointMuscleTorquesList );
       double jointEMGStiffness = EMGProcessing.GetJointStiffness( jointID, jointMuscleTorquesList );
       
       axisMeasuresTable[ jointIndex ]->force = jointEMGTorque;
       axisMeasuresTable[ jointIndex ]->stiffness  = jointEMGStiffness;  
       
       double targetStiffness = 1.0 / jointEMGStiffness;
       setpointStiffness = 0.99 * setpointStiffness + 0.01 * targetStiffness;
       
       jointMeasuresTable[ jointIndex ]->stiffness  = setpointStiffness;  
       
       if( controller->currentControlState == ROBOT_OPERATION )
       {
         DataLogging.RegisterValues( sampler->protocolLog, 1, Timing.GetExecTimeSeconds() );
         DataLogging.RegisterList( sampler->protocolLog, sampler->musclesCount, jointMuscleSignalsList );
         DataLogging.RegisterList( sampler->protocolLog, sampler->musclesCount, jointMuscleTorquesList );
         DataLogging.RegisterValues( sampler->protocolLog, 5, jointMeasuredAngle, jointExternalTorque, jointEMGTorque, targetStiffness, jointEMGStiffness );
         sampler->isLogging = false; 
       }
       
       DEBUG_PRINT( "emg:%.5f - t:%.5f - s:%.5f", jointEMGStiffness, targetStiffness, setpointStiffness );
    }
    
    jointSetpointsTable[ jointIndex ]->force = setpointStiffness * positionError;
  }
}
