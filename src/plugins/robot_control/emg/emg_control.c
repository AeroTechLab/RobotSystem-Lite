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

#include "sensors.h" 
#include "robot_control_interface.h"

#include "timing/timing.h"

#include "debug/data_logging.h"

#include "data_io.h"

#include <stdlib.h>
#include <math.h>
#include <string.h> 


typedef struct _EMGReader
{
  Sensor sensor;
  double* rawBuffer;
  size_t rawBufferLength;
}
EMGReader;

typedef struct _ControlData 
{
  enum RobotState currentControlState;
  EMGModel emgModel;
  EMGSamplingData samplingData;
  EMGReader* emgReadersList;
  size_t jointsNumber, musclesNumber;
  Log offsetLog, calibrationLog, samplingLog;
  Log currentLog, emgRawLog;
}
ControlData;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


RobotController InitController( const char* configurationString )
{
  //DEBUG_PRINT( "Trying to load robot control config %s", configurationString );

  Log_SetBaseDirectory( "test" );
  
  ControlData* newController = (ControlData*) malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
    
  newController->currentControlState = ROBOT_PASSIVE;
  
  if( (newController->emgModel = EMGProcessing_InitModel( configurationString )) == NULL )
  {
    free( newController );
    return NULL;
  }
    
  newController->jointsNumber = EMGProcessing_GetJointsCount( newController->emgModel );
    
  newController->musclesNumber = EMGProcessing_GetMusclesCount( newController->emgModel );
  newController->emgReadersList = (EMGReader*) calloc( newController->musclesNumber, sizeof(EMGReader) );
  const char** muscleNamesList = EMGProcessing_GetMuscleNames( newController->emgModel );
  for( size_t muscleIndex = 0; muscleIndex < newController->musclesNumber; muscleIndex++ )
  {
    char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
    snprintf( filePath, DATA_IO_MAX_FILE_PATH_LENGTH, "emg/%s", muscleNamesList[ muscleIndex ] );
    DataHandle sensorData = DataIO_LoadFileData( filePath );
    newController->emgReadersList[ muscleIndex ].sensor = Sensor_Init( sensorData );
    DataIO_UnloadData( sensorData );
    newController->emgReadersList[ muscleIndex ].rawBufferLength = Sensor_GetInputBufferLength( newController->emgReadersList[ muscleIndex ].sensor );
    newController->emgReadersList[ muscleIndex ].rawBuffer = (double*) calloc( newController->emgReadersList[ muscleIndex ].rawBufferLength, sizeof(double) );
  }
    
  newController->offsetLog = Log_Init( "joints/offset", DATA_LOG_MAX_PRECISION );
  newController->calibrationLog = Log_Init( "joints/calibration", DATA_LOG_MAX_PRECISION );
  newController->samplingLog = Log_Init( "joints/sampling", DATA_LOG_MAX_PRECISION );
  newController->emgRawLog = Log_Init( "joints/raw", 6 );
  newController->currentLog = NULL;

  //DEBUG_PRINT( "robot control config %s loaded", configurationString );
   
  return (RobotController) newController;
}

void EndController( RobotController genericController )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  //DEBUG_PRINT( "ending robot controller %p", controller );
  
  EMGProcessing_EndModel( controller->emgModel );
  
  for( size_t muscleIndex = 0; muscleIndex < controller->musclesNumber; muscleIndex++ )
  {
    Sensor_End( controller->emgReadersList[ muscleIndex ].sensor );
    free( controller->emgReadersList[ muscleIndex ].rawBuffer );
  }
  free( controller->emgReadersList );
  
  Log_End( controller->offsetLog );
  Log_End( controller->calibrationLog );
  Log_End( controller->samplingLog );
  Log_End( controller->emgRawLog );
}

size_t GetJointsNumber( RobotController genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return EMGProcessing_GetJointsCount( controller->emgModel );
}

const char** GetJointNamesList( RobotController genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return EMGProcessing_GetJointNames( controller->emgModel );
}

size_t GetAxesNumber( RobotController genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return EMGProcessing_GetJointsCount( controller->emgModel );   
}

const char** GetAxisNamesList( RobotController genericController )
{
  if( genericController == NULL ) return 0;
  
  ControlData* controller = (ControlData*) genericController;
  
  return EMGProcessing_GetJointNames( controller->emgModel );
}

void SetControlState( RobotController genericController, enum RobotState newControlState )
{
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  //DEBUG_PRINT( "setting new control state: %d", newControlState );
  
  enum SigProcState signalProcessingState = SIG_PROC_STATE_MEASUREMENT;
  
  if( newControlState == ROBOT_OFFSET )
  {
    //DEBUG_PRINT( "starting offset phase for joint %d", jointID );
    controller->currentLog = controller->offsetLog;
    signalProcessingState = SIG_PROC_STATE_OFFSET;
  }
  else if( newControlState == ROBOT_CALIBRATION )
  {
    //DEBUG_PRINT( "starting calibration for joint %d", jointID );
    controller->currentLog = controller->calibrationLog;
    signalProcessingState = SIG_PROC_STATE_CALIBRATION;
  }
  else if( newControlState == ROBOT_PREPROCESSING )
  {
    //DEBUG_PRINT( "reseting sampling count for joint %d", jointID );
    controller->currentLog = controller->samplingLog;
    controller->samplingData.samplesCount = 0;
  }
  else 
  {
  //  if( newControlState == ROBOT_OPERATION )
  //  {
  //    if( controller->currentControlState == ROBOT_PREPROCESSING && sampler->samplesCount > 0 )
  //    {
        //DEBUG_PRINT( "starting optimization for shared joint %d", jointID );
  //      EMGProcessing_FitJointParameters( jointID, sampler );
  //    }
  //  }
    controller->currentLog = NULL;
  }
  
  for( size_t muscleIndex = 0; muscleIndex < controller->musclesNumber; muscleIndex++ )
    Sensor_SetState( controller->emgReadersList[ muscleIndex ].sensor, signalProcessingState );
  
  controller->currentControlState = newControlState;
}

void RegisterValues( ControlData* controller, double* muscleActivationsList, double* jointAnglesList, double* jointRefAnglesList, double* jointExtTorquesList )
{
  double currentTime = Time_GetExecSeconds();
  
  if( controller->currentControlState == ROBOT_PREPROCESSING && controller->samplingData.samplesCount < EMG_MAX_SAMPLES_NUMBER )
  {
    controller->samplingData.sampleTimesList[ controller->samplingData.samplesCount ] = currentTime;

    for( size_t muscleIndex = 0; muscleIndex < controller->musclesNumber; muscleIndex++ ) 
      controller->samplingData.muscleSignalsList[ muscleIndex ][ controller->samplingData.samplesCount ] = muscleActivationsList[ muscleIndex ];

    for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
    {
      controller->samplingData.measuredAnglesList[ jointIndex ][ controller->samplingData.samplesCount ] = jointAnglesList[ jointIndex ];
      controller->samplingData.setpointAnglesList[ jointIndex ][ controller->samplingData.samplesCount ] = jointRefAnglesList[ jointIndex ];
      controller->samplingData.externalTorquesList[ jointIndex ][ controller->samplingData.samplesCount ] = jointExtTorquesList[ jointIndex ];
    }

    //DEBUG_PRINT( "Saving sample %u: %.3f, %.3f", sampler->samplesCount, jointMeasuredAngle, jointExternalTorque );

    controller->samplingData.samplesCount++;
  }
  
  if( controller->currentLog != NULL )
  {
    Log_EnterNewLine( controller->currentLog, currentTime );
    Log_RegisterList( controller->currentLog, controller->jointsNumber, jointRefAnglesList );
    Log_RegisterList( controller->currentLog, controller->jointsNumber, jointAnglesList );
    Log_RegisterList( controller->currentLog, controller->jointsNumber, jointExtTorquesList );
    Log_RegisterList( controller->currentLog, controller->musclesNumber, muscleActivationsList );
  
    if( controller->emgRawLog != NULL )
    {
      Log_EnterNewLine( controller->emgRawLog, currentTime );
      for( size_t muscleIndex = 0; muscleIndex < controller->musclesNumber; muscleIndex++ )
        Log_RegisterList( controller->emgRawLog, controller->emgReadersList[ muscleIndex ].rawBufferLength, controller->emgReadersList[ muscleIndex ].rawBuffer );
    }
  }
}

void RunControlStep( RobotController genericController, RobotVariables** jointMeasuresTable, RobotVariables** axisMeasuresTable, RobotVariables** jointSetpointsTable, RobotVariables** axisSetpointsTable, double timeDelta )
{
  static double muscleActivationsList[ EMG_MAX_MUSCLES_NUMBER ];
  static double jointAnglesList[ EMG_MAX_JOINTS_NUMBER ], jointRefAnglesList[ EMG_MAX_JOINTS_NUMBER ];
  static double jointTorquesList[ EMG_MAX_JOINTS_NUMBER ], jointStiffnessesList[ EMG_MAX_JOINTS_NUMBER ];
  
  if( genericController == NULL ) return;
  
  ControlData* controller = (ControlData*) genericController;
  
  for( size_t muscleIndex = 0; muscleIndex < controller->musclesNumber; muscleIndex++ )
    muscleActivationsList[ muscleIndex ] = Sensor_Update( controller->emgReadersList[ muscleIndex ].sensor, controller->emgReadersList[ muscleIndex ].rawBuffer );
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    jointAnglesList[ jointIndex ] = jointMeasuresTable[ jointIndex ]->position * 360.0;
    jointTorquesList[ jointIndex ] = jointMeasuresTable[ jointIndex ]->force;

    axisMeasuresTable[ jointIndex ]->position = jointMeasuresTable[ jointIndex ]->position;
    axisMeasuresTable[ jointIndex ]->velocity = jointMeasuresTable[ jointIndex ]->velocity;
    axisMeasuresTable[ jointIndex ]->acceleration = jointMeasuresTable[ jointIndex ]->acceleration;
    
    jointSetpointsTable[ jointIndex ]->position = axisSetpointsTable[ jointIndex ]->position;
    jointSetpointsTable[ jointIndex ]->velocity = axisSetpointsTable[ jointIndex ]->velocity;
    jointSetpointsTable[ jointIndex ]->acceleration = axisSetpointsTable[ jointIndex ]->acceleration;
    
    jointRefAnglesList[ jointIndex ] = jointSetpointsTable[ jointIndex ]->position * 360.0;

    //DEBUG_PRINT( "Joint pos: %.3f - force: %.3f - stiff: %.3f", jointAnglesList[ jointIndex ], jointMeasuresTable[ jointIndex ]->force, jointMeasuresTable[ jointIndex ]->stiffness );
    
    jointSetpointsTable[ jointIndex ]->force = axisSetpointsTable[ jointIndex ]->force;
    jointSetpointsTable[ jointIndex ]->stiffness = axisSetpointsTable[ jointIndex ]->stiffness ;
    jointSetpointsTable[ jointIndex ]->damping = axisSetpointsTable[ jointIndex ]->damping;
  }
  
  RegisterValues( controller, muscleActivationsList, jointAnglesList, jointRefAnglesList, jointTorquesList );
  
  EMGProcessing_RunStep( controller->emgModel, muscleActivationsList, jointAnglesList, jointTorquesList );
  
  EMGProcessing_GetJointTorques( controller->emgModel, jointTorquesList );
  EMGProcessing_GetJointStiffnesses( controller->emgModel, jointStiffnessesList ); 
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    double setpointForce = jointSetpointsTable[ jointIndex ]->force;
    double positionError = jointSetpointsTable[ jointIndex ]->position - jointMeasuresTable[ jointIndex ]->position;
    
    if( controller->currentControlState != ROBOT_PREPROCESSING && controller->currentControlState != ROBOT_OPERATION ) setpointForce = 0.0;
    else if( controller->currentControlState == ROBOT_OPERATION )
    {
       axisMeasuresTable[ jointIndex ]->force = jointTorquesList[ jointIndex ];
       axisMeasuresTable[ jointIndex ]->stiffness  = jointStiffnessesList[ jointIndex ];  
       
       double targetStiffness = 1.0 / jointStiffnessesList[ jointIndex ];
       setpointForce = 0.99 * setpointForce + 0.01 * targetStiffness * positionError;
       
       jointMeasuresTable[ jointIndex ]->stiffness = jointStiffnessesList[ jointIndex ];  
       
       //DEBUG_PRINT( "emg:%.5f - t:%.5f - s:%.5f", jointEMGStiffness, targetStiffness, setpointStiffness );
    }
    
    jointSetpointsTable[ jointIndex ]->force = setpointForce;
  }
}
