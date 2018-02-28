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
  bool* jointsChangedList;
  double* jointForceErrorsList;
  double* jointStiffnessesList;
  double* jointVelocitySetpointsList;
  size_t jointsNumber, musclesNumber;
  Log offsetLog, calibrationLog, samplingLog, operationLog;
  Log currentLog, emgRawLog;
}
ControlData;


DECLARE_MODULE_INTERFACE( ROBOT_CONTROL_INTERFACE );


RobotController InitController( const char* configurationString )
{
  DEBUG_PRINT( "Trying to load EMG control config %s", configurationString );

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
  newController->jointsChangedList = (bool*) calloc( newController->jointsNumber, sizeof(bool) );
  newController->jointForceErrorsList = (double*) calloc( newController->jointsNumber, sizeof(double) );
  newController->jointStiffnessesList = (double*) calloc( newController->jointsNumber, sizeof(double) );
  newController->jointVelocitySetpointsList = (double*) calloc( newController->jointsNumber, sizeof(double) );
    
  newController->musclesNumber = EMGProcessing_GetMusclesCount( newController->emgModel );
  newController->emgReadersList = (EMGReader*) calloc( newController->musclesNumber, sizeof(EMGReader) );
  const char** muscleNamesList = EMGProcessing_GetMuscleNames( newController->emgModel );
  for( size_t muscleIndex = 0; muscleIndex < newController->musclesNumber; muscleIndex++ )
  {
    DEBUG_PRINT( "initializing sensor for muscle %s", muscleNamesList[ muscleIndex ] );
    char filePath[ DATA_IO_MAX_FILE_PATH_LENGTH ];
    snprintf( filePath, DATA_IO_MAX_FILE_PATH_LENGTH, "./config/sensors/emg/%s", muscleNamesList[ muscleIndex ] );
    DataHandle sensorData = DataIO_LoadFileData( filePath );
    newController->emgReadersList[ muscleIndex ].sensor = Sensor_Init( sensorData );
    DataIO_UnloadData( sensorData );
    newController->emgReadersList[ muscleIndex ].rawBufferLength = Sensor_GetInputBufferLength( newController->emgReadersList[ muscleIndex ].sensor );
    newController->emgReadersList[ muscleIndex ].rawBuffer = (double*) calloc( newController->emgReadersList[ muscleIndex ].rawBufferLength, sizeof(double) );
  }
    
  newController->offsetLog = Log_Init( "joints/offset", DATA_LOG_MAX_PRECISION );
  newController->calibrationLog = Log_Init( "joints/calibration", DATA_LOG_MAX_PRECISION );
  newController->samplingLog = Log_Init( "joints/sampling", DATA_LOG_MAX_PRECISION );
  newController->operationLog = Log_Init( "joints/operation", DATA_LOG_MAX_PRECISION );
  newController->emgRawLog = Log_Init( "joints/raw", 6 );
  newController->currentLog = NULL;

  DEBUG_PRINT( "robot control config %s loaded", configurationString );
   
  return (RobotController) newController;
}

void EndController( RobotController ref_controller )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  DEBUG_PRINT( "ending robot controller %p", controller );
  
  EMGProcessing_EndModel( controller->emgModel );
  
  for( size_t muscleIndex = 0; muscleIndex < controller->musclesNumber; muscleIndex++ )
  {
    Sensor_End( controller->emgReadersList[ muscleIndex ].sensor );
    free( controller->emgReadersList[ muscleIndex ].rawBuffer );
  }
  free( controller->emgReadersList );
  
  free( controller->jointsChangedList );
  free( controller->jointForceErrorsList );
  free( controller->jointStiffnessesList );
  free( controller->jointVelocitySetpointsList );
  
  Log_End( controller->offsetLog );
  Log_End( controller->calibrationLog );
  Log_End( controller->samplingLog );
  Log_End( controller->operationLog );
  Log_End( controller->emgRawLog );
}

size_t GetJointsNumber( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return EMGProcessing_GetJointsCount( controller->emgModel );
}

const char** GetJointNamesList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return EMGProcessing_GetJointNames( controller->emgModel );
}

const bool* GetJointsChangedList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const bool*) controller->jointsChangedList;
}

size_t GetAxesNumber( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return EMGProcessing_GetJointsCount( controller->emgModel );   
}

const char** GetAxisNamesList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return EMGProcessing_GetJointNames( controller->emgModel );
}

const bool* GetAxesChangedList( RobotController ref_controller )
{
  if( ref_controller == NULL ) return 0;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  return (const bool*) controller->jointsChangedList;
}

void SetControlState( RobotController ref_controller, enum RobotState newControlState )
{
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  //DEBUG_PRINT( "setting new control state: %d", newControlState );
  
  enum SensorState sensorsState = SENSOR_STATE_MEASUREMENT;
  
  if( newControlState == ROBOT_OFFSET )
  {
    DEBUG_PRINT( "starting offset phase for controller %p", ref_controller );
    controller->currentLog = controller->offsetLog;
    sensorsState = SENSOR_STATE_OFFSET;
  }
  else if( newControlState == ROBOT_CALIBRATION )
  {
    DEBUG_PRINT( "starting calibration for controller %p", ref_controller );
    controller->currentLog = controller->calibrationLog;
    sensorsState = SENSOR_STATE_CALIBRATION;
  }
  else if( newControlState == ROBOT_PREPROCESSING )
  {
    DEBUG_PRINT( "reseting sampling count for controller %p", ref_controller );
    controller->currentLog = controller->samplingLog;
    controller->samplingData.samplesCount = 0;
  }
  else if( newControlState == ROBOT_OPERATION )
  {
    DEBUG_PRINT( "starting operation for controller %p", ref_controller );
    controller->currentLog = controller->operationLog;
  //  if( controller->currentControlState == ROBOT_PREPROCESSING && controller->samplingData.samplesCount > 0 )
  //  {
      //DEBUG_PRINT( "starting optimization for controller %p", ref_controller );
  //    EMGProcessing_FitParameters( controller->emgModel, &(controller->samplingData) );
  //  }
  }
  else 
  {
    controller->currentLog = NULL;
  }
  
  for( size_t muscleIndex = 0; muscleIndex < controller->musclesNumber; muscleIndex++ )
    Sensor_SetState( controller->emgReadersList[ muscleIndex ].sensor, sensorsState );
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    controller->jointVelocitySetpointsList[ jointIndex ] = 0.0;
    controller->jointStiffnessesList[ jointIndex ] = 0.0;
  }
  
  controller->currentControlState = newControlState;
}

void RegisterValues( ControlData* controller, double* muscleActivationsList, double* jointAnglesList, double* jointRefAnglesList, double* jointVelocitiesList, double* jointExtTorquesList )
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
    Log_RegisterList( controller->currentLog, controller->jointsNumber, jointVelocitiesList );
    Log_RegisterList( controller->currentLog, controller->jointsNumber, jointExtTorquesList );
    Log_RegisterList( controller->currentLog, controller->musclesNumber, muscleActivationsList );
  
    if( controller->emgRawLog != NULL )
    {
      Log_EnterNewLine( controller->emgRawLog, currentTime );
      Log_RegisterList( controller->emgRawLog, controller->jointsNumber, jointRefAnglesList );
      Log_RegisterList( controller->emgRawLog, controller->jointsNumber, jointAnglesList );
      Log_RegisterList( controller->emgRawLog, controller->jointsNumber, jointExtTorquesList );
      for( size_t muscleIndex = 0; muscleIndex < controller->musclesNumber; muscleIndex++ )
        Log_RegisterList( controller->emgRawLog, controller->emgReadersList[ muscleIndex ].rawBufferLength, controller->emgReadersList[ muscleIndex ].rawBuffer );
    }
  }
}

void ControlJoint( RobotVariables* ref_jointMeasures, RobotVariables* ref_jointSetpoints, double* ref_previousForceError, double* ref_velocitySetpoint, double timeDelta )
{
  const double K_P = 370;// * 150 (reduction)
  const double K_I = 3.5;// * 150 (reduction) 
  
  double positionError = ref_jointSetpoints->position - ref_jointMeasures->position;    // e_p = x_d - x
  double velocityError = ref_jointSetpoints->velocity - ref_jointMeasures->velocity;    // e_v = xdot_d - xdot
  // F_actuator = K * e_p + B * e_v - D * x_dot
  ref_jointSetpoints->force = ref_jointSetpoints->stiffness * positionError - ref_jointSetpoints->damping * velocityError;
  
  double forceError = ref_jointSetpoints->force - ref_jointMeasures->force;
  *ref_velocitySetpoint += K_P * ( forceError - (*ref_previousForceError) ) + K_I * timeDelta * forceError;
  //DEBUG_PRINT( "control: %.3f + %g*(%.5f-%.5f) + %g*%.5f*%.5f = %.3f", ref_jointSetpoints->velocity, K_P, forceError, *ref_previousForceError, K_I, timeDelta, forceError, ref_jointSetpoints->velocity );
  ref_jointSetpoints->velocity = *ref_velocitySetpoint;
  *ref_previousForceError = forceError;
}

void RunControlStep( RobotController ref_controller, RobotVariables** jointMeasuresTable, RobotVariables** axisMeasuresTable, RobotVariables** jointSetpointsTable, RobotVariables** axisSetpointsTable, double timeDelta )
{
  static double muscleActivationsList[ EMG_MAX_MUSCLES_NUMBER ];
  static double jointAnglesList[ EMG_MAX_JOINTS_NUMBER ], jointRefAnglesList[ EMG_MAX_JOINTS_NUMBER ], jointVelocitiesList[ EMG_MAX_JOINTS_NUMBER ];
  static double jointTorquesList[ EMG_MAX_JOINTS_NUMBER ], jointStiffnessesList[ EMG_MAX_JOINTS_NUMBER ];
  
  if( ref_controller == NULL ) return;
  
  ControlData* controller = (ControlData*) ref_controller;
  
  for( size_t muscleIndex = 0; muscleIndex < controller->musclesNumber; muscleIndex++ )
    muscleActivationsList[ muscleIndex ] = Sensor_Update( controller->emgReadersList[ muscleIndex ].sensor, controller->emgReadersList[ muscleIndex ].rawBuffer );
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    //jointMeasuresTable[ jointIndex ]->position -= M_PI / 2.0;
    jointAnglesList[ jointIndex ] = jointMeasuresTable[ jointIndex ]->position * 180.0 / M_PI;
    jointVelocitiesList[ jointIndex ] = jointMeasuresTable[ jointIndex ]->velocity * 180.0 / M_PI;
    jointTorquesList[ jointIndex ] = jointMeasuresTable[ jointIndex ]->force;

    controller->jointsChangedList[ jointIndex ] = false;
    
    if( fabs( jointMeasuresTable[ jointIndex ]->position - axisMeasuresTable[ jointIndex ]->position ) > 0.01 )
    {
      axisMeasuresTable[ jointIndex ]->position = jointMeasuresTable[ jointIndex ]->position;
      axisMeasuresTable[ jointIndex ]->velocity = jointMeasuresTable[ jointIndex ]->velocity;
      axisMeasuresTable[ jointIndex ]->acceleration = jointMeasuresTable[ jointIndex ]->acceleration;
      axisMeasuresTable[ jointIndex ]->force = jointMeasuresTable[ jointIndex ]->force;
      
      controller->jointsChangedList[ jointIndex ] = true;
    }
    
    jointSetpointsTable[ jointIndex ]->position = axisSetpointsTable[ jointIndex ]->position;
    jointSetpointsTable[ jointIndex ]->velocity = axisSetpointsTable[ jointIndex ]->velocity;
    jointSetpointsTable[ jointIndex ]->acceleration = axisSetpointsTable[ jointIndex ]->acceleration;
    
    jointRefAnglesList[ jointIndex ] = jointSetpointsTable[ jointIndex ]->position * 180.0 / M_PI;
    
    jointSetpointsTable[ jointIndex ]->force = axisSetpointsTable[ jointIndex ]->force;
    jointSetpointsTable[ jointIndex ]->stiffness = axisSetpointsTable[ jointIndex ]->stiffness ;
    jointSetpointsTable[ jointIndex ]->damping = axisSetpointsTable[ jointIndex ]->damping;
  }
  
  RegisterValues( controller, muscleActivationsList, jointAnglesList, jointRefAnglesList, jointVelocitiesList, jointTorquesList );
  
  EMGProcessing_RunStep( controller->emgModel, muscleActivationsList, jointAnglesList, jointVelocitiesList, jointTorquesList );
  
  EMGProcessing_GetJointTorques( controller->emgModel, jointTorquesList );
  EMGProcessing_GetJointStiffnesses( controller->emgModel, jointStiffnessesList ); 
  
  for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
  {
    if( controller->currentControlState == ROBOT_OPERATION )
    {
      axisMeasuresTable[ jointIndex ]->force = jointTorquesList[ jointIndex ];
      axisMeasuresTable[ jointIndex ]->stiffness  = jointStiffnessesList[ jointIndex ];  
      
      const double MAX_STIFFNESS = 60.0;
      const double UPDATE_FACTOR = 0.05;
      const double ERROR_CORRECTION_FACTOR = 20.0;
      double targetStiffness = ( jointStiffnessesList[ jointIndex ] == 0.0 ) ? MAX_STIFFNESS : 500.0 / jointStiffnessesList[ jointIndex ];
      targetStiffness += ERROR_CORRECTION_FACTOR * fabs( jointSetpointsTable[ jointIndex ]->position - jointMeasuresTable[ jointIndex ]->position );
      double complementaryStiffness = MAX_STIFFNESS - jointStiffnessesList[ jointIndex ];
      if( targetStiffness > complementaryStiffness ) targetStiffness = complementaryStiffness;
      if( jointSetpointsTable[ jointIndex ]->stiffness > 0.0 ) targetStiffness = jointSetpointsTable[ jointIndex ]->stiffness;
      if( targetStiffness < 0.0 ) targetStiffness = 0.0;
      controller->jointStiffnessesList[ jointIndex ] = ( 1.0 - UPDATE_FACTOR ) * controller->jointStiffnessesList[ jointIndex ] + UPDATE_FACTOR * targetStiffness;
      jointSetpointsTable[ jointIndex ]->stiffness = controller->jointStiffnessesList[ jointIndex ];
      
      //DEBUG_PRINT( "stiffness: user=%g, robot=%g", axisMeasuresTable[ jointIndex ]->stiffness, jointSetpointsTable[ jointIndex ]->stiffness );
    }
    
    ControlJoint( jointMeasuresTable[ jointIndex ], jointSetpointsTable[ jointIndex ], &(controller->jointForceErrorsList[ jointIndex ]), &(controller->jointVelocitySetpointsList[ jointIndex ]), timeDelta );
  }
  
  if( controller->currentControlState == ROBOT_OPERATION )
  {
    Log_RegisterList( controller->currentLog, controller->jointsNumber, jointTorquesList );
    Log_RegisterList( controller->currentLog, controller->jointsNumber, jointStiffnessesList );
    for( size_t jointIndex = 0; jointIndex < controller->jointsNumber; jointIndex++ )
      Log_RegisterValues( controller->currentLog, 1, jointSetpointsTable[ jointIndex ]->stiffness );
  }
}
