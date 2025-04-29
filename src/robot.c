////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2025 Leonardo Consoni <leonardojc@protonmail.com>      //
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


#include "robot.h"

#include "config_keys.h"

#include "actuator.h"

#include "input.h"
#include "output.h"

#include "data_io/interface/data_io.h"
#include "threads/threads.h"
#include "timing/timing.h"
#include "debug/data_logging.h"

#include "linearizer/system_linearizer.h"

#include <stdlib.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICE                             /////
/////////////////////////////////////////////////////////////////////////////////

typedef struct _RobotData
{
  DECLARE_MODULE_INTERFACE_REF( ROBOT_CONTROL_INTERFACE );
  Thread controlThread;
  volatile bool isControlRunning;
  enum ControlState controlState;
  double controlTimeStep;
  Actuator* actuatorsList;
  DoFVariables** jointMeasuresList;
  DoFVariables** jointSetpointsList;
  LinearSystem* jointLinearizersList;
  size_t jointsNumber;
  DoFVariables** axisMeasuresList;
  DoFVariables** axisSetpointsList;
  size_t axesNumber;
  Input* extraInputsList;
  double* extraInputValuesList;
  size_t extraInputsNumber;
  Output* extraOutputsList;
  double* extraOutputValuesList;
  size_t extraOutputsNumber;
  Log controlLog;
} 
RobotData;

static RobotData robot;


const double CONTROL_PASS_DEFAULT_INTERVAL = 0.005;

static void* AsyncControl( void* );

bool Robot_Init( const char* configName )
{
  char filePath[ DATA_IO_MAX_PATH_LENGTH ];

  DEBUG_PRINT( "trying to initialize robot %s", configName );
  
  bool loadSuccess = false;
  
  sprintf( filePath, KEY_CONFIG "/" KEY_ROBOTS "/%s", configName );
  DataHandle configuration = DataIO_LoadStorageData( filePath );
  if( configuration != NULL )
  {
    sprintf( filePath, KEY_MODULES "/" KEY_ROBOT_CONTROL "/%s", DataIO_GetStringValue( configuration, "", KEY_CONTROLLER "." KEY_TYPE ) );
    LOAD_MODULE_IMPLEMENTATION( ROBOT_CONTROL_INTERFACE, filePath, &robot, &loadSuccess );
    if( loadSuccess )
    {
      //PRINT_PLUGIN_FUNCTIONS( ROBOT_CONTROL_INTERFACE, (&robot) );
      const char* controllerConfigString = DataIO_GetStringValue( configuration, "", KEY_CONTROLLER "." KEY_CONFIG );
      DEBUG_PRINT( "loading controller config %s", controllerConfigString ); 
      if( (loadSuccess = robot.InitController( controllerConfigString )) )
      {
        robot.controlTimeStep = DataIO_GetNumericValue( configuration, CONTROL_PASS_DEFAULT_INTERVAL, KEY_CONTROLLER "." KEY_TIME_STEP );   
        robot.jointsNumber = robot.GetJointsNumber();
        robot.actuatorsList = (Actuator*) calloc( robot.jointsNumber, sizeof(Actuator) );
        robot.jointMeasuresList = (DoFVariables**) calloc( robot.jointsNumber, sizeof(DoFVariables*) );
        robot.jointSetpointsList = (DoFVariables**) calloc( robot.jointsNumber, sizeof(DoFVariables*) );
        robot.jointLinearizersList = (LinearSystem*) calloc( robot.jointsNumber, sizeof(LinearSystem) );
        DEBUG_PRINT( "found %lu joints", robot.jointsNumber );
        for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
        {
          const char* actuatorName = DataIO_GetStringValue( configuration, "", KEY_ACTUATORS ".%lu", jointIndex );
          robot.actuatorsList[ jointIndex ] = Actuator_Init( actuatorName );
          robot.jointMeasuresList[ jointIndex ] = (DoFVariables*) malloc( sizeof(DoFVariables) );
          robot.jointSetpointsList[ jointIndex ] = (DoFVariables*) malloc( sizeof(DoFVariables) );
          robot.jointLinearizersList[ jointIndex ] = SystemLinearizer_CreateSystem( 3, 1, LINEARIZATION_MAX_SAMPLES );
        }

        robot.axesNumber = robot.GetAxesNumber();
        robot.axisMeasuresList = (DoFVariables**) calloc( robot.axesNumber, sizeof(DoFVariables*) );
        robot.axisSetpointsList = (DoFVariables**) calloc( robot.axesNumber, sizeof(DoFVariables*) );
        DEBUG_PRINT( "found %lu axes", robot.axesNumber );
        for( size_t axisIndex = 0; axisIndex < robot.axesNumber; axisIndex++ )
        {
          robot.axisMeasuresList[ axisIndex ] = (DoFVariables*) malloc( sizeof(DoFVariables) );
          robot.axisSetpointsList[ axisIndex ] = (DoFVariables*) malloc( sizeof(DoFVariables) );
        }
        
        robot.extraInputsNumber = robot.GetExtraInputsNumber();
        robot.extraInputsList = (Input*) calloc( robot.extraInputsNumber, sizeof(Input) );
        for( size_t inputIndex = 0; inputIndex < robot.extraInputsNumber; inputIndex++ )
          robot.extraInputsList[ inputIndex ] = Input_Init( DataIO_GetSubData( configuration, KEY_EXTRA_INPUTS ".%lu", inputIndex ) );
        robot.extraInputValuesList = (double*) calloc( robot.extraInputsNumber, sizeof(double) );
        
        robot.extraOutputsNumber = robot.GetExtraOutputsNumber();
        robot.extraOutputsList = (Output*) calloc( robot.extraOutputsNumber, sizeof(Output) );
        for( size_t outputIndex = 0; outputIndex < robot.extraOutputsNumber; outputIndex++ )
          robot.extraOutputsList[ outputIndex ] = Output_Init( DataIO_GetSubData( configuration, KEY_EXTRA_OUTPUTS ".%lu", outputIndex ) );
        robot.extraOutputValuesList = (double*) calloc( robot.extraOutputsNumber, sizeof(double) );
        
        if( DataIO_HasKey( configuration, KEY_LOG ) )
          robot.controlLog = Log_Init( DataIO_GetBooleanValue( configuration, false, KEY_LOG "." KEY_FILE ) ? configName : "", 
                                       (size_t) DataIO_GetNumericValue( configuration, 3, KEY_LOG "." KEY_PRECISION ) );
        
        DEBUG_PRINT( "robot %s initialized", configName );
      }
    }
    
    DataIO_UnloadData( configuration );

    if( !loadSuccess ) Robot_End();
    
    // testing hack
    //Robot_Enable();
    //Robot_SetControlState( CONTROL_OPERATION );
  }
  
  return loadSuccess;
}

void Robot_End()
{
  Robot_Disable();
  
  if( robot.EndController != NULL ) robot.EndController();
  
  for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
  {
    Actuator_End( robot.actuatorsList[ jointIndex ] );
    free( robot.jointMeasuresList[ jointIndex ] );
    free( robot.jointSetpointsList[ jointIndex ] );
    SystemLinearizer_DeleteSystem( robot.jointLinearizersList[ jointIndex ] );
  }
  free( robot.actuatorsList );
  free( robot.jointMeasuresList );
  free( robot.jointSetpointsList );
  
  for( size_t axisIndex = 0; axisIndex < robot.axesNumber; axisIndex++ )
  {
    free( robot.axisMeasuresList[ axisIndex ] );
    free( robot.axisSetpointsList[ axisIndex ] );
  }
  free( robot.axisMeasuresList );
  free( robot.axisSetpointsList );
    
  for( size_t inputIndex = 0; inputIndex < robot.extraInputsNumber; inputIndex++ )
    Input_End( robot.extraInputsList[ inputIndex ] );
  if( robot.extraInputsList != NULL ) free( robot.extraInputsList );
  if( robot.extraInputValuesList != NULL ) free( robot.extraInputValuesList );
  
  for( size_t outputIndex = 0; outputIndex < robot.extraOutputsNumber; outputIndex++ )
    Output_End( robot.extraOutputsList[ outputIndex ] );
  if( robot.extraOutputsList != NULL ) free( robot.extraOutputsList );
  if( robot.extraOutputValuesList != NULL ) free( robot.extraOutputValuesList );
  
  Log_End( robot.controlLog );
  
  memset( &robot, 0, sizeof(RobotData) );
}

bool Robot_Enable()
{ 
  Robot_SetControlState( CONTROL_OFFSET );
  
  for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
  {
    if( !Actuator_Enable( robot.actuatorsList[ jointIndex ] ) ) return false;
  }
  
  if( !(robot.isControlRunning) )
  {
    robot.controlThread = Thread_Start( AsyncControl, &robot, THREAD_JOINABLE );
  
    if( robot.controlThread == THREAD_INVALID_HANDLE ) return false;
  }
  
  return true;
}

bool Robot_Disable()
{
  if( robot.controlThread == THREAD_INVALID_HANDLE ) return false;
  
  robot.isControlRunning = false;
  Thread_WaitExit( robot.controlThread, 5000 );
  robot.controlThread = THREAD_INVALID_HANDLE;
  
  for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
  {
    DoFVariables stopSetpoints = { 0.0 };
    (void) Actuator_SetSetpoints( robot.actuatorsList[ jointIndex ], &stopSetpoints );
    
    Actuator_Disable( robot.actuatorsList[ jointIndex ] );
  }
  
  return true;
}

bool Robot_SetControlState( enum ControlState newState )
{
  if( newState == robot.controlState ) return false;
  
  if( newState >= CONTROL_STATES_NUMBER ) return false;
  
  robot.SetControlState( newState );
  
  for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
    Actuator_SetControlState( robot.actuatorsList[ jointIndex ], newState );
  
  robot.controlState = newState;
  
  return true;
}

const char* Robot_GetJointName( size_t jointIndex )
{
  if( jointIndex >= robot.jointsNumber ) return NULL;
  
  const char** jointNamesList = robot.GetJointNamesList();
  
  if( jointNamesList == NULL ) return NULL;
  
  return jointNamesList[ jointIndex ];
}

const char* Robot_GetAxisName( size_t axisIndex )
{
  if( axisIndex >= robot.axesNumber ) return NULL;
  
  const char** axisNamesList = robot.GetAxisNamesList();
  
  if( axisNamesList == NULL ) return NULL;
  
  return axisNamesList[ axisIndex ];
}

bool Robot_GetJointMeasures( size_t jointIndex, DoFVariables* ref_measures )
{
  if( jointIndex >= robot.jointsNumber ) return false;
  
  *ref_measures = *(robot.jointMeasuresList[ jointIndex ]);
  
  return true;
}

bool Robot_GetAxisMeasures( size_t axisIndex, DoFVariables* ref_measures )
{
  if( axisIndex >= robot.axesNumber ) return false;
  
  *ref_measures = *(robot.axisMeasuresList[ axisIndex ]);
  
  return true;
}

void Robot_SetAxisSetpoints( size_t axisIndex, DoFVariables* ref_setpoints )
{
  if( axisIndex >= robot.axesNumber ) return;
  
  *(robot.axisSetpointsList[ axisIndex ]) = *ref_setpoints;
}

size_t Robot_GetJointsNumber()
{
  return robot.jointsNumber;
}

size_t Robot_GetAxesNumber()
{
  return robot.axesNumber;
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCHRONOUS CONTROL                          /////
/////////////////////////////////////////////////////////////////////////////////

void LinearizeDoF( DoFVariables* measures, DoFVariables* setpoints, LinearSystem linearizer )
{
  double inputsList[ 3 ], outputsList[ 1 ], impedancesList[ 3 ];
  
  inputsList[ 0 ] = measures->position;
  inputsList[ 1 ] = measures->velocity;
  inputsList[ 2 ] = measures->acceleration;
  outputsList[ 0 ] = measures->force + setpoints->force;
  if( SystemLinearizer_AddSample( linearizer, inputsList, outputsList ) >= LINEARIZATION_MAX_SAMPLES )
  {
    if( SystemLinearizer_Identify( linearizer, impedancesList ) )
    {
      measures->stiffness = ( impedancesList[ 0 ] > 0.0 ) ? impedancesList[ 0 ] : 0.0;
      measures->damping = ( impedancesList[ 1 ] > 0.0 ) ? impedancesList[ 1 ] : 0.0;
      measures->inertia = ( impedancesList[ 2 ] > 0.1 ) ? impedancesList[ 2 ] : 0.1;
    }
  }
}

void LogRobotData( RobotData* robot, double execTime )
{
  Log_EnterNewLine( robot->controlLog, execTime );
    for( size_t axisIndex = 0; axisIndex < robot->axesNumber; axisIndex++ )
    {
      Log_RegisterList( robot->controlLog, sizeof(DoFVariables)/sizeof(double), (double*) robot->axisSetpointsList[ axisIndex ] );
      Log_RegisterList( robot->controlLog, sizeof(DoFVariables)/sizeof(double), (double*) robot->axisMeasuresList[ axisIndex ] );
    }
    Log_RegisterList( robot->controlLog, robot->extraInputsNumber, robot->extraInputValuesList );
    Log_RegisterList( robot->controlLog, robot->extraOutputsNumber, robot->extraOutputValuesList );
}

static void* AsyncControl( void* ref_robot )
{
  RobotData* robot = (RobotData*) ref_robot;
  
  double execTime = Time_GetExecSeconds(), elapsedTime = 0.0;
  
  robot->isControlRunning = true;
  
  DEBUG_PRINT( "starting to run control for robot %p on thread %lx", robot, Thread_GetID );
  
  while( robot->isControlRunning )
  {
    elapsedTime = Time_GetExecSeconds() - execTime;
    
    execTime = Time_GetExecSeconds();
    
    for( size_t inputIndex = 0; inputIndex < robot->extraInputsNumber; inputIndex++ )
      robot->extraInputValuesList[ inputIndex ] = Input_Update( robot->extraInputsList[ inputIndex ] );
    robot->SetExtraInputsList( robot->extraInputValuesList );
    
    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
      (void) Actuator_GetMeasures( robot->actuatorsList[ jointIndex ], robot->jointMeasuresList[ jointIndex ], elapsedTime );

    if( robot->controlState == CONTROL_OPERATION || robot->controlState == CONTROL_CALIBRATION )
    {
      for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
        LinearizeDoF( robot->jointMeasuresList[ jointIndex ], robot->jointSetpointsList[ jointIndex ], robot->jointLinearizersList[ jointIndex ] );
    }

    robot->RunControlStep( robot->jointMeasuresList, robot->axisMeasuresList, robot->jointSetpointsList, robot->axisSetpointsList, elapsedTime );

    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
      (void) Actuator_SetSetpoints( robot->actuatorsList[ jointIndex ], robot->jointSetpointsList[ jointIndex ] );

    robot->GetExtraOutputsList( robot->extraOutputValuesList );
    for( size_t outputIndex = 0; outputIndex < robot->extraOutputsNumber; outputIndex++ )
      Output_Update( robot->extraOutputsList[ outputIndex ], robot->extraOutputValuesList[ outputIndex ] );
    
    LogRobotData( robot, execTime );
    
    elapsedTime = Time_GetExecSeconds() - execTime;
    if( elapsedTime < robot->controlTimeStep ) Time_Delay( (unsigned long) ( 1000 * ( robot->controlTimeStep - elapsedTime ) ) );
    //DEBUG_PRINT( "step time for robot %p: before delay=%.5fs, after delay=%.5fs", robot, elapsedTime, Time_GetExecSeconds() - execTime );
  }
  
  return NULL;
}
