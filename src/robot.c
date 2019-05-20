////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2019 Leonardo Consoni <consoni_2519@hotmail.com>       //
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
  enum RobotState controlState;
  double controlTimeStep;
  Actuator* actuatorsList;
  RobotVariables** jointMeasuresList;
  RobotVariables** jointSetpointsList;
  size_t jointsNumber;
  RobotVariables** axisMeasuresList;
  RobotVariables** axisSetpointsList;
  size_t axesNumber;
  Input* extraInputsList;
  double* extraInputValuesList;
  size_t extraInputsNumber;
  Output* extraOutputsList;
  double* extraOutputValuesList;
  size_t extraOutputsNumber;
} 
RobotData;

static RobotData robot;


const double CONTROL_PASS_DEFAULT_INTERVAL = 0.005;

static void* AsyncControl( void* );

bool Robot_Init( const char* configPathName )
{
  char filePath[ DATA_IO_MAX_PATH_LENGTH ];

  DEBUG_PRINT( "trying to initialize robot %s", configPathName );
  
  bool loadSuccess = false;
  
  sprintf( filePath, KEY_CONFIG "/" KEY_ROBOTS "/%s", configPathName );
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
        robot.jointMeasuresList = (RobotVariables**) calloc( robot.jointsNumber, sizeof(RobotVariables*) );
        robot.jointSetpointsList = (RobotVariables**) calloc( robot.jointsNumber, sizeof(RobotVariables*) );
        DEBUG_PRINT( "found %lu joints", robot.jointsNumber );
        for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
        {
          const char* actuatorName = DataIO_GetStringValue( configuration, "", KEY_ACTUATORS ".%lu", jointIndex );
          robot.actuatorsList[ jointIndex ] = Actuator_Init( actuatorName );
          robot.jointMeasuresList[ jointIndex ] = (RobotVariables*) malloc( sizeof(RobotVariables) );
          robot.jointSetpointsList[ jointIndex ] = (RobotVariables*) malloc( sizeof(RobotVariables) );
        }

        robot.axesNumber = robot.GetAxesNumber();
        robot.axisMeasuresList = (RobotVariables**) calloc( robot.axesNumber, sizeof(RobotVariables*) );
        robot.axisSetpointsList = (RobotVariables**) calloc( robot.axesNumber, sizeof(RobotVariables*) );
        DEBUG_PRINT( "found %lu axes", robot.axesNumber );
        for( size_t axisIndex = 0; axisIndex < robot.axesNumber; axisIndex++ )
        {
          robot.axisMeasuresList[ axisIndex ] = (RobotVariables*) malloc( sizeof(RobotVariables) );
          robot.axisSetpointsList[ axisIndex ] = (RobotVariables*) malloc( sizeof(RobotVariables) );
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
        
        DEBUG_PRINT( "robot %s initialized", configPathName );
      }
    }
    
    DataIO_UnloadData( configuration );

    if( !loadSuccess ) Robot_End();
    
    // testing hack
    //Robot_Enable();
    //Robot_SetControlState( ROBOT_OPERATION );
  }
  
  return loadSuccess;
}

void Robot_End()
{
  Robot_Disable();
  
  robot.EndController();
  
  for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
  {
    Actuator_End( robot.actuatorsList[ jointIndex ] );
    free( robot.jointMeasuresList[ jointIndex ] );
    free( robot.jointSetpointsList[ jointIndex ] );
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
  
  memset( &robot, 0, sizeof(RobotData) );
}

bool Robot_Enable()
{ 
  Robot_SetControlState( ROBOT_OFFSET );
  
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
    ActuatorVariables stopSetpoints = { 0.0 };
    (void) Actuator_SetSetpoints( robot.actuatorsList[ jointIndex ], &stopSetpoints );
    
    Actuator_Disable( robot.actuatorsList[ jointIndex ] );
  }
  
  return true;
}

bool Robot_SetControlState( enum RobotState newState )
{
  if( newState == robot.controlState ) return false;
  
  if( newState >= ROBOT_STATES_NUMBER ) return false;
  
  robot.SetControlState( newState );
  
  enum ActuatorState actuatorState = ACTUATOR_OPERATION;
  if( newState == ROBOT_OFFSET ) actuatorState = ACTUATOR_OFFSET;
  else if( newState == ROBOT_CALIBRATION ) actuatorState = ACTUATOR_CALIBRATION;
  
  for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
    Actuator_SetControlState( robot.actuatorsList[ jointIndex ], actuatorState );
  
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

bool Robot_GetJointMeasures( size_t jointIndex, RobotVariables* ref_measures )
{
  if( jointIndex >= robot.jointsNumber ) return false;
  
  *ref_measures = *(robot.jointMeasuresList[ jointIndex ]);
  
  return true;
}

bool Robot_GetAxisMeasures( size_t axisIndex, RobotVariables* ref_measures )
{
  if( axisIndex >= robot.axesNumber ) return false;
  
  *ref_measures = *(robot.axisMeasuresList[ axisIndex ]);
  
  return true;
}

void Robot_SetAxisSetpoints( size_t axisIndex, RobotVariables* ref_setpoints )
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

static void* AsyncControl( void* ref_robot )
{
  RobotData* robot = (RobotData*) ref_robot;
  
  double execTime = Time_GetExecSeconds(), elapsedTime = 0.0;
  
  robot->isControlRunning = true;
  
  DEBUG_PRINT( "starting to run control for robot %p on thread %lx", robot, Thread_GetID );
  
  while( robot->isControlRunning )
  {
    elapsedTime = Time_GetExecSeconds() - execTime;
    //DEBUG_PRINT( "step time for robot %p (after delay): %.5f s", robot, elapsedTime );
    
    execTime = Time_GetExecSeconds();
    
    for( size_t inputIndex = 0; inputIndex < robot->extraInputsNumber; inputIndex++ )
      robot->extraInputValuesList[ inputIndex ] = Input_Update( robot->extraInputsList[ inputIndex ] );
    robot->SetExtraInputsList( robot->extraInputValuesList );
    
    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
      (void) Actuator_GetMeasures( robot->actuatorsList[ jointIndex ], (ActuatorVariables*) robot->jointMeasuresList[ jointIndex ], elapsedTime );
    
    robot->RunControlStep( robot->jointMeasuresList, robot->axisMeasuresList, robot->jointSetpointsList, robot->axisSetpointsList, elapsedTime );
    
    //DEBUG_PRINT( "s1: %.5f, s2: %.5f", robot->jointSetpointsList[ 0 ]->position, robot->jointSetpointsList[ 1 ]->position );

    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
      (void) Actuator_SetSetpoints( robot->actuatorsList[ jointIndex ], (ActuatorVariables*) robot->jointSetpointsList[ jointIndex ] );

    robot->GetExtraOutputsList( robot->extraOutputValuesList );
    for( size_t outputIndex = 0; outputIndex < robot->extraOutputsNumber; outputIndex++ )
      Output_Update( robot->extraOutputsList[ outputIndex ], robot->extraOutputValuesList[ outputIndex ] );
    
    elapsedTime = Time_GetExecSeconds() - execTime;
    //DEBUG_PRINT( "step time for robot %p (before delay): %.5f s", robot, elapsedTime );
    if( elapsedTime < robot->controlTimeStep ) Time_Delay( (unsigned long) ( 1000 * ( robot->controlTimeStep - elapsedTime ) ) );
    //DEBUG_PRINT( "step time for robot %p (before delay): %.5f s", robot, Time_GetExecSeconds() - execTime );
  }
  
  return NULL;
}
