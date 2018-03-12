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


#include "robot.h"

#include "config_keys.h"

#include "actuator.h"

#include "data_io/interface/data_io.h"
#include "threads/threads.h"
#include "timing/timing.h"
#include "debug/data_logging.h"

#include <stdlib.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////////////////
/////                            CONTROL DEVICE                             /////
/////////////////////////////////////////////////////////////////////////////////

typedef struct _JointData       // Single robot joint internal data structure
{
  Actuator actuator;
  RobotVariables* measures;
  RobotVariables* setpoints;
  bool hasChanged;
}
JointData; 

typedef struct _AxisData         // Single robot axis internal data structure
{
  RobotVariables* measures;
  RobotVariables* setpoints;
  bool hasChanged;
}
AxisData;

struct _RobotData
{
  DECLARE_MODULE_INTERFACE_REF( ROBOT_CONTROL_INTERFACE );
  RobotController controller;
  Thread controlThread;
  volatile bool isControlRunning;
  enum RobotState controlState;
  double controlTimeStep;
  JointData* jointsList;
  RobotVariables** jointMeasuresList;
  RobotVariables** jointSetpointsList;
  size_t jointsNumber;
  AxisData* axesList;
  RobotVariables** axisMeasuresList;
  RobotVariables** axisSetpointsList;
  size_t axesNumber;
};


const double CONTROL_PASS_INTERVAL = 0.005;

static void* AsyncControl( void* );

Robot Robot_Init( const char* configPathName )
{
  static char filePath[ DATA_IO_MAX_PATH_LENGTH ];

  DEBUG_PRINT( "trying to create robot %s", configPathName );
  
  Robot newRobot = NULL;
  
  sprintf( filePath, CONFIG "/" ROBOT "/%s", configPathName );
  DataHandle configuration = DataIO_LoadStorageData( filePath );
  if( configuration != NULL )
  {
    newRobot = (Robot) malloc( sizeof(RobotData) );
    memset( newRobot, 0, sizeof(RobotData) );
  
    bool loadSuccess = false;
    sprintf( filePath, MODULES "/" ROBOT_CONTROL "/%s", DataIO_GetStringValue( configuration, "", CONTROLLER "." TYPE ) );
    LOAD_MODULE_IMPLEMENTATION( ROBOT_CONTROL_INTERFACE, filePath, newRobot, &loadSuccess );
    if( loadSuccess )
    {
      const char* controllerConfigString = DataIO_GetStringValue( configuration, "", CONTROLLER "." CONFIG );
      DEBUG_PRINT( "loading controller config %s", controllerConfigString ); 
      newRobot->controller = newRobot->InitController( controllerConfigString );
      DEBUG_PRINT( "loaded controller handle %p", newRobot->controller );
      
      newRobot->controlTimeStep = DataIO_GetNumericValue( configuration, CONTROL_PASS_INTERVAL, CONTROLLER ".time_step" );   
      
      newRobot->jointsNumber = newRobot->GetJointsNumber( newRobot->controller );
      newRobot->jointsList = (JointData*) calloc( newRobot->jointsNumber, sizeof(JointData) );
      newRobot->jointMeasuresList = (RobotVariables**) calloc( newRobot->jointsNumber, sizeof(RobotVariables*) );
      newRobot->jointSetpointsList = (RobotVariables**) calloc( newRobot->jointsNumber, sizeof(RobotVariables*) );
      for( size_t jointIndex = 0; jointIndex < newRobot->jointsNumber; jointIndex++ )
      {
        DataHandle actuatorConfiguration = DataIO_GetSubData( configuration, ACTUATOR "s.%lu", jointIndex );
        newRobot->jointsList[ jointIndex ].actuator = Actuator_Init( actuatorConfiguration );
        newRobot->jointMeasuresList[ jointIndex ] = newRobot->jointsList[ jointIndex ].measures = (RobotVariables*) malloc( sizeof(RobotVariables) );
        newRobot->jointSetpointsList[ jointIndex ] = newRobot->jointsList[ jointIndex ].setpoints = (RobotVariables*) malloc( sizeof(RobotVariables) );
      }

      newRobot->axesNumber = newRobot->GetAxesNumber( newRobot->controller );
      newRobot->axesList = (AxisData*) calloc( newRobot->axesNumber, sizeof(AxisData) );
      newRobot->axisMeasuresList = (RobotVariables**) calloc( newRobot->axesNumber, sizeof(RobotVariables*) );
      newRobot->axisSetpointsList = (RobotVariables**) calloc( newRobot->axesNumber, sizeof(RobotVariables*) );
      for( size_t axisIndex = 0; axisIndex < newRobot->axesNumber; axisIndex++ )
      {
        newRobot->axisMeasuresList[ axisIndex ] = newRobot->axesList[ axisIndex ].measures = (RobotVariables*) malloc( sizeof(RobotVariables) );
        newRobot->axisSetpointsList[ axisIndex ] = newRobot->axesList[ axisIndex ].setpoints = (RobotVariables*) malloc( sizeof(RobotVariables) );
      }
    }
    
    DataIO_UnloadData( configuration );

    if( !loadSuccess )
    {
      Robot_End( newRobot );
      return NULL;
    }
    
    Log_PrintString( NULL, "robot %s created (handle: %p)", configPathName, newRobot );
  }
  
  return newRobot;
}

void Robot_End( Robot robot )
{
  if( robot == NULL ) return;
  
  Log_PrintString( NULL, "ending robot robot %p", robot );
  
  Robot_Disable( robot );
  
  robot->EndController( robot->controller );
  
  for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
  {
    Actuator_End( robot->jointsList[ jointIndex ].actuator );
    free( robot->jointsList[ jointIndex ].measures );
    free( robot->jointsList[ jointIndex ].setpoints );
  }
  free( robot->jointsList );
  
  for( size_t axisIndex = 0; axisIndex < robot->axesNumber; axisIndex++ )
  {
    free( robot->axesList[ axisIndex ].measures );
    free( robot->axesList[ axisIndex ].setpoints );
  }
  free( robot->axesList );
  
  free( robot->jointMeasuresList );
  free( robot->jointSetpointsList );
  free( robot->axisMeasuresList );
  free( robot->axisSetpointsList );
    
  free( robot );

  Log_PrintString( NULL, "robot robot %p discarded", robot );
}

bool Robot_Enable( Robot robot )
{ 
  if( robot == NULL ) return false;
  
  Robot_SetControlState( robot, ROBOT_OFFSET );
  
  for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
  {
    if( !Actuator_Enable( robot->jointsList[ jointIndex ].actuator ) ) return false;
  }
  
  if( !(robot->isControlRunning) )
  {
    robot->controlThread = Thread_Start( AsyncControl, robot, THREAD_JOINABLE );
  
    if( robot->controlThread == THREAD_INVALID_HANDLE ) return false;
  }
  
  return true;
}

bool Robot_Disable( Robot robot )
{
  if( robot == NULL ) return false;
  
  if( robot->controlThread == THREAD_INVALID_HANDLE ) return false;
  
  robot->isControlRunning = false;
  Thread_WaitExit( robot->controlThread, 5000 );
  robot->controlThread = THREAD_INVALID_HANDLE;
  
  for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
  {
    ActuatorVariables stopSetpoints = { 0.0 };
    (void) Actuator_SetSetpoints( robot->jointsList[ jointIndex ].actuator, &stopSetpoints );
    
    Actuator_Disable( robot->jointsList[ jointIndex ].actuator );
  }
  
  return true;
}

bool Robot_SetControlState( Robot robot, enum RobotState newState )
{
  if( robot == NULL ) return false;
  
  if( newState == robot->controlState ) return false;
  
  if( newState >= ROBOT_STATES_NUMBER ) return false;
  
  robot->SetControlState( robot->controller, newState );
  
  enum ActuatorState actuatorState = ACTUATOR_OPERATION;
  if( newState == ROBOT_OFFSET ) actuatorState = ACTUATOR_OFFSET;
  else if( newState == ROBOT_CALIBRATION ) actuatorState = ACTUATOR_CALIBRATION;
  
  for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
    Actuator_SetControlState( robot->jointsList[ jointIndex ].actuator, actuatorState );
  
  robot->controlState = newState;
  
  return true;
}

const char* Robot_GetJointName( Robot robot, size_t jointIndex )
{
  if( robot == NULL ) return NULL;
  
  if( jointIndex >= robot->jointsNumber ) return NULL;
  
  const char** jointNamesList = robot->GetJointNamesList( robot->controller );
  
  if( jointNamesList == NULL ) return NULL;
  
  return jointNamesList[ jointIndex ];
}

const char* Robot_GetAxisName( Robot robot, size_t axisIndex )
{
  if( robot == NULL ) return NULL;
  
  if( axisIndex >= robot->axesNumber ) return NULL;
  
  const char** axisNamesList = robot->GetAxisNamesList( robot->controller );
  
  if( axisNamesList == NULL ) return NULL;
  
  return axisNamesList[ axisIndex ];
}

bool Robot_GetJointMeasures( Robot robot, size_t jointIndex, RobotVariables* ref_measures )
{
  if( robot == NULL ) return false;
  
  if( jointIndex >= robot->jointsNumber ) return false;
  
  bool measuresChanged = robot->jointsList[ jointIndex ].hasChanged;
  robot->jointsList[ jointIndex ].hasChanged = false;
  
  *ref_measures = *(robot->jointsList[ jointIndex ].measures);
  
  return measuresChanged;
}

bool Robot_GetAxisMeasures( Robot robot, size_t axisIndex, RobotVariables* ref_measures )
{
  if( robot == NULL ) return false;
  
  if( axisIndex >= robot->axesNumber ) return false;
  
  bool measuresChanged = robot->axesList[ axisIndex ].hasChanged;
  robot->axesList[ axisIndex ].hasChanged = false;
  
  *ref_measures = *(robot->axesList[ axisIndex ].measures);
  
  return measuresChanged;
}

void Robot_SetAxisSetpoints( Robot robot, size_t axisIndex, RobotVariables* ref_setpoints )
{
  if( robot == NULL ) return;
  
  if( axisIndex >= robot->axesNumber ) return;
  
  *(robot->axesList[ axisIndex ].setpoints) = *ref_setpoints;
}

size_t Robot_GetJointsNumber( Robot robot )
{
  if( robot == NULL ) return 0;
  
  return robot->jointsNumber;
}

size_t Robot_GetAxesNumber( Robot robot )
{
  if( robot == NULL ) return 0;
  
  return robot->axesNumber;
}

/////////////////////////////////////////////////////////////////////////////////
/////                         ASYNCHRONOUS CONTROL                          /////
/////////////////////////////////////////////////////////////////////////////////

static void* AsyncControl( void* ref_robot )
{
  Robot robot = (Robot) ref_robot;
  
  double execTime = Time_GetExecSeconds(), elapsedTime = 0.0;
  
  robot->isControlRunning = true;
  
  //DEBUG_PRINT( "starting to run control for robot %p on thread %lx", robot, THREAD_ID );
  
  while( robot->isControlRunning )
  {
    elapsedTime = Time_GetExecSeconds() - execTime;
    //DEBUG_PRINT( "step time for robot %p (after delay): %.5f s", robot, elapsedTime );
    
    execTime = Time_GetExecSeconds();
    
    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
      (void) Actuator_GetMeasures( robot->jointsList[ jointIndex ].actuator, (ActuatorVariables*) robot->jointMeasuresList[ jointIndex ], elapsedTime );
    
    robot->RunControlStep( robot->controller, robot->jointMeasuresList, robot->axisMeasuresList, robot->jointSetpointsList, robot->axisSetpointsList, elapsedTime );
    
    //DEBUG_PRINT( "s1: %.5f, s2: %.5f", robot->jointSetpointsList[ 0 ]->position, robot->jointSetpointsList[ 1 ]->position );
    
    const bool* axesChangedList = robot->GetAxesChangedList( robot->controller );
    for( size_t axisIndex = 0; axisIndex < robot->axesNumber; axisIndex++ )
      if( axesChangedList[ axisIndex ] ) robot->axesList[ axisIndex ].hasChanged = true;
    
    const bool* jointsChangedList = robot->GetJointsChangedList( robot->controller );
    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
    {
      if( jointsChangedList[ jointIndex ] ) robot->jointsList[ jointIndex ].hasChanged = true;
      
      if( Actuator_HasError( robot->jointsList[ jointIndex ].actuator ) ) Actuator_Reset( robot->jointsList[ jointIndex ].actuator );
      
      (void) Actuator_SetSetpoints( robot->jointsList[ jointIndex ].actuator, (ActuatorVariables*) robot->jointSetpointsList[ jointIndex ] );
    }
    
    elapsedTime = Time_GetExecSeconds() - execTime;
    //DEBUG_PRINT( "step time for robot %p (before delay): %.5f s", robot, elapsedTime );
    if( elapsedTime < robot->controlTimeStep ) Time_Delay( (unsigned long) ( 1000 * ( robot->controlTimeStep - elapsedTime ) ) );
  }
  
  return NULL;
}
