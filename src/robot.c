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
}
JointData; 

typedef struct _AxisData         // Single robot axis internal data structure
{
  RobotVariables* measures;
  RobotVariables* setpoints;
}
AxisData;

typedef struct _RobotData
{
  DECLARE_MODULE_INTERFACE_REF( ROBOT_CONTROL_INTERFACE );
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
  
  sprintf( filePath, KEY_CONFIG "/" KEY_ROBOT "/%s", configPathName );
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
        robot.jointsList = (JointData*) calloc( robot.jointsNumber, sizeof(JointData) );
        robot.jointMeasuresList = (RobotVariables**) calloc( robot.jointsNumber, sizeof(RobotVariables*) );
        robot.jointSetpointsList = (RobotVariables**) calloc( robot.jointsNumber, sizeof(RobotVariables*) );
        DEBUG_PRINT( "found %lu joints", robot.jointsNumber );
        for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
        {
          const char* actuatorName = DataIO_GetStringValue( configuration, "", KEY_ACTUATOR "s.%lu", jointIndex );
          robot.jointsList[ jointIndex ].actuator = Actuator_Init( actuatorName );
          robot.jointMeasuresList[ jointIndex ] = robot.jointsList[ jointIndex ].measures = (RobotVariables*) malloc( sizeof(RobotVariables) );
          robot.jointSetpointsList[ jointIndex ] = robot.jointsList[ jointIndex ].setpoints = (RobotVariables*) malloc( sizeof(RobotVariables) );
        }

        robot.axesNumber = robot.GetAxesNumber();
        robot.axesList = (AxisData*) calloc( robot.axesNumber, sizeof(AxisData) );
        robot.axisMeasuresList = (RobotVariables**) calloc( robot.axesNumber, sizeof(RobotVariables*) );
        robot.axisSetpointsList = (RobotVariables**) calloc( robot.axesNumber, sizeof(RobotVariables*) );
        DEBUG_PRINT( "found %lu axes", robot.axesNumber );
        for( size_t axisIndex = 0; axisIndex < robot.axesNumber; axisIndex++ )
        {
          robot.axisMeasuresList[ axisIndex ] = robot.axesList[ axisIndex ].measures = (RobotVariables*) malloc( sizeof(RobotVariables) );
          robot.axisSetpointsList[ axisIndex ] = robot.axesList[ axisIndex ].setpoints = (RobotVariables*) malloc( sizeof(RobotVariables) );
        }
        
        DEBUG_PRINT( "robot %s initialized", configPathName );
      }
    }
    
    DataIO_UnloadData( configuration );

    if( !loadSuccess ) Robot_End();
    
    // testing hack
    Robot_Enable();
    Robot_SetControlState( ROBOT_OPERATION );
  }
  
  return loadSuccess;
}

void Robot_End()
{
  Robot_Disable();
  
  robot.EndController();
  
  for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
  {
    Actuator_End( robot.jointsList[ jointIndex ].actuator );
    free( robot.jointsList[ jointIndex ].measures );
    free( robot.jointsList[ jointIndex ].setpoints );
  }
  free( robot.jointsList );
  
  for( size_t axisIndex = 0; axisIndex < robot.axesNumber; axisIndex++ )
  {
    free( robot.axesList[ axisIndex ].measures );
    free( robot.axesList[ axisIndex ].setpoints );
  }
  free( robot.axesList );
  
  free( robot.jointMeasuresList );
  free( robot.jointSetpointsList );
  free( robot.axisMeasuresList );
  free( robot.axisSetpointsList );
    
  memset( &robot, 0, sizeof(RobotData) );
}

bool Robot_Enable()
{ 
  Robot_SetControlState( ROBOT_OFFSET );
  
  for( size_t jointIndex = 0; jointIndex < robot.jointsNumber; jointIndex++ )
  {
    if( !Actuator_Enable( robot.jointsList[ jointIndex ].actuator ) ) return false;
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
    (void) Actuator_SetSetpoints( robot.jointsList[ jointIndex ].actuator, &stopSetpoints );
    
    Actuator_Disable( robot.jointsList[ jointIndex ].actuator );
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
    Actuator_SetControlState( robot.jointsList[ jointIndex ].actuator, actuatorState );
  
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
  
  *ref_measures = *(robot.jointsList[ jointIndex ].measures);
  
  return true;
}

bool Robot_GetAxisMeasures( size_t axisIndex, RobotVariables* ref_measures )
{
  if( axisIndex >= robot.axesNumber ) return false;
  
  *ref_measures = *(robot.axesList[ axisIndex ].measures);
  
  return true;
}

void Robot_SetAxisSetpoints( size_t axisIndex, RobotVariables* ref_setpoints )
{
  if( axisIndex >= robot.axesNumber ) return;
  
  *(robot.axesList[ axisIndex ].setpoints) = *ref_setpoints;
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
    
    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
      (void) Actuator_GetMeasures( robot->jointsList[ jointIndex ].actuator, (ActuatorVariables*) robot->jointMeasuresList[ jointIndex ], elapsedTime );
    
    robot->RunControlStep( robot->jointMeasuresList, robot->axisMeasuresList, robot->jointSetpointsList, robot->axisSetpointsList, elapsedTime );
    
    //DEBUG_PRINT( "s1: %.5f, s2: %.5f", robot->jointSetpointsList[ 0 ]->position, robot->jointSetpointsList[ 1 ]->position );

    for( size_t jointIndex = 0; jointIndex < robot->jointsNumber; jointIndex++ )
    {
      if( Actuator_HasError( robot->jointsList[ jointIndex ].actuator ) ) Actuator_Reset( robot->jointsList[ jointIndex ].actuator );

      (void) Actuator_SetSetpoints( robot->jointsList[ jointIndex ].actuator, (ActuatorVariables*) robot->jointSetpointsList[ jointIndex ] );
    }

    elapsedTime = Time_GetExecSeconds() - execTime;
    //DEBUG_PRINT( "step time for robot %p (before delay): %.5f s", robot, elapsedTime );
    if( elapsedTime < robot->controlTimeStep ) Time_Delay( (unsigned long) ( 1000 * ( robot->controlTimeStep - elapsedTime ) ) );
    //DEBUG_PRINT( "step time for robot %p (before delay): %.5f s", robot, Time_GetExecSeconds() - execTime );
  }
  
  return NULL;
}
