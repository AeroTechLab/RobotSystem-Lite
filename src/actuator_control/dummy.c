//////////////////////////////////////////////////////////////////////////////////////
//                                                                                  //
//  Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>             //
//                                                                                  //
//  This file is part of Robot Control Library.                                     //
//                                                                                  //
//  Robot Control Library is free software: you can redistribute it and/or modify   //
//  it under the terms of the GNU Lesser General Public License as published        //
//  by the Free Software Foundation, either version 3 of the License, or            //
//  (at your option) any later version.                                             //
//                                                                                  //
//  Robot Control Library is distributed in the hope that it will be useful,        //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of                  //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                    //
//  GNU Lesser General Public License for more details.                             //
//                                                                                  //
//  You should have received a copy of the GNU Lesser General Public License        //
//  along with Robot Control Library. If not, see <http://www.gnu.org/licenses/>.   //
//                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////


#include "actuator_control/interface.h"

#include <string.h>

typedef struct _ControlData
{
  double outputsList[ CONTROL_MODES_NUMBER ];
}
ControlData;

DECLARE_MODULE_INTERFACE( ACTUATOR_CONTROL_INTERFACE );

Controller InitController( void )
{
  void* newController = malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
  
  return (Controller) newController;
}

double* RunControlStep( Controller controller, double* measuresList, double* setpointsList, double* ref_error )
{ 
  if( controller == NULL ) return NULL;
  
  ControlData* controlData = (ControlData*) controller;

  controlData->outputsList[ CONTROL_POSITION ] = setpointsList[ CONTROL_POSITION ];
  controlData->outputsList[ CONTROL_VELOCITY ] = setpointsList[ CONTROL_VELOCITY ];
  controlData->outputsList[ CONTROL_FORCE ] = setpointsList[ CONTROL_FORCE ];
  controlData->outputsList[ CONTROL_ACCELERATION ] = setpointsList[ CONTROL_ACCELERATION ];

  return (double*) controlData->outputsList;
}

void EndController( Controller controller )
{
  free( (void*) controller );
}
