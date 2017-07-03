//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
//  Original work Copyright (c) 2014 Wilian dos Santos                                  //
//  Modified work Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>   //
//                                                                                      //
//  This file is part of RobRehabSystem.                                                //
//                                                                                      //
//  RobRehabSystem is free software: you can redistribute it and/or modify              //
//  it under the terms of the GNU Lesser General Public License as published            //
//  by the Free Software Foundation, either version 3 of the License, or                //
//  (at your option) any later version.                                                 //
//                                                                                      //
//  RobRehabSystem is distributed in the hope that it will be useful,                   //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of                      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                        //
//  GNU Lesser General Public License for more details.                                 //
//                                                                                      //
//  You should have received a copy of the GNU Lesser General Public License            //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.              //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////


#include "actuator_control/interface.h"

#include <string.h>

typedef struct _ControlData
{
  double positionErrorSum, positionSetpointSum;
  double velocitySetpoint;
  double forceError[ 2 ];
  double controlError;
}
ControlData;

typedef ControlData* Controller;

DECLARE_MODULE_INTERFACE( ACTUATOR_CONTROL_INTERFACE )

ActuatorController InitController( void )
{
  void* newController = malloc( sizeof(ControlData) );
  memset( newController, 0, sizeof(ControlData) );
  
  return (Controller) newController;
}

ActuatorVariables RunControlStep( ActuatorController ref_controller, ActuatorVariables* measures, ActuatorVariables* setpoints, double timeDelta )
{
  const double K_P = 370;//1.6527; // 370 * ( F_in_max / V_out_max )
  const double K_I = 3.5;//0.0156; // 3.5 * ( F_in_max / V_out_max )
  
  Controller controller = (ActuatorController) ref_controller;
  
  ActuatorVariables outputs = { .position = setpoints->position };
  
  double positionError = measures->position - setpoints->position;
  controller->positionErrorSum += timeDelta * positionError * positionError;
  controller->positionSetpointSum += timeDelta * setpoints->position * setpoints->position;
  
  //controller->controlError = ( controller->positionSetpointSum > 0.0 ) ? controller->positionErrorSum / controller->positionSetpointSum : 1.0;
  /*if( controller->controlError > 1.0 )*/ controller->controlError = 1.0;
  
  double forceSetpoint = controller->controlError * setpoints->force;
  outputs.force = forceSetpoint;

  controller->forceError[ 0 ] = forceSetpoint - measures->force;
  
  controller->velocitySetpoint += K_P * ( controller->forceError[ 0 ] - controller->forceError[ 1 ] ) + K_I * timeDelta * controller->forceError[ 0 ];
  outputs.velocity = controller->velocitySetpoint;
  
  //velocitySetpoint[0] = 0.9822 * velocitySetpoint[1] + 0.01407 * velocitySetpoint[2] + 338.6 * forceError[1] - 337.4 * forceError[2]; //5ms
  
  controller->forceError[ 1 ] = controller->forceError[ 0 ];

  return outputs;
}

void EndController( ActuatorController controller )
{
  free( (void*) controller );
}
