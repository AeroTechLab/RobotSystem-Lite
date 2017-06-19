////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>       //
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


#include "actuator_control/interface.h"

#include <string.h>

DECLARE_MODULE_INTERFACE( ACTUATOR_CONTROL_INTERFACE )

ActuatorController InitController( void )
{ 
  return (ActuatorController) NULL;
}

ActuatorVariables RunControlStep( ActuatorController controller, ActuatorVariables* measures, ActuatorVariables* setpoints, double timeDelta )
{ 
  ActuatorVariables outputs;

  outputs.position = setpoints->position;
  outputs.velocity = setpoints->velocity;
  outputs.force = setpoints->force;
  outputs.acceleration = setpoints->acceleration;

  return outputs;
}

void EndController( ActuatorController controller )
{
  
}
