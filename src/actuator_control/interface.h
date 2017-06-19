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


/// @file actuator_control/interface.h
/// @brief Generic actuator control functions.
///
/// Common actuator control interface to be implemented by plugins. Follows definitions by control_definitions.h.

#ifndef ACTUATOR_CONTROL_INTERFACE_H
#define ACTUATOR_CONTROL_INTERFACE_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159      ///< Defines mathematical Pi value if standard math.h one is not available
#endif

#include "utils/modules.h"

#define ACTUATOR_CONTROLLER_INVALID_HANDLE NULL    ///< Reference/pointer to be returned on controller creation failure

typedef void* ActuatorController;                 ///< Generic/opaque type to reference robot/actuator controllers

/// Control used variables list indexes enumeration
typedef struct ActuatorVariables
{
  double position, velocity, force, acceleration;
}
ActuatorVariables;

/// Actuator control interface declaration macro
#define ACTUATOR_CONTROL_INTERFACE( Interface, INIT_FUNCTION ) \
        INIT_FUNCTION( ActuatorController, Interface, InitController, void ) \
        INIT_FUNCTION( ActuatorVariables, Interface, RunControlStep, ActuatorController, ActuatorVariables*, ActuatorVariables*, double ) \
        INIT_FUNCTION( void, Interface, EndController, ActuatorController )

#endif  // ACTUATOR_CONTROL_INTERFACE_H
        
/// @class ACTUATOR_CONTROL_INTERFACE
/// @brief Actuator control methods to be implemented by plugins
///           
/// @memberof ACTUATOR_CONTROL_INTERFACE
/// @fn ActuatorController InitController( void )                                                                                 
/// @brief Calls plugin specific actuator controller initialization                                                       
/// @return generic pointer/reference to the created controller  
///           
/// @memberof ACTUATOR_CONTROL_INTERFACE
/// @fn void EndController( ActuatorController controller )
/// @brief Calls plugin specific actuator controller data deallocation                              
/// @param[in] controller reference to actuator controller being terminated
///               
/// @memberof ACTUATOR_CONTROL_INTERFACE
/// @fn double* RunControlStep( ActuatorController controller, double* measuresList, double* setpointsList, double timeDelta )                                                                                  
/// @brief Calls plugin specific logic to process single control pass
/// @param[in] controller reference to actuator controller being updated
/// @param[in,out] measuresList vector of control variables representing current actuator measures                                    
/// @param[in] setpointsList vector of control variables representing desired actuator state
/// @param[in] timeDelta time (in seconds) since the last control pass was called
/// @return vector of control variables representing corresponding actions to be applied to the actuator 
///
/// @memberof ACTUATOR_CONTROL_INTERFACE
