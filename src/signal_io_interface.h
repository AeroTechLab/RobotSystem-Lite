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


/// @file signal_io_interface.h
/// @brief Signal aquisition/generation functions
///
/// Physical/virtual signal aquisition and generation interface to be implemented by hardware specific plugins
/// Each aquisition/generation (input/output) operation/process/thread created by ab implementation is generically defined as a Task

#ifndef SIGNAL_IO_INTERFACE_H
#define SIGNAL_IO_INTERFACE_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159    ///< Defines mathematical Pi value if standard math.h one is not available
#endif

#include "plugin_loader/loader_macros.h"

#define SIGNAL_INPUT_CHANNEL_MAX_USES 5     ///< Max allowed concurrent readers of the same signal aquisition channel

#define SIGNAL_IO_TASK_INVALID_ID -1        ///< Task identifier to be returned on task creation errors

/// Signal input/output interface declaration macro
#define SIGNAL_IO_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( int, Namespace, InitTask, const char* ) \
        INIT_FUNCTION( void, Namespace, EndTask, int ) \
        INIT_FUNCTION( void, Namespace, Reset, int ) \
        INIT_FUNCTION( bool, Namespace, HasError, int ) \
        INIT_FUNCTION( size_t, Namespace, GetMaxInputSamplesNumber, int ) \
        INIT_FUNCTION( size_t, Namespace, Read, int, unsigned int, double* ) \
        INIT_FUNCTION( bool, Namespace, AcquireInputChannel, int, unsigned int ) \
        INIT_FUNCTION( void, Namespace, ReleaseInputChannel, int, unsigned int ) \
        INIT_FUNCTION( void, Namespace, EnableOutput, int, bool ) \
        INIT_FUNCTION( bool, Namespace, IsOutputEnabled, int ) \
        INIT_FUNCTION( bool, Namespace, Write, int, unsigned int, double ) \
        INIT_FUNCTION( bool, Namespace, AcquireOutputChannel, int, unsigned int ) \
        INIT_FUNCTION( void, Namespace, ReleaseOutputChannel, int, unsigned int ) 

        
/// @class SIGNAL_IO_INTERFACE
/// @brief File/string/stream data input/output methods to be implemented by plugins
///    
/// @memberof SIGNAL_IO_INTERFACE
/// @fn                                                                             
/// @brief Creates plugin specific signal input/output task data structure
/// @param[in] taskConfig implementation specific task configuration string
/// @return generic identifier to newly created task (SIGNAL_IO_TASK_INVALID_ID on errors)
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn void EndTask( int taskID )
/// @brief Discards given signal input/output task data structure    
/// @param[in] taskID input/output task identifier
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn bool HasError( int taskID )                                                                                
/// @brief Verifies occurence of errors on given task
/// @param[in] taskID input/output task identifier 
/// @return true on detected error, false otherwise 
///   
/// @memberof SIGNAL_IO_INTERFACE        
/// @fn void Reset( int taskID )
/// @brief Resets data and errors for given task
/// @param[in] taskID input/output task identifier
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn size_t GetMaxInputSamplesNumber( int taskID )
/// @brief Gets number of samples aquired for every given task input channel on each Read() call
/// @param[in] taskID input task identifier
/// @return max read samples number (0 on errors)
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn bool AcquireInputChannel( int taskID, unsigned int channel )
/// @brief Adds new reader for specified input channel of given task
/// @param[in] taskID input task identifier
/// @param[in] channel input task channel index
/// @return true on successful channel aquisition (availability for reading), false otherwise
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn void ReleaseInputChannel( int taskID, unsigned int channel )
/// @brief Removes reader for specified input channel of given task
/// @param[in] taskID input task identifier
/// @param[in] channel input task channel index
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn size_t Read( int taskID, unsigned int channel, double* ref_value )
/// @brief Reads samples list from specified channel of given task
/// @param[in] taskID input task identifier
/// @param[in] channel input task channel index
/// @param[out] ref_value allocated buffer long enough to hold the samples number returned by GetMaxInputSamplesNumber()
/// @return number of samples read (0 on errors)
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn void EnableOutput( int taskID, bool enable )
/// @brief Sets signal output operation state for given task
/// @param[in] taskID output task identifier
/// @param[in] enable true for enabling output, false for disabling it
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn bool IsOutputEnabled( int taskID )
/// @brief Verifies output operation state of given task 
/// @param[in] taskID output task identifier
/// @return true if output is enabled, false otherwise
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn bool Write( int taskID, unsigned int channel, double value )
/// @brief Writes value to specified channel of given task
/// @param[in] taskID output task identifier
/// @param[in] channel output task channel index
/// @param[in] value value to be written
/// @return true on successful writing, false otherwise
///
/// @memberof SIGNAL_IO_INTERFACE


#endif // SIGNAL_IO_INTERFACE_H 
