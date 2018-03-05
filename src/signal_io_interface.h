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

#ifndef SIGNAL_IO_INTERFACE_H
#define SIGNAL_IO_INTERFACE_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159    ///< Defines mathematical Pi value if standard math.h one is not available
#endif

#include "plugin_loader/loader_macros.h"

#define SIGNAL_IO_DEVICE_INVALID_ID -1        ///< Device identifier to be returned on device creation errors

/// Signal input/output interface declaration macro
#define SIGNAL_IO_INTERFACE( Namespace, INIT_FUNCTION ) \
        INIT_FUNCTION( int, Namespace, InitDevice, const char* ) \
        INIT_FUNCTION( void, Namespace, EndDevice, int ) \
        INIT_FUNCTION( void, Namespace, Reset, int ) \
        INIT_FUNCTION( bool, Namespace, HasError, int ) \
        INIT_FUNCTION( size_t, Namespace, GetMaxInputSamplesNumber, int ) \
        INIT_FUNCTION( size_t, Namespace, Read, int, unsigned int, double* ) \
        INIT_FUNCTION( bool, Namespace, CheckInputChannel, int, unsigned int ) \
        INIT_FUNCTION( bool, Namespace, Write, int, unsigned int, double ) \
        INIT_FUNCTION( bool, Namespace, AcquireOutputChannel, int, unsigned int ) \
        INIT_FUNCTION( void, Namespace, ReleaseOutputChannel, int, unsigned int ) 

        
/// @class SIGNAL_IO_INTERFACE
/// @brief File/string/stream data input/output methods to be implemented by plugins
///    
/// @memberof SIGNAL_IO_INTERFACE
/// @fn int InitDevice( const char* deviceConfig )                                                                            
/// @brief Creates plugin specific signal input/output device data structure
/// @param[in] deviceConfig implementation specific device configuration string
/// @return generic identifier to newly created device (SIGNAL_IO_TASK_INVALID_ID on errors)
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn void EndDevice( int deviceID )
/// @brief Discards given signal input/output device data structure    
/// @param[in] deviceID input/output device identifier
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn bool HasError( int deviceID )                                                                                
/// @brief Verifies occurence of errors on given device
/// @param[in] deviceID input/output device identifier 
/// @return true on detected error, false otherwise 
///   
/// @memberof SIGNAL_IO_INTERFACE        
/// @fn void Reset( int deviceID )
/// @brief Resets data and errors for given device
/// @param[in] deviceID input/output device identifier
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn size_t GetMaxInputSamplesNumber( int deviceID )
/// @brief Gets number of samples aquired for every given device input channel on each Read() call
/// @param[in] deviceID input device identifier
/// @return max read samples number (0 on errors)
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn bool CheckInputChannel( int deviceID, unsigned int channel )
/// @brief Check reading availability for specified input channel of given device
/// @param[in] deviceID input device identifier
/// @param[in] channel input device channel index
/// @return true on channel availability for reading, false otherwise
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn size_t Read( int deviceID, unsigned int channel, double* ref_value )
/// @brief Reads samples list from specified channel of given device
/// @param[in] deviceID input device identifier
/// @param[in] channel input device channel index
/// @param[out] ref_value allocated buffer long enough to hold the samples number returned by GetMaxInputSamplesNumber()
/// @return number of samples read (0 on errors)
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn bool AcquireOutputChannel( int deviceID, unsigned int channel )
/// @brief Check availability and get exclusive access for specified output channel of given device
/// @param[in] deviceID output device identifier
/// @param[in] channel output device channel index
/// @return true on successful channel acquisition (availability for writing), false otherwise
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn void ReleaseOutputChannel( int deviceID, unsigned int channel )
/// @brief Give up exclusive access for specified output channel of given device
/// @param[in] deviceID output device identifier
/// @param[in] channel output device channel index
///   
/// @memberof SIGNAL_IO_INTERFACE
/// @fn bool Write( int deviceID, unsigned int channel, double value )
/// @brief Writes value to specified channel of given device
/// @param[in] deviceID output device identifier
/// @param[in] channel output device channel index
/// @param[in] value value to be written
/// @return true on successful writing, false otherwise
///
/// @memberof SIGNAL_IO_INTERFACE


#endif // SIGNAL_IO_INTERFACE_H 
