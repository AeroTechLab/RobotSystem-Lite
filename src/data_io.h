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


/// @file data_io.h
/// @brief Data read/write functions.
///
/// Common data file/string parsing/saving interface to be implemented by plugins.

#ifndef DATA_IO_H
#define DATA_IO_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>

#define DATA_IO_MAX_FILE_PATH_LENGTH 256    ///< Maximum length of data file path (from base directory) string
#define DATA_IO_MAX_KEY_PATH_LENGTH 256     ///< Maximum length of value key/index string inside data structure
#define DATA_IO_MAX_VALUE_LENGTH 128        ///< Maximum length of value string inside data structure

typedef void* DataHandle;                   ///< Opaque reference to internal data structure  
        
                                                                             
/// @brief Creates plugin specific empty data structure object 
/// @return reference/pointer to newly created internal data structure (NULL on errors)
DataHandle DataIO_CreateEmptyData( void );

/// @brief Loads all given storage to fill plugin specific data structure
/// @param[in] filePath path (from base directory) to data file
/// @return reference/pointer to created and filled data structure (NULL on errors)
DataHandle DataIO_LoadFileData( const char* filePath );

/// @brief Overwrites default root path from which configuration files will be searched                              
/// @param[in] basePath path (absolute or relative) to desired root data file path
void DataIO_SetBaseFilePath( const char* basePath );
                                                                           
/// @brief Parses given string to fill plugin specific data structure
/// @param[in] dataString string containing data to be parsed
/// @return reference/pointer to created and filled data structure (NULL on errors)
DataHandle DataIO_LoadStringData( const char* dataString );

/// @brief Deallocates and destroys given data structure
/// @param[in] data reference to internal data structure
void DataIO_UnloadData( DataHandle data );

/// @brief Get given data structure content in serialized string form
/// @param[in] data reference to internal data structure to be serialized
/// @return allocated pointer to serialized data string (needs to be manually deallocated)
char* DataIO_GetDataString( DataHandle data );

/// @brief Get reference to inner data level from given data strucuture
/// @param[in] data reference to internal data structure where the value will be searched
/// @param[in] pathFormat format string (like in printf) to value path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched value path from pathFormat (like in printf)
/// @return reference/pointer to internal data structure (NULL on errors)
DataHandle DataIO_GetSubData( DataHandle data, const char* pathFormat, ... );

/// @brief Get specified numeric value (floating point format) from given data strucuture
/// @param[in] data reference to internal data structure where the value will be searched
/// @param[in] defaultValue value to be returned if specified field is not found
/// @param[in] pathFormat format string (like in printf) to value path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched value path from pathFormat (like in printf)
/// @return numeric value (floating point format) found or the default one
double DataIO_GetNumericValue( DataHandle data, const double defaultValue, const char* pathFormat, ... );

/// @brief Get specified string value from given data strucuture
/// @param[in] data reference to internal data structure where the value will be searched
/// @param[in] defaultValue value to be returned if specified field is not found
/// @param[in] pathFormat format string (like in printf) to value path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched value path from pathFormat (like in printf)
/// @return string value found or the default one
const char* DataIO_GetStringValue( DataHandle data, const char* defaultValue, const char* pathFormat, ... );

/// @brief Get specified boolean value from given data strucuture
/// @param[in] data reference to internal data structure where the value will be searched
/// @param[in] defaultValue value to be returned if specified field is not found
/// @param[in] pathFormat format string (like in printf) to value path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched value path from pathFormat (like in printf)
/// @return boolean value found or the default one
bool DataIO_GetBooleanValue( DataHandle data, const bool defaultValue, const char* pathFormat, ... );

/// @brief Get number of elements for specified list from given data strucuture
/// @param[in] data reference to internal data structure where the list will be searched
/// @param[in] pathFormat format string (like in printf) to list path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched list path from pathFormat (like in printf)
/// @return number of elements of the list (or 0 if list is not found)
size_t DataIO_GetListSize( DataHandle data, const char* pathFormat, ... );

/// @brief Insert list on specified field of given data strucuture
/// @param[in] data reference to internal data structure where the list will be placed
/// @param[in] key string identifier of the field where the list will be placed (NULL for appending to list)
/// @return reference/pointer to newly created internal data structure (NULL on errors)
DataHandle DataIO_AddList( DataHandle data, const char* key );

/// @brief Insert nesting level on specified field of given data strucuture
/// @param[in] data reference to internal data structure where the nesting level will be added
/// @param[in] key string identifier of the field where the nesting level will be added (NULL for appending to list)
/// @return reference/pointer to newly created internal data structure (NULL on errors)
DataHandle DataIO_AddLevel( DataHandle data, const char* key );

/// @brief Set numeric value (floating point format) for specified field of given data strucuture
/// @param[in] data reference to internal data structure where the value will be placed/updated
/// @param[in] key string identifier of the field where the value will be placed/updated (NULL for appending to list)
/// @param[in] value numeric value to be inserted/updated on given data structure field
/// @return true if value is inserted successfully, false otherwise
bool DataIO_SetNumericValue( DataHandle data, const char* key, const double value );

/// @brief Set string value (floating point format) for specified field of given data strucuture
/// @param[in] data reference to internal data structure where the value will be placed/updated
/// @param[in] key string identifier of the field where the value will be placed/updated (NULL for appending to list)
/// @param[in] value string value to be inserted/updated on given data structure field
/// @return true if value is inserted successfully, false otherwise
bool DataIO_SetStringValue( DataHandle data, const char* key, const char* value );

/// @brief Set boolean value (floating point format) for specified field of given data strucuture
/// @param[in] data reference to internal data structure where the value will be placed/updated
/// @param[in] key string identifier of the field where the value will be placed/updated (NULL for appending to list)
/// @param[in] value boolean value to be inserted/updated on given data structure field
/// @return true if value is inserted successfully, false otherwise
bool DataIO_SetBooleanValue( DataHandle data, const char* key, const bool value );

/// @brief Verify if specified value field/key is present inside given data strucuture
/// @param[in] data reference to internal data structure where the key will be searched
/// @param[in] pathFormat format string (like in printf) to key path inside the data structure (key or index fields separated by ".")
/// @param[in] ... list of string keys or numeric indexes to build searched path from pathFormat (like in printf)
/// @return true if key is found, false otherwise
bool DataIO_HasKey( DataHandle data, const char* pathFormat, ... );


#endif // DATA_IO_H
