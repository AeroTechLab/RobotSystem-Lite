////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2020 Leonardo Consoni <leonardojc@protonmail.com>      //
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


#include "signal_io/signal_io.h"

#include <stdlib.h>

DECLARE_MODULE_INTERFACE( SIGNAL_IO_INTERFACE );

long int InitDevice( const char* taskConfig )
{
  return 0;
}

void EndDevice( long int taskID )
{
  return;
}

size_t GetMaxInputSamplesNumber( long int taskID )
{
  return 1;
}

size_t Read( long int taskID, unsigned int channel, double* ref_value )
{
  *ref_value = ( rand() % 1001 ) / 1000.0 - 0.5;

  return 1;
}

bool HasError( long int taskID )
{
  return false;
}

void Reset( long int taskID )
{
  return;
}

bool CheckInputChannel( long int taskID, unsigned int channel )
{
  return true;
}

bool Write( long int taskID, unsigned int channel, double value )
{
  return true;
}

bool AcquireOutputChannel( long int taskID, unsigned int channel )
{
  return true;
}

void ReleaseOutputChannel( long int taskID, unsigned int channel )
{
  return;
}
