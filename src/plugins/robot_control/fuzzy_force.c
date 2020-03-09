////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//  Copyright (c) 2016-2020 Leonardo Consoni <leonardojc@protonmail.com>      //
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

#include <math.h>

enum { NEGATIVE_HIGH, NEGATIVE_LOW, ZERO, POSITIVE_LOW, POSITIVE_HIGH, FUZZY_SETS_NUMBER };

#define DISCRETIZATION_INTERVAL 0.01

typedef struct _NormalDistribuitionData
{
  double medianValue;
  double variance;
}
NormalDistribuitionData;

const NormalDistribuitionData FUZZY_SETS_LIST[ FUZZY_SETS_NUMBER ] = { {-1, 0.2}, {-0.5, 0.2}, {0, 0.2}, {0.5, 0.2}, {1, 0.2} };
                                                          
const int INFERENCE_RULES[ FUZZY_SETS_NUMBER ][ FUZZY_SETS_NUMBER ] = 
{
  { POSITIVE_LOW, ZERO, NEGATIVE_LOW, NEGATIVE_LOW, NEGATIVE_HIGH },
  { POSITIVE_LOW, ZERO, ZERO, NEGATIVE_LOW, NEGATIVE_HIGH },
  { POSITIVE_HIGH, POSITIVE_LOW, ZERO, NEGATIVE_LOW, NEGATIVE_HIGH },
  { POSITIVE_HIGH, POSITIVE_LOW, ZERO, ZERO, NEGATIVE_LOW },
  { POSITIVE_HIGH, POSITIVE_LOW, POSITIVE_LOW, ZERO, NEGATIVE_LOW }
};

DEFINE_INTERFACE( CONTROL_FUNCTIONS )

double* Run( double measuresList[ CONTROL_VARS_NUMBER ], double setpointsList[ CONTROL_VARS_NUMBER ], double deltaTime, double* ref_error )
{
  static double outputsList[ CONTROL_VARS_NUMBER ];
  
  double outputSetCutsList[ FUZZY_SETS_NUMBER ] = { 0 };
  
  double positionError = ( setpointsList[ CONTROL_POSITION ] - measuresList[ CONTROL_POSITION ] ) / 0.3;
  double forceError = ( setpointsList[ CONTROL_FORCE ] - measuresList[ CONTROL_FORCE ] ) / 5;
  
  for( size_t positionErrorSetIndex = 0; positionErrorSetIndex < FUZZY_SETS_NUMBER; positionErrorSetIndex++ )
  {
    double medianPoint = FUZZY_SETS_LIST[ positionErrorSetIndex ].medianValue;
    double variance = FUZZY_SETS_LIST[ positionErrorSetIndex ].variance;
    double positionErrorInclusion = exp( -pow( positionError - medianPoint, 2 ) / ( 2 * pow( variance, 2 ) ) );
    
    for( size_t forceErrorSetIndex = 0; forceErrorSetIndex < FUZZY_SETS_NUMBER; forceErrorSetIndex++ )
    {
      medianPoint = FUZZY_SETS_LIST[ forceErrorSetIndex ].medianValue;
      variance = FUZZY_SETS_LIST[ forceErrorSetIndex ].variance;
      double forceErrorInclusion = exp( -pow( forceError - medianPoint, 2 ) / ( 2 * pow( variance, 2 ) ) );   
        
      double cutValue = ( positionErrorInclusion < forceErrorInclusion ) ? positionErrorInclusion : forceErrorInclusion;
      size_t outputSetIndex = INFERENCE_RULES[ positionErrorSetIndex ][ forceErrorSetIndex ];
      if( cutValue > outputSetCutsList[ outputSetIndex ] ) outputSetCutsList[ outputSetIndex ] = cutValue;
    }
  }

  double outputSum = 0.0;
  double outputWeightedSum = 0.0;
  for( double pointPosition = -1.0; pointPosition <= 1.0; pointPosition += DISCRETIZATION_INTERVAL )
  {
    double outputPointValue = 0.0;

    for( size_t outputSetIndex = 0; outputSetIndex < FUZZY_SETS_NUMBER; outputSetIndex++ )
    {
      double medianPoint = FUZZY_SETS_LIST[ outputSetIndex ].medianValue;
      double variance = FUZZY_SETS_LIST[ outputSetIndex ].variance;
      double outputInclusion = exp( -pow( pointPosition - medianPoint, 2 ) / ( 2 * pow( variance, 2 ) ) );   
      
      if( outputInclusion > outputSetCutsList[ outputSetIndex ] ) outputInclusion = outputSetCutsList[ outputSetIndex ];
      
      if( outputPointValue < outputInclusion ) outputPointValue = outputInclusion;
    }
    
    outputSum += outputPointValue;
    outputWeightedSum += outputPointValue * pointPosition;
  }

  outputsList[ CONTROL_VELOCITY ] = -( outputWeightedSum / outputSum ) * 600;
  
  return (double*) outputsList;
}
