#include "control_interface.h"

IMPLEMENT_INTERFACE( CONTROL_FUNCTIONS )

// Torque Filter
const double a2 = -0.9428;
const double a3 = 0.3333;
const double b1 = 0.0976;
const double b2 = 0.1953;
const double b3 = 0.0976;

// Angular Velocity Filter
const double c2 = -1.889;
const double c3 = 0.8949;
const double d1 = 0.0015;
const double d2 = 0.0029;
const double d3 = 0.0015;

double* Run( double measuresList[ CONTROL_VARS_NUMBER ], double setpointsList[ CONTROL_VARS_NUMBER ], double deltaTime, double* ref_error )
{
  static double position, positionSetpoint, positionError, positionErrorSum, positionSetpointSum;
  static double velocity[3], velocityFiltered[3], velocitySetpoint[3];
  static double force[3], forceFiltered[3], forceError[3];
  
  static double outputsList[ CONTROL_VARS_NUMBER ];
  
  static double error;
  
  if( *ref_error == 0.0 )
  {
    /*error = ( positionSetpointSum > 0.0 ) ? positionErrorSum / positionSetpointSum : 1.0;
    if( error > 1.0 )*/ error = 1.0;
    
    positionErrorSum = 0.0;
    positionSetpointSum = 0.0;
  }

  velocity[0] = ( measuresList[ CONTROL_POSITION ] - position ) / deltaTime;
  position = measuresList[ CONTROL_POSITION ];

  velocityFiltered[0] = -c2 * velocityFiltered[1] - c3 * velocityFiltered[2] + d1 * velocity[0] + d2 * velocity[1] + d3 * velocity[2];
  
  positionSetpoint = setpointsList[ CONTROL_POSITION ];
  outputsList[ CONTROL_POSITION ] = positionSetpoint;
  positionError = position - positionSetpoint;
  positionErrorSum += deltaTime * positionError * positionError;
  positionSetpointSum += deltaTime * positionSetpoint * positionSetpoint;
  
  *ref_error = positionError / positionSetpoint;
  
  //double forceSetpoint = -setpointsList[ CONTROL_STIFFNESS ] * positionError - setpointsList[ CONTROL_DAMPING ] * velocityFiltered[0];
  double forceSetpoint = error * setpointsList[ CONTROL_FORCE ];
  outputsList[ CONTROL_FORCE ] = forceSetpoint;

  force[0] = measuresList[ CONTROL_FORCE ];
  forceFiltered[0] = -a2 * forceFiltered[1] - a3 * forceFiltered[2] + b1 * force[0] + b2 * force[1] + b3 * force[2];
  
  forceError[0] = forceSetpoint - forceFiltered[0];

  velocitySetpoint[0] += 370.0 * ( forceError[0] - forceError[1] ) + 3.5 * deltaTime * forceError[0];
  outputsList[ CONTROL_VELOCITY ] = velocitySetpoint[ 0 ];
  
  //velocitySetpoint[0] = 0.9822 * velocitySetpoint[1] + 0.01407 * velocitySetpoint[2] + 338.6 * forceError[1] - 337.4 * forceError[2]; //5ms
  
  for( int i = 2; i > 0; i-- )
  {
    forceError[ i ] = forceError[ i - 1 ];
    velocity[ i ] = velocity[ i - 1 ];
    velocityFiltered[ i ] = velocityFiltered[ i - 1 ];
    forceFiltered[ i ] = forceFiltered[ i - 1 ];
    velocitySetpoint[ i ] = velocitySetpoint[ i - 1 ];
  }

  return (double*) outputsList;
}
