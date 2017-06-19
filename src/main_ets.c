/*******************************************************************************
* This example uses the Network Variable library to communicate between a
* DLL running on a real-time target and an executable running on a host machine.
* The real-time DLL generates and publishes sine wave data based on the 
* amplitude and frequency specified by the host executable. When the host 
* executable exits it signals the real-time DLL to exit as well.
*
* This example uses several network variables. Two network variables communicate 
* the amplitude and frequency information from the host executable to the real-
* time DLL. Another network variable publishes the sine wave data from the real-
* time DLL to the host executable. One more network variable indicates that the 
* host executable is exiting so the real-time DLL can exit as well.
*
* The real-time DLL uses multiple threads. The main thread publishes data while
* Network Variable Library threads run callbacks when network variable values
* are updated. The real-time DLL uses thread safe variables to pass data between
* the network variable callbacks and the main thread.
*
* NOTE: This example requires the LabWindows/CVI Real-Time Module.
*
* This example consists of two projects:
*
* NetworkVariableHost.prj - This project builds an executable with a user 
* interface. This executable communicates with the real-time DLL over TCP via 
* the Network Variable Library.
*
* NetworkVariableRT.prj - This project builds a DLL that will run on a real-time
* target. This DLL interacts with the host executable over TCP via the Network
* Variable Library. Download and run the DLL on a real-time target. See the 
* LabWindows/CVI documentation for detailed instructions.
*
* Run the real-time DLL project first and then run the host executable. When you
* exit the host executable the real-time DLL project will also exit.
*******************************************************************************/

/* Include files */

#include <utility.h>
#include <rtutil.h>
#include <analysis.h>
#include <cvinetv.h>
#include <cvirte.h>
#include <ansi_c.h>

#include "system.h" 


/* Program entry-point */
void CVIFUNC_C RTmain( void )
{
  if( InitCVIRTE( 0, 0, 0 ) == 0 )
    return;

  //int status;
  //int systemStarted = 0;

  //while( !RTIsShuttingDown() && !systemStarted )
  //{
  //	status = CNVProcessIsRunning( "Shared", &systemStarted );
  //	SleepUS( 10000 );
  //}
  
  //fprintf( stderr, "Network variables ready\n" );
  
  SetDir( "C:\\ni-rt" );
  
  const int argc = 4;
  //const char* argv[] = { "", "config", "JSON", "shared_robots_config" };
  const char* argv[] = { "", "config", "JSON", "dual_motors_config" };
  //const char* argv[] = { "", "config", "JSON", "osim_robot_config" };
  //const char* argv[] = { "", "config", "JSON", "virtual_robots_config" };
  if( System_Init( argc, argv ) )
  {
  	while( !RTIsShuttingDown() ) // Check for program termination conditions
  	{
      System_Update();
    
      SleepUntilNextMultipleUS( 1000 * UPDATE_INTERVAL_MS ); // Sleep to give the desired loop rate.
  	}
  }

  System_End();

	CloseCVIRTE();
}

