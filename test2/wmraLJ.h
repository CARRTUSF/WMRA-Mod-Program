/*
 * wmraLJ.h
 *
 *  Created on: August 2, 2010
 *      Author: USF CARRT (WMRA Project)
 *
 *      This wmraLJ.h file includes all of the functions that
 *		are used to control the motion of the wheelchair via
 *		PC using the Lab Jack U3 device through USB. The 
 *		Lab Jack device must be connected to the computer through
 *		USB. The Lab Jack driver files must be installed in 
 *		order for this library to work. The driver files are 
 *		included and used in this library. 
 *
 *		DAC0 -> blue wire
 *		DAC1 -> yellow wire
 *		AIN0 -> blue wire
 *		AIN1 -> yellow wire
 *		AIN2 -> I1 proximity sensor
 *		AIN3 -> I2 proximity sensor
 *
 */

#ifndef WMRALJ_H //these statements define wmraLJ.h
#define WMRALJ_H // if wmraLJ.h has not been defined

// YOU MUST HAVE THE FILE BELOW INSTALLED ON THE PC RUNNING THIS
// PROGRAM. YOU MUST ALSO HAVE THE labjackud.lib FILE INCLUDED IN
// THE PROJECT BY SPECIFYING THE LOCATION ON YOUR PC. RIGHT-CLICK
// THE PROJECT AND GO TO ADD->EXISTING ITEM. BROWSE TO THE 
// labjackud.lib FILE WHICH SHOULD BE LOCATED AT 
// C:\Program Files\LabJack\Drivers BY DEFAULT WHEN THE LAB JACK
// DRIVER IS INSTALLED. THE FILE LabJackUD.h MUST BE LOCATED AT 
// THE DIRECTORY AS SPECIFIED DIRECTLY BELOW. 
// If Visual Studio complains (it does this a lot) after you add 
// the labjackud.lib file to this project, just click No. We are
// only referencing the library file as per LabJackUD.h, not 
// editing or building it. 
#include "C:\Program Files (x86)\LabJack\Drivers\LabJackUD.h"
//#include "C:\Program Files\LabJack\Drivers\LabJackUD.h"
// It is probably a good idea to go ahead and right-click Header
// Files and Add->Existing Item and then add LabJackUD.h as well.
// LabJackUD.h is located in the same Drivers directory mentioned
// above.

//using namespace std; //we use namespace to keep things simple

//==========================================================
// ErrorHandler() is an error checking function built in. 
// We use this when we call a Lab Jack function just in case
// the Lab Jack has an issue during operation. The sample
// code uses this function, so we may as well to stay on the
// safe side. 
//==========================================================
//This is our simple error handling function that is called after every UD
//function call.  This function displays the errorcode and string description
//of the error.  It also has a line number input that can be used with the
//macro __LINE__ to display the line number in source code that called the
//error handler.  It also has an iteration input is useful when processing
//results in a loop (getfirst/getnext).
void ErrorHandler (LJ_ERROR lngErrorcode, long lngLineNumber, long lngIteration);

//==========================================================
// Initialize() goes through the rounds of finding and 
// opening the Lab Jack U3. You must uses this Initialize()
// function before using any other functions. Returns 1 for
// error condition, 0 for proper execution.
//==========================================================
int InitializeU6 ();

//==========================================================
// Initialize() goes through the rounds of finding and 
// opening the Lab Jack U3. You must uses this Initialize()
// function before using any other functions. Returns 1 for
// error condition, 0 for proper execution.
//==========================================================
int InitializeU3 ();

//==========================================================
// SetTD sets the analog output for DACA and DACB on the LJ
// Tick DAC (+/- 10V). Channel specifies the channel to set
// (A,B) and value is the analog voltage output. Value must 
// be 0-10 (volts). Returns 1 for error condition, 0 for 
// proper execution.
//==========================================================
//int SetTD (char channel, double value);

//==========================================================
// SetDAC sets the analog output for DAC0 and DAC1 (+/- 5V). 
// Channel specifies the channel to set (0,1) and value is 
// the analog voltage output. Value must be 0-5 (volts). 
// Returns 1 for error condition, 0 for proper execution.
//==========================================================
int SetDAC (int channel, double value);

//==========================================================
// GetAIN gets the analog input for AIN0 through AIN13 
// (+/- 10V). Channel specifies the channel to read (0-13) 
// and value (analog input) is passed and updated. Returns 1 
// for error condition, 0 for proper execution.
//==========================================================
int GetAIN_U6 (int channel, double &value);

//==========================================================
// GetAIN gets the analog input for AIN0 through AIN13 
// (+/- 10V). Channel specifies the channel to read (0-13) 
// and value (analog input) is passed and updated. Returns 1 
// for error condition, 0 for proper execution.
//==========================================================
int GetAIN_U3 (int channel, double &value);
//==========================================================
// SetDO sets the digital output for FIO0 through MIO2 
// (3.3/5V). Channel specifies the channel to set (0-22) 
// and value is the desired output (1=high, 0=low). Returns 
// 1 for error condition, 0 for proper execution.
//==========================================================
int SetDO (int channel, long value);

//==========================================================
// GetDI gets the digital input for FIO0 through MIO2 
// (3.3/5V). Channel specifies the channel to read (0-22) 
// and value is the digital input (1=high, 0=low). Returns 
// 1 for error condition, 0 for proper execution.
//==========================================================
int GetDI (int channel, long &value);

//==========================================================
// StartStream sets up the stream to read the sonar 
// proximity sensor distance. Stream is configured and 
// started, then GetStreamDistance() is used to get the 
// distance. Returns 1 for error condition, 0 for proper
// execution.
//==========================================================
int StartStream ();

//==========================================================
// GetStreamDistance reads the sonar proximity sensor 
// distance in cm. Sonar sensor signal wire is on EIO0 for
// S1 and EIO1 for S2. StartStream() must be called before 
// GetStreamDistance. Updates s1 and s2 with distances in
// cm. Returns 1 for error condition, 0 for proper 
// execution.
//==========================================================
int GetStreamDistance (double &s1, double &s2, double &s3, double &s4);

//==========================================================
// StopStream stops the stream reading the sonar proximity
// sensors. Stream should be stopped after the application 
// is done reading distance values. Returns 1 for error 
// condition, 0 for proper execution.
//==========================================================
int StopStream ();

//==========================================================
// InitializeTimers configures and starts the timers for 
// encoder 1 and 2. Note that when this function is called,
// timers are reset. Returns 1 for error condition, 0 for 
// proper execution.
//==========================================================
int InitializeTimersU6 ();

//==========================================================
// GetTimers gets the timer values for encoder 1 and 2. The
// parameters are passed into the function and updated. 
// Returns 1 for error condition, 0 for proper execution.
//==========================================================
int GetTimers (double &encoder1, double &encoder2);

//==========================================================
// Stop() is a function that stops the wheelchair motion and
// sets it to an idle condition. It sets the analog outputs
// to 2.5V for idle condition. Returns 1 for error 
// condition, 0 for proper execution.
//==========================================================
int Stop ();

//==========================================================
// GetReference() reads in the analog input value for 
// reference from the joystick (green wire). This will give
// us the proper value to set the analog outputs for an idle
// situation. It returns the value of the reference, which
// should be close to 6V. Returns 6 on error condition.
//==========================================================
double GetReference ();

#endif /* WMRALJ_H_ */
