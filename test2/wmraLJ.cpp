/*
 * wmraLJ.cpp
 *
 *  Created on: August 2, 2010
 *      Author: USF CARRT (WMRA Project)
 *
 *      This wmraLJ.cpp file includes all of the functions that
 *		are used to control the motion of the wheelchair via
 *		PC using the Lab Jack U3 device through USB. The 
 *		Lab Jack device must be connected to the computer through
 *		USB. The Lab Jack driver files must be installed in 
 *		order for this library to work. The driver files are 
 *		included and used in this library. 
 *		This is where we do stuff! Prototypes are in wmraLJ.h.
 *		Please see wmraLJ.h for additional required notes!
 *
 *		DAC0 -> blue wire
 *		DAC1 -> yellow wire
 *		AIN0 -> blue wire
 *		AIN1 -> yellow wire
 *		AIN2 -> I1 proximity sensor
 *		AIN3 -> I2 proximity sensor
 */

#include <stdio.h>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <afxwin.h>
//#include <windows.h>
#include <iostream>
#include <math.h>
#include "wmraLJ.h"
// SEE THE wmraLJ.h FILE FOR INFORMATION ON ADDING THE REQUIRED
// DRIVER FILES AND LIBRARY!!!
#include "C:\Program Files (x86)\LabJack\Drivers\LabJackUD.h"
//#include "C:\Program Files\LabJack\Drivers\LabJackUD.h"

 LJ_HANDLE lngHandleU6=0, lngHandleU3=0 ; //global variables for LabJack U6 & U3 handles

using namespace std; //we use namespace to keep things simple

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
void ErrorHandler (LJ_ERROR lngErrorcode, long lngLineNumber, long lngIteration)
{
	char err[255];

	if (lngErrorcode != LJE_NOERROR)
	{
		ErrorToString(lngErrorcode,err);
		printf("Error number = %d\n",lngErrorcode);
		printf("Error string = %s\n",err);
		printf("Source line number = %d\n",lngLineNumber);
		printf("Iteration = %d\n\n",lngIteration);
		if(lngErrorcode > LJE_MIN_GROUP_ERROR)
		{
			//Quit if this is a group error.
			getchar();
			exit(0);
		}
   }
}

//==========================================================
// InitializeU6() goes through the rounds of finding and 
// opening the Lab Jack U6. You must uses this InitializeU6()
// function before using any other functions. Returns 1 for
// error condition, 0 for proper execution.
//==========================================================
int InitializeU6 ()
{
	LJ_ERROR lngErrorcode;

	

	//Open the first found LabJack U6.
	lngErrorcode = OpenLabJack (LJ_dtU6, LJ_ctUSB, "1", 1, &lngHandleU6);
	
	ErrorHandler(lngErrorcode, __LINE__, 0);
	

	//I think this resets the U6 to the default settings??
	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandleU6, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	return 0;
}

//==========================================================
// InitializeU3() goes through the rounds of finding and 
// opening the Lab Jack U6. You must uses this InitializeU3()
// function before using any other functions. Returns 1 for
// error condition, 0 for proper execution.
//==========================================================
int InitializeU3 ()
{
	LJ_ERROR lngErrorcode;

	

	//Open the first found LabJack U6.
	lngErrorcode = OpenLabJack (LJ_dtU3, LJ_ctUSB, "1", 1, &lngHandleU3);
	
	ErrorHandler(lngErrorcode, __LINE__, 0);
	

	//I think this resets the U6 to the default settings??
	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	lngErrorcode = ePut (lngHandleU3, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	return 0;
}
//==========================================================
// SetTD sets the analog output for DACA and DACB on the LJ
// Tick DAC (+/- 10V). Channel specifies the channel to set
// (A,B) and value is the analog voltage output. Value must 
// be 0-10 (volts). Returns 1 for error condition, 0 for 
// proper execution.
//==========================================================
/*

int SetTD (char channel, double value)
{
	LJ_ERROR lngErrorcode;

	//Here we do some error checking to make sure value is
	//in range (0-10). 
	if ((value < 0) || (value > 10))
	{
		cerr << "In setting Tick DAC voltage, value is out of range. Program will terminate!" << endl;
		return 1;
	}

	//Tell the driver that SCL is on FIO0. The driver then assumes that SDA is on FIO1.
	//This is just setting a parameter in the driver, and not actually talking
	//to the hardware, and thus executes very fast.
	lngErrorcode = ePut (lngHandleU6, LJ_ioPUT_CONFIG, LJ_chTDAC_SCL_PIN_NUM, 0, 0);

	//Here we set the values.
	if ((channel == 'a') || (channel == 'A')) //DACA=value
	{
		lngErrorcode = ePut(lngHandleU6, LJ_ioTDAC_COMMUNICATION, LJ_chTDAC_UPDATE_DACA, value, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);
	}
	else if ((channel == 'b') || (channel == 'B')) //DACB=value
	{
		lngErrorcode = ePut(lngHandleU6, LJ_ioTDAC_COMMUNICATION, LJ_chTDAC_UPDATE_DACB, value, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);
	}
	else //Here we check to make sure channel is valid.
	{
		cerr << "In setting Tick DAC voltage, specified channel does not exist. Program will terminate!" << endl;
		return 1;
	}

	return 0;
}
*/
//==========================================================
// SetDAC sets the analog output for DAC0 and DAC1 (+/- 5V). 
// Channel specifies the channel to set (0,1) and value is 
// the analog voltage output. Value must be 0-5 (volts). 
// Returns 1 for error condition, 0 for proper execution.
//==========================================================
int SetDAC (int channel, double value)
{
	LJ_ERROR lngErrorcode;
	//value = value/54;
	//Here we do some error checking to make sure value is
	//in range (0-5). 
	if ((value < 0) || (value > 5))
	{
		cerr << "In setting Tick DAC voltage, value is out of range. Program will terminate!" << endl;
		return 1;
	}

	//Here we set the values.
	if (channel == 0) //DAC0=value
	{
		lngErrorcode = ePut(lngHandleU6, LJ_ioPUT_DAC, 0, value, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);
	}
	else if (channel == 1) //DAC1=value
	{
		lngErrorcode = ePut(lngHandleU6, LJ_ioPUT_DAC, 1, value, 0);
		ErrorHandler(lngErrorcode, __LINE__, 0);
	}
	else //Here we check to make sure channel is valid.
	{
		cerr << "In setting DAC voltage, specified channel does not exist. Program will terminate!" << endl;
		return 1;
	}

	return 0;
}

//==========================================================
// GetAIN gets the analog input for AIN0 through AIN15 
// (+/- 10V). Channel specifies the channel to read (0-15) 
// and value (analog input) is passed and updated. Returns 1 
// for error condition, 0 for proper execution.
//==========================================================
int GetAIN_U3 (int channel, double &value)
{
	LJ_ERROR lngErrorcode;

	//Here we do some error checking to make sure channel is
	//valid (0-13). 
	if ((channel < 0) || (channel > 13))
	{
		cerr << "In reading AIN voltage, specified channel does not exist. Program will terminate!" << endl;
		return 1;
	}

	//Here we read the values.
	//Take a single-ended measurement from AIN[channel].
	lngErrorcode = eAIN (lngHandleU3, channel, 31, &value, -1, -1, -1, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	return 0;
}


//==========================================================
// GetAIN gets the analog input for AIN0 through AIN15 
// (+/- 10V). Channel specifies the channel to read (0-15) 
// and value (analog input) is passed and updated. Returns 1 
// for error condition, 0 for proper execution.
//==========================================================
int GetAIN_U6 (int channel, double &value)
{
	LJ_ERROR lngErrorcode;

	//Here we do some error checking to make sure channel is
	//valid (0-13). 
	if ((channel < 0) || (channel > 13))
	{
		cerr << "In reading AIN voltage, specified channel does not exist. Program will terminate!" << endl;
		return 1;
	}

	//Here we read the values.
	//Take a single-ended measurement from AIN[channel].
	lngErrorcode = eAIN (lngHandleU6, channel, 199, &value, -1, -1, -1, 0, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	return 0;
}

//==========================================================
// SetDO sets the digital output for FIO0 through MIO2 
// (3.3/5V). Channel specifies the channel to read (0-22) 
// and value is the desired output (1=high, 0=low). Returns 
// 1 for error condition, 0 for proper execution.
//==========================================================
int SetDO (int channel, long value)
{
	LJ_ERROR lngErrorcode;

	//Here we do some error checking to make sure channel is
	//valid (0-22). 
	if ((channel < 0) || (channel > 22))
	{
		cerr << "In setting DO, specified channel does not exist. Program will terminate!" << endl;
		return 1;
	}

	//Here we do some error checking to make sure value is
	//valid (0 or 1).
	if ((value < 0) || (value > 1))
	{
		cerr << "In setting DO, va lue is out of range. Program will terminate!" << endl;
		return 1;
	}
	
	//Here we set the value.
	lngErrorcode = eDO (lngHandleU3, channel, value);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	return 0;
}

//==========================================================
// GetDI gets the digital input for FIO0 through MIO2 
// (3.3/5V). Channel specifies the channel to read (0-22) 
// and value is the digital input (1=high, 0=low). Returns 
// 1 for error condition, 0 for proper execution.
//==========================================================
int GetDI (int channel, long &value)
{
	LJ_ERROR lngErrorcode;

	//Here we do some error checking to make sure channel is
	//valid (0-22). 
	if ((channel < 0) || (channel > 22))
	{
		cerr << "In getting DI, specified channel does not exist. Program will terminate!" << endl;
		return 1;
	}

	//Here we set the value.
	lngErrorcode = eDI (lngHandleU3, channel, &value);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	return 0;
}

//==========================================================
// StartStream sets up the stream to read the sonar 
// proximity sensor distance. Stream is configured and 
// started, then GetStreamDistance() is used to get the 
// distance. Returns 1 for error condition, 0 for proper
// execution.
//==========================================================
int StartStream ()
{
	LJ_ERROR lngErrorcode;
	long lngGetNextIteration;
	long lngIOType = 0, lngChannel = 0;
	double dblValue = 0;
	double scanRate = 50000;

	//cout << "Starting stream..." << endl;

	//Start by using the pin_configuration_reset IOType so that all
	//pin assignments are in the factory default condition.
	//lngErrorcode = ePut (lngHandle, LJ_ioPIN_CONFIGURATION_RESET, 0, 0, 0);
	//ErrorHandler(lngErrorcode, __LINE__, 0);
	//This is done in Initialize(), so we comment it out here

	//Configure FIO0 and FIO1 as analog, all else as digital.  That means we
	//will start from channel 0 and update all 16 flexible bits.  We will
	//pass a value of b0000000000000011 or d3.
	lngErrorcode = ePut (lngHandleU3, LJ_ioPUT_ANALOG_ENABLE_PORT, 0, 3, 16);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Configure the stream:
    //Set the scan rate.
    lngErrorcode = AddRequest(lngHandleU3, LJ_ioPUT_CONFIG, LJ_chSTREAM_SCAN_FREQUENCY, scanRate, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Give the driver a 5 second buffer (scanRate * 2 channels * 5 seconds).
    lngErrorcode = AddRequest(lngHandleU3, LJ_ioPUT_CONFIG, LJ_chSTREAM_BUFFER_SIZE, scanRate*2*5, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Configure reads to retrieve whatever data is available without waiting (wait mode LJ_swNONE).
	//See comments below to change this program to use LJ_swSLEEP mode.
    lngErrorcode = AddRequest(lngHandleU3, LJ_ioPUT_CONFIG, LJ_chSTREAM_WAIT_MODE, LJ_swNONE, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    //Define the scan list as AIN0 then FIOEIO.
    lngErrorcode = AddRequest(lngHandleU3, LJ_ioCLEAR_STREAM_CHANNELS, 0, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    lngErrorcode = AddRequest(lngHandleU3, LJ_ioADD_STREAM_CHANNEL, 193, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
    //Execute the list of requests.
    lngErrorcode = GoOne(lngHandleU3);
    ErrorHandler(lngErrorcode, __LINE__, 0);
    
	//Get all the results just to check for errors.
	lngErrorcode = GetFirstResult(lngHandleU3, &lngIOType, &lngChannel, &dblValue, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngGetNextIteration=0;	//Used by the error handling function.

	while (lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
		lngErrorcode = GetNextResult(lngHandleU3, &lngIOType, &lngChannel, &dblValue, 0, 0);

		if (lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
		{
			ErrorHandler (lngErrorcode, __LINE__, lngGetNextIteration);
		}

		lngGetNextIteration++;
	}
    
	//Start the stream.
    lngErrorcode = eGet(lngHandleU3, LJ_ioSTART_STREAM, 0, &dblValue, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);

	//cout << "Stream started" << endl;

	return 0;
}

//==========================================================
// GetStreamDistance reads the sonar proximity sensor 
// distance in cm. Sonar sensor signal wire is on EIO0 for
// S1 and EIO1 for S2. StartStream() must be called before 
// GetStreamDistance. Updates s1 and s2 with distances in
// cm. Returns 1 for error condition, 0 for proper 
// execution.
//==========================================================
int GetStreamDistance (double &s1, double &s2, double &s3, double &s4)
{
	LJ_ERROR lngErrorcode;
	int k = 0;
	long value = 0;
	long delayms = 500; //default 500
	double numScans = 50000;  //2x the expected # of scans (2*scanRate*delayms/1000)
	double numScansRequested;
	double adblData[50000] = {0};  //Max buffer size (#channels*numScansRequested)
	int count1 = 0, flag1 = 0, count2 = 0, flag2 = 0, count3 = 0, flag3 = 0, count4 = 0, flag4 = 0;

	//Output data can be printed to file, uncomment outfile statements to print
	//FILE *results;

	//Make a long parameter which holds the address of the data array.  We do this
	//so the compiler does not generate a warning in the eGet call that retrieves
	//stream data.  Note that the x1 parameter  in eGet (and AddRequest) is fairly
	//generic, in that sometimes it could just be a write parameter, and sometimes
	//it has the address of an array.  Since x1 is not declared as a pointer, the
	//compiler will complain if you just pass the array pointer without casting
	//it to a long as follows.
	long padblData = (long)&adblData[0];

	//Send pulse signal to EIO4
	if (SetDO(12, 1))
	{
		cerr << "There was a problem sending the pulse. Program will terminate!" << endl;
		return 1;
	}

	//Set EIO4 back to digital input
	if (GetDI(12, value))
	{
		cerr << "There was a problem getting DI. Program will terminate!" << endl;
		return 1;
	}

	Sleep (38);

	//Send pulse signal to EIO0
	if (SetDO(8, 1))
	{
		cerr << "There was a problem sending the pulse. Program will terminate!" << endl;
		return 1;
	}

	//Set EIO0 back to digital input
	if (GetDI(8, value))
	{
		cerr << "There was a problem getting DI. Program will terminate!" << endl;
		return 1;
	}

	Sleep (38);


	//Send pulse signal to EIO6
	if (SetDO(14, 1))
	{
		cerr << "There was a problem sending the pulse. Program will terminate!" << endl;
		return 1;
	}
	//Set EIO6 back to digital input
	if (GetDI(14, value))
	{
		cerr << "There was a problem getting DI. Program will terminate!" << endl;
		return 1;
	}
	Sleep (38);
	//Send pulse signal to EIO2
	if (SetDO(10, 1))
	{
		cerr << "There was a problem sending the pulse. Program will terminate!" << endl;
		return 1;
	}

	//Set EIO2 back to digital input
	if (GetDI(10, value))
	{
		cerr << "There was a problem getting DI. Program will terminate!" << endl;
		return 1;
	}

	

	

	
	

	//Read data
	//Since we are using wait mode LJ_swNONE, we will wait a little, then
	//read however much data is available.  Thus this delay will control how
	//fast the program loops and how much data is read each loop.  An
	//alternative common method is to use wait mode LJ_swSLEEP where the
	//stream read waits for a certain number of scans.  In such a case
	//you would not have a delay here, since the stream read will actually
	//control how fast the program loops.
	//
	//To change this program to use sleep mode,
	//	-change numScans to the actual number of scans desired per read,
	//	-change wait mode addrequest value to LJ_swSLEEP,
	//	-comment out the following Sleep command.
	Sleep(delayms);	//Remove if using LJ_swSLEEP.

	//Read the data.  We will request twice the number we expect, to
	//make sure we get everything that is available.
	//Note that the array we pass must be sized to hold enough SAMPLES, and
	//the Value we pass specifies the number of SCANS to read.
	numScansRequested=numScans;
	lngErrorcode = eGet(lngHandleU3, LJ_ioGET_STREAM_DATA, LJ_chALL_CHANNELS, &numScansRequested, padblData);

	//if ((results = fopen("results.txt", "w")) == NULL)
	//{
	//	cerr << "Cannot open output file!" << endl;
	//}

	for (k=0; k<numScans; k++)
	{
		//fprintf(results,"Scan = %.3f\n",adblData[k]); //Print results to file

		//Results for sonar4 (EIO6)
		if ((flag4 == 0) && ((adblData[k] == 65520) || (adblData[k] == 65264) || (adblData[k] == 64496) || (adblData[k] == 64240) || (adblData[k] == 61424) || (adblData[k] == 61168) || (adblData[k] == 60400) || (adblData[k] == 60144))) //When we reach initial pulse, set flag to 1
		{//first high pulse
			flag4 = 1;
		}
		else if ((flag4 == 1) && ((adblData[k] == 49136) || (adblData[k] == 48880) || (adblData[k] == 48112) || (adblData[k] == 47856) || (adblData[k] == 45040) || (adblData[k] == 44784) || (adblData[k] == 44016) || (adblData[k] == 43760))) //When we go back to hold period, set flag to 2
		{//goes low
			flag4 = 2;
		}
		else if ((flag4 == 2) && ((adblData[k] == 65520) || (adblData[k] == 65264) || (adblData[k] == 64496) || (adblData[k] == 64240) || (adblData[k] == 61424) || (adblData[k] == 61168) || (adblData[k] == 60400) || (adblData[k] == 60144))) //When we reach the response pulse, measure delay
		{//goes high again
			count4++;
		}

		//Results for sonar3 (EIO4)
		if ((flag3 == 0) && ((adblData[k] == 65520) || (adblData[k] == 65264) || (adblData[k] == 64496) || (adblData[k] == 64240) || (adblData[k] == 49136) || (adblData[k] == 48880) || (adblData[k] == 48112) || (adblData[k] == 47856))) //When we reach initial pulse, set flag to 1
		{//first high pulse
			flag3 = 1;
		}
		else if ((flag3 == 1) && ((adblData[k] == 61424) || (adblData[k] == 61168) || (adblData[k] == 60400) || (adblData[k] == 60144) || (adblData[k] == 45040) || (adblData[k] == 44784) || (adblData[k] == 44016) || (adblData[k] == 43760))) //When we go back to hold period, set flag to 2
		{//goes low
			flag3 = 2;
		}
		else if ((flag3 == 2) && ((adblData[k] == 65520) || (adblData[k] == 65264) || (adblData[k] == 64496) || (adblData[k] == 64240) || (adblData[k] == 49136) || (adblData[k] == 48880) || (adblData[k] == 48112) || (adblData[k] == 47856))) //When we reach the response pulse, measure delay
		{//goes high again
			count3++;
		}

		//Results for sonar2 (EIO2)
		if ((flag2 == 0) && ((adblData[k] == 65520) || (adblData[k] == 65264) || (adblData[k] == 61424) || (adblData[k] == 61168) || (adblData[k] == 49136) || (adblData[k] == 48880) || (adblData[k] == 45040) || (adblData[k] == 44784))) //When we reach initial pulse, set flag to 1
		{//first high pulse
			flag2 = 1;
		}
		else if ((flag2 == 1) && ((adblData[k] == 64496) || (adblData[k] == 64240) || (adblData[k] == 60400) || (adblData[k] == 60144) || (adblData[k] == 48112) || (adblData[k] == 47856) || (adblData[k] == 44016) || (adblData[k] == 43760))) //When we go back to hold period, set flag to 2
		{//goes low
			flag2 = 2;
		}
		else if ((flag2 == 2) && ((adblData[k] == 65520) || (adblData[k] == 65264) || (adblData[k] == 61424) || (adblData[k] == 61168) || (adblData[k] == 49136) || (adblData[k] == 48880) || (adblData[k] == 45040) || (adblData[k] == 44784))) //When we reach the response pulse, measure delay
		{//goes high again
			count2++;
		}

		//Results for sonar1 (EIO0)
		if ((flag1 == 0) && ((adblData[k] == 65520) || (adblData[k] == 64496) || (adblData[k] == 61424) || (adblData[k] == 60400) || (adblData[k] == 49136) || (adblData[k] == 48112) || (adblData[k] == 45040) || (adblData[k] == 44016))) //When we reach initial pulse, set flag to 1
		{//first high pulse
			flag1 = 1;
		}
		else if ((flag1 == 1) && ((adblData[k] == 65264) || (adblData[k] == 64240) || (adblData[k] == 61168) || (adblData[k] == 60144) || (adblData[k] == 48880) || (adblData[k] == 47856) || (adblData[k] == 44784) || (adblData[k] == 43760))) //When we go back to hold period, set flag to 2
		{//goes low
			flag1 = 2;
		}
		else if ((flag1 == 2) && ((adblData[k] == 65520) || (adblData[k] == 64496) || (adblData[k] == 61424) || (adblData[k] == 60400) || (adblData[k] == 49136) || (adblData[k] == 48112) || (adblData[k] == 45040) || (adblData[k] == 44016))) //When we reach the response pulse, measure delay
		{//goes high again
			count1++;
		}

	}

	//fclose(results);

	//We divide by the calibration factor (2.95) to find distance in cm
	s1 = count1 / 2.95;
	s2 = count2 / 2.95;
	s3 = count3 / 2.95;
	s4 = count4 / 2.95;

	return 0;
}

//==========================================================
// StopStream stops the stream reading the sonar proximity
// sensors. Stream should be stopped after the application 
// is done reading distance values. Returns 1 for error 
// condition, 0 for proper execution.
//==========================================================
int StopStream ()
{
	LJ_ERROR lngErrorcode;

	//cout << "Stopping stream..." << endl;

    //Stop the stream
    lngErrorcode = eGet(lngHandleU3, LJ_ioSTOP_STREAM, 0, 0, 0);
    ErrorHandler(lngErrorcode, __LINE__, 0);

	//cout << "Stream stopped" << endl;

	return 0;
}

//==========================================================
// InitializeTimers configures and starts the timers for 
// encoder 1 and 2. Note that when this function is called,
// timers are reset. Returns 1 for error condition, 0 for 
// proper execution.
//==========================================================
int InitializeTimersU6 ()
{
	LJ_ERROR lngErrorcode;
	long lngIOType, lngChannel, lngGetNextIteration;
	double dblValue;

	//TIMER SETUP---->
	//Set pin offset to 2
	AddRequest (lngHandleU6, LJ_ioPUT_CONFIG, LJ_chTIMER_COUNTER_PIN_OFFSET, 2, 0, 0);

	//Enable 4 timers. They will use FIO2-FIO5
	AddRequest (lngHandleU6, LJ_ioPUT_CONFIG, LJ_chNUMBER_TIMERS_ENABLED, 4, 0, 0);

	//Make sure Counters are disabled.
	AddRequest (lngHandleU6, LJ_ioPUT_COUNTER_ENABLE, 0, 0, 0, 0);
	AddRequest (lngHandleU6, LJ_ioPUT_COUNTER_ENABLE, 1, 0, 0, 0);

	//Configure Timers as quadrature.
	AddRequest(lngHandleU6, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_BASE, LJ_tc1MHZ_DIV, 0, 0);
	AddRequest(lngHandleU6, LJ_ioPUT_CONFIG, LJ_chTIMER_CLOCK_DIVISOR, 1, 0, 0);
	AddRequest(lngHandleU6, LJ_ioPUT_TIMER_MODE, 0, LJ_tmQUAD, 0, 0); 
	AddRequest(lngHandleU6, LJ_ioPUT_TIMER_MODE, 1, LJ_tmQUAD, 0, 0);
	AddRequest(lngHandleU6, LJ_ioPUT_TIMER_MODE, 2, LJ_tmQUAD, 0, 0); 
	AddRequest(lngHandleU6, LJ_ioPUT_TIMER_MODE, 3, LJ_tmQUAD, 0, 0);

	//Reset value
	AddRequest(lngHandleU6, LJ_ioPUT_TIMER_VALUE, 0, 0, 0, 0);
	
	lngErrorcode = GoOne (lngHandleU6);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	//Get all the results.  The input measurement results are stored.  All other
	//results are for configuration or output requests so we are just checking
	//whether there was an error.
	lngErrorcode = GetFirstResult(lngHandleU6, &lngIOType, &lngChannel, &dblValue, 0, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	lngGetNextIteration=0;	//Used by the error handling function.
	while(lngErrorcode < LJE_MIN_GROUP_ERROR)
	{
		lngErrorcode = GetNextResult(lngHandleU6, &lngIOType, &lngChannel, &dblValue, 0, 0);

		if(lngErrorcode != LJE_NO_MORE_DATA_AVAILABLE)
		{
			ErrorHandler(lngErrorcode, __LINE__, lngGetNextIteration);
		}

		lngGetNextIteration++;
	}
	//END TIMER SETUP---->

	return 0;
}

//==========================================================
// GetTimers gets the timer values for encoder 1 and 2. The
// parameters are passed into the function and updated. 
// Returns 1 for error condition, 0 for proper execution.
//==========================================================
int GetTimers (double &encoder1, double &encoder2)
{
	LJ_ERROR lngErrorcode;

	//Read values from encoders
	//ENCODER1 (read FIO2 : FIO2+FIO3 quad1)
	lngErrorcode = eGet (lngHandleU6, LJ_ioGET_TIMER, 0, &encoder1, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);
	//ENCODER2 (read FIO4 : FIO4+FIO5 quad2)
	lngErrorcode = eGet (lngHandleU6, LJ_ioGET_TIMER, 2, &encoder2, 0);
	ErrorHandler(lngErrorcode, __LINE__, 0);

	return 0;
}

//==========================================================
// Stop() is a function that stops the wheelchair motion and
// sets it to an idle condition. It sets the analog outputs
// to 2.5V for idle condition. Returns 1 for error 
// condition, 0 for proper execution.
//==========================================================
int Stop ()
{
	if ((SetDAC(0, 2.5)) || (SetDAC(1, 2.5)))
	{
		cerr << "There was a problem setting reference. Program will terminate!" << endl;
		return 1;
	}

	return 0;
}


//==========================================================
// GetReference() reads in the analog input value for 
// reference from the joystick (green wire). This will give
// us the proper value to set the analog outputs for an idle
// situation. It returns the value of the reference, which
// should be close to 6V. 
//==========================================================
double GetReference ()
{
	double green = 6;
	
	//Take a single-ended measurement from AIN0. AIN0 is the green wire!
	if (GetAIN_U6(0, green))
	{
		cerr << "There was a problem reading reference. Program will terminate!" << endl;
		return 6;
	}

	return green;
}