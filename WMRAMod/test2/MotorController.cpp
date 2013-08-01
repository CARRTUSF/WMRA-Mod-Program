
#define _USE_MATH_DEFINES

#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include "stringUtility.h"
#include "MotorController.h"
#include "galilController.h"
//#include "Galil.h"

using namespace std;

// PI = M_PI;

#define PI 3.14159265

//Galil MotorController::controller("192.168.1.22");
galilController controller;
string MotorController::motorLookup[] = {"H","A","B","C","D","E","F","G","H"};


MotorController::MotorController()
{
	long int sum;
	initialized = false;

	//calculate conversion values
	enc2Radian[1] = PI/6595000;//a
	enc2Radian[2] = PI/5100000;//b
	enc2Radian[3] = PI/1800000;//c
	enc2Radian[4] = PI/2300000;//d
	enc2Radian[5] = PI/2000000;//e
	enc2Radian[6] = -PI/2000000;//f
	enc2Radian[7] = PI/1500000; //g
	
	rad2Enc[0] = 0;
	rad2Enc[1] = 6595000/PI;//a
	rad2Enc[2] = 5100000/PI;//b
	rad2Enc[3] = 1800000/PI;//c
	rad2Enc[4] = 2300000/PI;//d
	rad2Enc[5] = 2000000/PI;//e
	rad2Enc[6] = -2000000/PI;//f
	rad2Enc[7] = 1500000/PI; //g
	rad2Enc[8] = 0;

	sum = 6595000+5100000+1800000+2300000+2000000+2000000+1500000;
	normalize[1] = 6595000/sum;
	normalize[2] = 5100000/sum;
	normalize[3] = 1800000/sum;
	normalize[4] = 2300000/sum;
	normalize[5] = 2000000/sum;
	normalize[6] = 2000000/sum;
	normalize[7] = 1500000/sum;
	

	//read settings
	//call initialize();
	//set initilez flag
}
//MotorController::~MotorController()
//{
//	controller.command("MO"); //turn off motors
//}
bool MotorController::initialize(){
	// set PID values

	// set motor types as in brushed motr etc..
/*
	controller.command("BRA=1");
	controller.command("BRB=1");
	controller.command("BRC=1");
	controller.command("BRD=1");

	//set all motors to position tracking mode
	controller.command("PTA=1");
	controller.command("PTB=1");
	controller.command("PTC=1");
	controller.command("PTD=1");
	controller.command("PTE=1");
	controller.command("PTF=1");
	controller.command("PTG=1");
	controller.command("PTH=1");

	// set accelaration decelaration values
	//#debug CHANGE LATER TO PROPER ANGLES

	setAccel(1, 0.2);
	setAccel(2, 0.2);
	setAccel(3, 0.2);
	setAccel(4, 0.2);
	setAccel(5, 0.2);
	setAccel(6, 0.2);
	setAccel(7, 0.2);

	setDecel(1, 0.2);
	setDecel(2, 0.2);
	setDecel(3, 0.2);
	setDecel(4, 0.2);
	setDecel(5, 0.2);
	setDecel(6, 0.2);
	setDecel(7, 0.2);

	//set accelaration smoothing 
	controller.command("IT*=0.6");	

	// ready position, #debug I believe this should be a function of its own, and the ready position should be recovered from the text file as well.
	definePosition(1, (PI/2));
	definePosition(2, (PI/2));
	definePosition(3, 0);
	definePosition(4, (PI/2));
	definePosition(5, (PI/2));
	definePosition(6, -(PI/2));
	definePosition(7, 0);
	definePosition(8, 0);

	controller.command("SH"); //turn on motors
*/
	controller.initialize("192.168.1.22");
	initialized = wmraSetup();
	return true;
}

bool MotorController::isInitialized() // return initialized
{
	return initialized;
}

bool MotorController::wmraSetup() //WMRA setup
{
	// set motor types as in brushed motr etc..
	controller.command("BRA=1");
	controller.command("BRB=1");
	controller.command("BRC=1");
	controller.command("BRD=1");

	//set all motors to position tracking mode
	controller.command("PTA=1");
	controller.command("PTB=1");
	controller.command("PTC=1");
	controller.command("PTD=1");
	controller.command("PTE=1");
	controller.command("PTF=1");
	controller.command("PTG=1");
	controller.command("PTH=1");

	// set accelaration decelaration values
	//#debug CHANGE LATER TO PROPER ANGLES

	setAccel(1, 0.2);
	setAccel(2, 0.2);
	setAccel(3, 0.2);
	setAccel(4, 0.2);
	setAccel(5, 0.2);
	setAccel(6, 0.2);
	setAccel(7, 0.2);

	setDecel(1, 0.2);
	setDecel(2, 0.2);
	setDecel(3, 0.2);
	setDecel(4, 0.2);
	setDecel(5, 0.2);
	setDecel(6, 0.2);
	setDecel(7, 0.2);

	//set accelaration smoothing 
	controller.command("IT*=0.6");	

	// ready position, #debug I believe this should be a function of its own, and the ready position should be recovered from the text file as well.
	definePosition(1, (PI/2));
	definePosition(2, (PI/2));
	definePosition(3, 0);
	definePosition(4, (PI/2));
	definePosition(5, (PI/2));
	definePosition(6, -(PI/2));
	definePosition(7, 0);
	definePosition(8, 0);

	controller.command("SH"); //turn on motors
	
	return 1;
}

bool MotorController::Stop() //emergancy stop
{
	controller.command("ST ABCDEFGH");
	return true; // #debug does this return need to happen after the Arm has fully stopped?
}

bool MotorController::Stop(int motorNum) // emergancy stop a single motor
{

	if(isValidMotor(motorNum)){
		string motor = motorLookup[motorNum];
		controller.command("ST " + motor);
		return true;
	}
	else{
		return false;
	}

}
bool MotorController::setPID(int motorNum, int P, int I, int D){
	return false;
}


float MotorController::readPos(int motorNum) // returns the current motor angle in radians
{
	long encoderVal;	
	string result;
	string motor;
	if ( isValidMotor(motorNum)){
		motor = motorLookup[motorNum];
		result = controller.command( "TP" + motor);	
		istringstream stream(result);
		stream >> encoderVal;
		return encToAng(motorNum, encoderVal);        
	}
	else{
		if(motorNum != 9 && motorNum != 10)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

float MotorController::readPosErr(int motorNum) // returns the error in  
{

	long encoderVal;	
	string result;
	string motor;
	if ( isValidMotor(motorNum)){
		motor = motorLookup[motorNum];
		result = controller.command( "TE" + motor);	
		istringstream stream(result);
		stream >> encoderVal;
		return encToAng(motorNum, encoderVal);        
	}
	else{
		if(motorNum != 9 && motorNum != 10)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

bool MotorController::setMaxVelocity(int motorNum, float angularVelocity)
{
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angularVelocity));
		string motor = motorLookup[motorNum];
		if ((encVal >= 0) && (encVal < 12000000)){
			string command = "SP" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The velocity is outside the range" << endl;
			throw std::out_of_range ("velocity out_of_range");            
		}
	}
	else{
		if(motorNum != 9 && motorNum != 10)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}

}

bool MotorController::setAccel(int motorNum, float angularAccelaration)
{	
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angularAccelaration));
		string motor = motorLookup[motorNum];
		if ((encVal >= 1024) && (encVal <= 67107840)){
			string command = "AC" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The Accelaration is outside the range" << endl;
			throw std::out_of_range ("Accelaration out_of_range");            
		}
	}
	else{
		if(motorNum != 9 && motorNum != 10)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}

}

bool MotorController::setDecel(int motorNum, float angularDecelaration)
{
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angularDecelaration));
		string motor = motorLookup[motorNum];
		if ((encVal >= 1024) && (encVal <= 67107840)){
			string command = "DC" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The Decelaration is outside the range" << endl;
			throw std::out_of_range ("Decelaration out_of_range");            
		}
	}
	else{
		if(motorNum != 9 && motorNum != 10)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}

}

bool MotorController::definePosition(int motorNum,float angle)
{
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angle));
		string motor = motorLookup[motorNum];
		if ((encVal >= -2147483647) && (encVal <= 2147483648)){
			string command = "DP" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The Position is outside the range" << endl;
			throw std::out_of_range ("Position out_of_range");            
		}
	}
	else{
		if(motorNum != 9 && motorNum != 10)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

bool MotorController::positionControl(int motorNum,float angle)
{
	if(isValidMotor(motorNum)){
		long encVal = (angToEnc(motorNum,angle));
		string motor;
		motor = motorLookup[motorNum];
		long temp = abs(encVal);
		if (temp <= 2147483648){
			string command = "PA" + motor + "=" + toString(encVal);
			try	{
				controller.command(command);
			}
			catch(string s){
				cout << s << endl;
				cout << endl;
			}
			/*
			string command2 = "BG " + motor;
			try	{
				controller.command(command2);
			}
			catch(string s){
				cout << s << endl;
				cout << endl;
			}
			*/
			return true;            
		}
		else{
			cerr << "The Position is outside the range" << endl;
			throw std::out_of_range ("Position out_of_range");            
		}
	}
	else{
		if(motorNum != 9 && motorNum != 10)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

bool MotorController::MotorsOFF()
{
	controller.command("MO"); //turn off motors
	return 1;
}

/*------------------------------------------------------

Private Functions

------------------------------------------------------*/

inline bool MotorController::isValidMotor(int motorNum){
	if( motorNum >= 1 && motorNum <= 8) return true;
	else return false;
}

bool readSettings() // Returns 0 is no file found or error occured
{
	return true;

}

float MotorController::encToAng(int motorNum, long encCount) // #debug needs to be finished, Also need to check initialized
{
	if (motorNum < 9 && motorNum > 0){
		return encCount * enc2Radian[motorNum]; 
	}
	else{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}

}

long MotorController::angToEnc(int motorNum, float encCount) // #debug needs to be finished, Also need to check initialized
{
	if (motorNum < 9 && motorNum > 0){
		return encCount * rad2Enc[motorNum]; 
	}
	else{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}

}