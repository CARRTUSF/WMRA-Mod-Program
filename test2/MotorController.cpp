
#define _USE_MATH_DEFINES

#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include "stringUtility.h"
#include "MotorController.h"
#include "galilController.h"

using namespace std;

#define PI 3.14159265

galilController MotorController::controller;
string MotorController::motorLookup[] = {"H","A","B","C","D","E","F","G","H"};


MotorController::MotorController()
{
	long int sum;
	initialized = false;

	//calculate conversion values
	enc2Radian[1] = (2*PI)/13200000;//a
	enc2Radian[2] = (2*PI)/13320000;//b
	enc2Radian[3] = (2*PI)/ 4720000;//c
	enc2Radian[4] = (2*PI)/ 4720000;//d
	enc2Radian[5] = (2*PI)/ 3840000;//e
	enc2Radian[6] = -(2*PI)/4000000;//f
	enc2Radian[7] = (2*PI)/ 2880000; //g
	
	rad2Enc[0] = 0;
	rad2Enc[1] = 1/enc2Radian[1];
	rad2Enc[2] = 1/enc2Radian[2];
	rad2Enc[3] = 1/enc2Radian[3];
	rad2Enc[4] = 1/enc2Radian[4];
	rad2Enc[5] = 1/enc2Radian[5];
	rad2Enc[6] = 1/enc2Radian[6];
	rad2Enc[7] = 1/enc2Radian[7];
	rad2Enc[8] = 1/enc2Radian[7]; // #debug need to find the ratio for gripper

}

bool MotorController::initialize(){

	controller.initialize();
	initialized = wmraSetup();
	return true;
}

bool MotorController::setMotorMode(motorControlMode mode) // 0=Position Tracking, 1=Linear Interpolation
{
   /*gripper will always be in position control mode*/
   controller.command("PTH=1");

   /* set mode for 7 arm joints A through G */
   if(mode == MotorController::POS_CONTROL){ //position tracking		
		controller.command("PTA=1");
		controller.command("PTB=1");
		controller.command("PTC=1");
		controller.command("PTD=1");
		controller.command("PTE=1");
		controller.command("PTF=1");
		controller.command("PTG=1");
      motorMode = mode;
	}
   else if(mode == MotorController::LINEAR){ //linear control mode
		/* galil manual pg.88 (pdf pg.98) */
		controller.command("LM ABCDEFGH");	
      motorMode = mode;
	}
	else
		return 0;
	return 1;
}

bool MotorController::addLinearMotionSegment(vector<double> angles, vector<double> speeds)
{	
   if(angles.size() == 7 && speeds.size() == 7)
	{ 
      /*convert angles to position counts
      * also Galil will return an error if all positions sent are 0
      * so check if all positions are zero
      */
      vector<long> posCount(7);
      double zeroErrCheck = 0;
      for(int i = 0; i < 7 ; i++){
         int motorNum = i+1;
         posCount[i] = (long)angToEnc(motorNum,angles[i]);
         zeroErrCheck += abs(posCount[i]);
      }
      /* If all positions are zero, do not send command to galil controller*/
      if(zeroErrCheck < 1) return false; 
     
      //calculate vector speed
      vector<double> speedCount(7);
      for(int i = 0; i < 7 ; i++){
         int motorNum = i+1;
         speedCount[i] = angToEnc(motorNum,speeds[i]);
      }
      double vectorSpeed; /// this is for Galil controller
      vectorSpeed = (speedCount[0]*speedCount[0])+(speedCount[1]*speedCount[1])+(speedCount[2]*speedCount[2])+
         (speedCount[3]*speedCount[3])+(speedCount[4]*speedCount[4]) + (speedCount[5]*speedCount[5])
          + (speedCount[6]*speedCount[6]) ;
      vectorSpeed = std::sqrt(vectorSpeed);

      std::stringstream ss;
      ss << "LI " << posCount[0] << "," << posCount[1] << "," << posCount[2] << "," << posCount[3] << "," << posCount[4] << "," 
            <<  posCount[5] << "," << posCount[6] ;
      //add speed to the command string 
      ss << "<" << (int)vectorSpeed << ">" << (int)vectorSpeed;
		string command = ss.str();
		controller.command(command);
	}
	else{
		cerr << "addLinearMotionSegment takes 7 joint angles and speeds" << endl;
			throw std::out_of_range ("Angles, Speeds incorrect vector size. Should be 7");
	}
	return 1;
}

bool MotorController::beginLI()
{
	//controller.command("LE"); // Linear End, for smooth stopping
	controller.command("BGS"); // Begin Sequence
	return 1;
}

bool MotorController::endLIseq()
{
	controller.command("LE"); // Linear End, for smooth stopping
	//controller.command("BGS"); // Begin Sequence
	return 1;
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

   /*set all 7 arm joints to LI mode*/
   setMotorMode(LINEAR);

   /*set gripper to position control mode*/
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
	definePosition(6, (PI/3)); //joint 6 is now 60 degrees
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


double MotorController::readPos(int motorNum) // returns the current motor angle in radians
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
		if(motorNum != 9 && motorNum != 10){
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

double MotorController::readPosErr(int motorNum) // returns the error in  
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
		long encVal = angToEnc(motorNum,angle);
		string motor = motorLookup[motorNum];
		if (true){ //#debug  check if angle is in range
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

bool MotorController::motorsOn()
{
	controller.command("SH"); //turn on motors
	return true;
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