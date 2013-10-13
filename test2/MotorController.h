#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <exception>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <ctype.h>
//#include "Galil.h"
#include "WmraTypes.h"
#include "galilController.h"

using namespace std;


class MotorController {
public:
   enum motionMode  {POS_CONTROL = 0, LINEAR};
   MotorController();
   bool initialize();
   bool setMotorMode(int mode);
   /**
   * \brief adds the next linear segment. sends to motor controller
   * \@param [in] angles vector of joint angles in radians. 
   * \@param [in] section speed in rad/s^-1 
   */
   bool addLinearMotionSegment(vector<double> angles, vector<double> speeds);
   bool beginLI();/// \brief start linear motion
   /**
   * \brief  waits until all waypoints are finished. blocking.
   */
   bool waitLinearMotionEnd(); 
   bool isInitialized(); /// \brief return true if initialized
   bool wmraSetup();
   bool Stop(); /// \brief emergancy stop
   bool Stop(int motorNum); // emergancy stop a single motor
   double readPos(int motorNum); // returns the current motor angle in radians
   double readPosAll(); // returns the current motor angle in radians
   double readPosErr(int motorNum); // returns the error in 

   bool setMaxVelocity(int motorNum, float angularVelocity);
   bool setAccel(int motorNum, float angularAccelaration);
   bool setAccelAll(std::vector<int> acclVal);
   bool setDecel(int motorNum, float angularDecelaration);
   float encToAng(int motorNum, long enc);
   long angToEnc(int motorNum, float angle);
   bool positionControl(int motor,float angle);
   bool motorsOn(); // Turns motors on
   bool MotorsOFF();
   double rad2Enc[9];

private:

   bool initialized ;
   string ipAddr;
   double enc2Radian[9];
   //double rad2Enc[9];
   double normalize[8]; // doesn't apply to gripper
   static string motorLookup[9];
   //bool readSettings(); // read settings 
   bool definePosition(int motorNum, float angle);
   bool setPID(int motorNum, int P, int I, int D);
   bool isValidMotor(int motorNum);
};

#endif;