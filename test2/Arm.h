#ifndef ARM_H
#define ARM_H

#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>
#include <vector>
#include <fstream>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <time.h>
#include <math.h>

#include "SockStream.h"
#include "tinythread.h"
#include "stringUtility.h"
#include "matrix.h" 
#include "MotorController.h"
#include "kinematics.h"
#include "trajectory.h"
#include "jacobian.h"
#include "Utility.h"
#include "ConfigReader.h"
#include "WmraTypes.h"
#include "MotorController.h"
#include "optimization.h"

using namespace tthread;

class Arm{
public:
	Arm();
	//WMRA::Pose getPosition();
	bool initialize();
	bool setDefaults();
	bool autonomous(WMRA::Pose dest, WMRA::CordFrame crodFr, bool blocking = true);
	bool openGripper(bool blocking = true);
	bool closeGripper(bool blocking = true);
    bool isGripperOpen();
	void closeDebug();
	bool toReady(bool blocking = true);
	bool ready2Park(bool blocking = true);
	bool park2Ready(bool blocking = true);
	bool moveJoint(int jointNum, double angle, int ref);
	WMRA::Pose getPose();
	WMRA::JointValueSet getJointAngles();
	void sendValues();
	bool isInitialized();
	bool setInitialJointAngles(WMRA::JointValueSet& joints);
	static void sendData(void* aArg);
	tthread::thread* t;

private:
	bool autonomousMove(Matrix start, Matrix dest);
	double dt;	// the default time between milestones
	double dt_mod;	// the default time between milestones
	double maxAngularVelocity;
	int control_velocity;
	
	Matrix gripperInitRotDiff;
	WMRA::JointValueSet readyPosition; //joint angles for ready position

	std::ofstream xyz_way; // Waypoint XYZ values
	std::ofstream xyz_sent; // command XYZ values
	std::ofstream xyz_cont; // command XYZ values
	std::ofstream jointVel; // joint velocity values
	bool initialized;
   bool gripperOpen;
	MotorController controller; 
};
#endif;