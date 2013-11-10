
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <math.h>
#include <windows.h>
#include "stringUtility.h"
#include "Arm.h"
#include "matrix.h" 
#include "MotorController.h"
#include "kinematics.h"
#include "trajectory.h"
#include "jacobian.h"
#include "Utility.h"
#include "ConfigReader.h"

using namespace std;
using namespace math;

Arm::Arm(){
	xyz_way.open("data/XYZ-way.csv");
	xyz_sent.open("data/XYZ-sent.csv");
	xyz_cont.open("data/XYZ-cont.csv");
}

bool Arm::initialize(){
	controller.initialize();
	if(!setDefaults())
		return 0;
	else if(controller.isInitialized())  // If controller class is initialized
	{
		initialized = 1;
		readyPosition = getJointAngles();
		return 1;
	}
	else
		return 0;
}

WMRA::JointValueSet Arm::getJointAngles(){
	WMRA::JointValueSet joints;
	for(int i = 0; i < joints.size(); i++){		// Sets the current location to a 1x8 vector		
		joints[i] = controller.readPos(i+1);
	}
	return joints;   
}

//WMRA::Pose Arm::getPosition(){
//   if(controller.isInitialized()){
//   Matrix currLoc_T(4,4);
//   vector<double> JointAng(7); // Joint angles for the 7 joints
//   for(int i = 0; i < startJointAng.size(); i++){		// Sets the current location to a 1x8 vector		
//			JointAng[i] = controller.readPos(i+1);
//		}
//		currLoc_T = kinematics(JointAng);
//   }
//}

/*******************************
// type: 1=Arm, 2=Wheelchair, 3=Both
// dx: distance on X axis destination is from the current location
// dy: distance on Y axis destination is from the current location
// dz: distance on Z axis destination is from the current location
// pitch: change in pitch from current location
// yaw: change in yaw from current location
// roll: change in roll from current location
*******************************/
bool Arm::autonomous(WMRA::Pose dest, WMRA::CordFrame crodFr)
//bool Arm::autonomous( int dx, int dy, int dz, double pitch, double yaw, double roll, int type) 
{
	// DEBUG - Erase test matrix
	Matrix test_T(4,4), debugPos_T(4,4);
	vector<double> debugPos(8);

	// VARIABLES
	vector<double> startJointAng(7), prevJointAng(7), currJointAng(7), delta(8), speeds(7);
	double totalTime, distance, detJoa;
	int numWayPoints;

	// Transformation variables
	Matrix Ta(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T67(4,4);
	Matrix startLoc_T(4,4), temp_dest(4,4), destination_T(4,4), destination_Rotation_T(4,4), currentLoc_T(4,4), prevPosTF(4,4), currPosTF(4,4);
	Matrix Joa(6,7), jointAng_Mat(7,1);

	// A vector of all waypoints, transformation matrix
	vector<Matrix> wayPoints;

	Arm::dt_mod = Arm::dt;

	// STARTING AUTONOMOUS MOVEMENT
	//cout << "Thread Started, Checking WMRA Controller Initialization" << endl;
	if(controller.isInitialized())	// If WMRA controller connection has been initialized start loop
	{
		//cout << "WMRA-2 Controller Initialized" << endl;
		startJointAng = controller.readPosAll(); // Sets the current location to a 1x8 vector  

		startLoc_T = kinematics(startJointAng,Ta,T01,T12,T23,T34,T45,T56,T67);

		// Destination transformation matrix using input angles
		temp_dest = WMRA_rotz(dest.pitch)*WMRA_roty(dest.yaw)*WMRA_rotx(dest.roll);
		destination_Rotation_T = startLoc_T * temp_dest;

		//Destination Transformation Matrix Td [4x4]
		destination_T(0,0) = destination_Rotation_T(0,0);	destination_T(0,1) = destination_Rotation_T(0,1);	destination_T(0,2) = destination_Rotation_T(0,2);	destination_T(0,3) = startLoc_T(0,3) + dest.x;
		destination_T(1,0) = destination_Rotation_T(1,0);	destination_T(1,1) = destination_Rotation_T(1,1);	destination_T(1,2) = destination_Rotation_T(1,2);	destination_T(1,3) = startLoc_T(1,3) + dest.y;
		destination_T(2,0) = destination_Rotation_T(2,0);	destination_T(2,1) = destination_Rotation_T(2,1);	destination_T(2,2) = destination_Rotation_T(2,2);	destination_T(2,3) = startLoc_T(2,3) + dest.z;
		destination_T(3,0) = 0;								destination_T(3,1) = 0;								destination_T(3,2) = 0;								destination_T(3,3) = 1;

		distance = sqrt(pow(destination_T(0,3)-startLoc_T(0,3),2) + pow(destination_T(1,3)-startLoc_T(1,3),2) + pow(destination_T(2,3)-startLoc_T(2,3),2));

		// Checking if translation distance is 0. If it is rotation distance is calculated
		if(distance == 0.0)
		{
			distance = sqrt(pow(dest.pitch, 2) + pow(dest.yaw, 2) + pow(dest.roll,2));
		}
		if(distance == 0.0) // if no translation or rotation change then movement is aborted.
		{
			cout << "No Movement Selected" << endl;
			return 0;	
		}

		// Determine the total time of movement, the number of waypoints, and the move time of each waypoint.
		totalTime = distance/Arm::control_velocity;
		numWayPoints = ceil(totalTime/Arm::dt); // Number of iterations rounded up.
		Arm::dt_mod = totalTime/numWayPoints;

		wayPoints = WMRA_traj(3, startLoc_T, destination_T, numWayPoints+1);

		// Main movement loop where each 4x4 milestone matrix is converted into jacobian angles for the 7 arm joints
		prevJointAng = startJointAng;
		prevPosTF =  startLoc_T;

		xyz_sent << startLoc_T(0,3) << "," << startLoc_T(1,3) << "," << startLoc_T(2,3) << endl;
		xyz_way << startLoc_T(0,3) << "," << startLoc_T(1,3) << "," << startLoc_T(2,3) << endl;

		for(int i = 1 ; i < numWayPoints +1; i++)
		{			
			currentLoc_T = kinematics(prevJointAng,Ta,T01,T12,T23,T34,T45,T56,T67);	

			// Calculating the 6X7 Jacobian of the arm in frame 0:
			WMRA_J07(T01, T12, T23, T34, T45, T56, T67, Joa, detJoa);

			//#debug need to transform waypoints to arm base see matlab code WMRA_main.m line 346
			currPosTF = wayPoints[i];
			xyz_way << currPosTF(0,3) << "," << currPosTF(1,3) << "," << currPosTF(2,3) << endl;

			WMRA_delta(delta, prevPosTF , currPosTF);

			jointAng_Mat = opt.WMRA_Opt(Joa, detJoa, delta, prevJointAng);

			for(int j = 0; j < 7; j++){
				currJointAng[j] = jointAng_Mat(j,0);
				prevJointAng[j] += currJointAng[j];
			}

			test_T = kinematics(prevJointAng,Ta,T01,T12,T23,T34,T45,T56,T67);	
			xyz_sent << test_T(0,3) << "," << test_T(1,3) << "," << test_T(2,3) << endl;

			for(int k = 0; k < currJointAng.size(); k++){
				speeds[k] = abs(currJointAng[k])/Arm::dt_mod;
			}
			controller.addLinearMotionSegment(currJointAng, speeds);

			if(controller.isDebug())
			{
				for(int j = 0; j < delta.size(); j++)	cout << delta[j] ;
				cout << endl;
				cout << wayPoints[i] << endl;
				cout << currPosTF << endl;
				cout << "Speeds: " << speeds[0];
				for(int k = 1; k < currJointAng.size(); k++)
					cout << ", " << speeds[k];
				cout << endl;
			}

			prevPosTF = currPosTF;
		}

		debugPos = controller.readPosAll();
		debugPos_T = kinematics(debugPos);
		xyz_cont << debugPos_T(0,3) << "," << debugPos_T(1,3) << "," << debugPos_T(2,3) << endl;

		controller.beginLI();
		controller.endLIseq();
		if(controller.isDebug())
		{
			for(int k = 0; k < (numWayPoints+10); k++)
			{
				debugPos = controller.readPosAll();
				debugPos_T = kinematics(debugPos);
				xyz_cont << debugPos_T(0,3) << "," << debugPos_T(1,3) << "," << debugPos_T(2,3) << endl;
				//cout << debugPos[0] << "," << debugPos[1] << "," << debugPos[2] << "," << debugPos[3] << "," << debugPos[4] << "," << debugPos[5] << "," << debugPos[6] << "," << debugPos[7] << endl;
				Sleep(1000*Arm::dt_mod);
			}


			cout << "start is " << endl;
			cout << startLoc_T << endl;

			cout << "destination is :" << endl;
			cout << destination_T << endl;

			cout << "\t\tDISTANCE: " << distance << endl;

			cout << "number of way points = " << numWayPoints << endl;

			//#debug - output end position after the loop
			cout << "last waypoint is :" << endl;
			cout <<  wayPoints[numWayPoints]<< endl;

			cout << "last waypoint ik->fwK is :" << endl;
			currPosTF = kinematics(prevJointAng);
			cout << currPosTF << endl;

			Sleep(10000);
			/*		for(int i = 0; i < startJointAng.size(); i++){		// Sets the current location to a 1x8 vector		
			startJointAng[i] = controller.readPos(i+1);
			}
			startLoc_T = kinematics(startJointAng);
			cout << "arm reached position is " << endl;
			cout << startLoc_T << endl;
			*/
			cout << "and the same thing AGAIN.... " << endl;
			for(int i = 0; i < startJointAng.size(); i++){		// Sets the current location to a 1x8 vector		
				startJointAng[i] = controller.readPos(i+1);
			}
			currPosTF = kinematics(startJointAng);
			cout << "arm reached position is " << endl;
			cout << currPosTF << endl;
			cout << endl;

			distance = sqrt(pow(currPosTF(0,3)-startLoc_T(0,3),2) + pow(currPosTF(1,3)-startLoc_T(1,3),2) + pow(currPosTF(2,3)-startLoc_T(2,3),2));

			cout << "\t\tDISTANCE: " << distance << endl;

			//#debug end
		}
	}

	else
		return 0;
	return 1;
}

/*
bool Arm::milestoneDelta(vector<double> destinationAng, double dt_mod) { // sets the joints to move from current position to destination arm pose, in dt_mod time.
vector<double> speeds;
if(controller.isInitialized()){
double vel;
for(int i = 0; i < destinationAng.size(); i++){
vel = abs(destinationAng[i])/dt_mod;
speeds.push_back(vel);
}
controller.addLinearMotionSegment(destinationAng, speeds);
return true; // Movement complete
}
else{
cout << "ERROR: controller not initialized (Arm.cpp)" << endl;
return false;
}	
}
*/

bool Arm::moveArm(vector<double> destinationAng){ // destinationAng: the destination pose. dt_mod: the amount of time to reach this pose from the current position.
	vector<double> currentAng, milestoneAng, moveStep;
	vector<bool> destFlag;
	Matrix main_pos_T(4,4), main_destination_T(4,4);
	double main_dist, main_time, main_velocity = 10, thresh = 10;
	int n;
	if(controller.isInitialized())
	{
		for(int i = 0; i < destinationAng.size(); i++)
		{
			currentAng.push_back(controller.readPos(i+1));
			moveStep.push_back(0);
			destFlag.push_back(0);
			milestoneAng.push_back(0);
		}	

		double maxVel;
		for(int i=0; i < currentAng.size(); i++){
			maxVel = abs(destinationAng[i]-currentAng[i]) / Arm::dt;
			controller.setMaxVelocity(i+1,maxVel);
		}

		main_pos_T = kinematics(currentAng);
		main_destination_T = kinematics(destinationAng);

		// Finding the distance from the current gripper pose to the destination gripper pose.
		main_dist = sqrt((pow((main_pos_T(0,3)-main_destination_T(0,3)),2) + pow((main_pos_T(1,3)-main_destination_T(1,3)),2) + pow((main_pos_T(2,3)-main_destination_T(2,3)),2)));

		// Determining the time needed for the set gripper velocity, and the number of milestones (n) needed.
		main_time = main_dist/main_velocity;
		n = ceil(main_time/Arm::dt);
		Arm::dt = main_time/n; // Recomputing dt.

		for(int i = 0; i < moveStep.size(); i++)
		{
			if(n != 0)
				moveStep[i] = (destinationAng[i]-currentAng[i])/n;
			else
				moveStep[i] = 0;
		}

		for(int j = 0; j < n; j++)
		{
			for(int i = 0; i < currentAng.size(); i++)
			{
				currentAng[i] = currentAng[i] + moveStep[i];
				controller.positionControl(i+1, currentAng[i]);
				destFlag[i] = 0;
				milestoneAng[i] = currentAng[i];
			}

			main_destination_T = kinematics(milestoneAng); // acctually destination angle (above line)
			for(int i = 0; i < currentAng.size(); i++)
			{
				currentAng[i] = controller.readPos(i+1);
			}			
			main_pos_T = kinematics(currentAng);

			// Finding the distance from the current gripper pose to the destination gripper pose.
			main_dist = sqrt((pow((main_pos_T(0,3)-main_destination_T(0,3)),2) + pow((main_pos_T(1,3)-main_destination_T(1,3)),2) + pow((main_pos_T(2,3)-main_destination_T(2,3)),2)));


			while (main_dist > thresh)
			{
				for(size_t i = 0; i < destFlag.size(); i++)
				{
					if(abs(currentAng[i]-milestoneAng[i]) < (0.01) && !destFlag[i])
					{
						destFlag[i] = 1;
						controller.positionControl(i+1, milestoneAng[i]);
					}
				}
				for(size_t i = 0; i < currentAng.size(); i++)
				{
					currentAng[i] = controller.readPos(i+1);
				}			
				main_pos_T = kinematics(currentAng);
				main_dist = sqrt((pow((main_pos_T(0,3)-main_destination_T(0,3)),2) + pow((main_pos_T(1,3)-main_destination_T(1,3)),2) + pow((main_pos_T(2,3)-main_destination_T(2,3)),2)));
			}
		}
	}
	else
	{
		cout << "ERROR: controller not initialized (Arm.cpp)" << endl;
		return 0;
	}
	return 1; // Movement complete
}

void Arm::closeDebug(){
	xyz_way.close();
	xyz_sent.close();
	xyz_cont.close();
}

bool Arm::toReady()
{
	vector<double> readyAng;
	vector<double> speeds;
	readyAng.push_back(1.570796327-controller.readPos(0));
	readyAng.push_back(1.570796327-controller.readPos(1));
	readyAng.push_back(0.0-controller.readPos(2));
	readyAng.push_back(1.570796327-controller.readPos(3));
	readyAng.push_back(1.570796327-controller.readPos(4));
	readyAng.push_back(1.0471975513-controller.readPos(5));
	readyAng.push_back(0.0-controller.readPos(6));
	readyAng.push_back(0.0-controller.readPos(7));
	speeds.push_back(0.01);
	speeds.push_back(0.01);
	speeds.push_back(0.01);
	speeds.push_back(0.01);
	speeds.push_back(0.01);
	speeds.push_back(0.01);
	speeds.push_back(0.01);
	speeds.push_back(0.01);
	controller.addLinearMotionSegment(readyAng, speeds);
	controller.beginLI();
	controller.endLIseq();
	return true;
}

bool Arm::setDefaults()
{
	ConfigReader reader;
	reader.parseFile("settings.conf");
	reader.setSection("WMRA_DEFAULTS");
	if(reader.keyPresent("dt"))
	{
		Arm::dt = reader.getDouble("dt");
		//cout << "dt: " << Arm::dt << endl;
	}
	else
	{
		cout << "'dt' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_type"))
	{
		Arm::control_type = reader.getInt("control_type");
		//cout << "control_type: " << Arm::control_type << endl;
	}
	else
	{
		cout << "'control_type' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_frame"))
	{
		Arm::control_frame = reader.getInt("control_frame");
		//cout << "control_frame: " << Arm::control_frame << endl;
	}
	else
	{
		cout << "'control_frame' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_cart"))
	{
		Arm::control_cart = reader.getInt("control_cart");
		//cout << "control_cart: " << Arm::control_cart << endl;
	}
	else
	{
		cout << "'control_cart' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_optim"))
	{
		Arm::control_optim = reader.getInt("control_optim");
		//cout << "control_optim: " << Arm::control_optim << endl;
	}
	else
	{
		cout << "'control_optim' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_JLA"))
	{
		Arm::control_JLA = reader.getInt("control_JLA");
		//cout << "control_JLA: " << Arm::control_JLA << endl;
	}
	else
	{
		cout << "'control_JLA' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_JLO"))
	{
		Arm::control_JLO = reader.getInt("control_JLO");
		//cout << "control_JLO: " << Arm::control_JLO << endl;
	}
	else
	{
		cout << "'control_JLO' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_cont"))
	{
		Arm::control_cont = reader.getInt("control_cont");
		//cout << "control_cont: " << Arm::control_cont << endl;
	}
	else
	{
		cout << "'control_cont' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_velocity"))
	{
		Arm::control_velocity = reader.getInt("control_velocity");
		//cout << "control_velocity: " << Arm::control_velocity << endl;
	}
	else
	{
		cout << "'control_velocity' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_traj"))
	{
		Arm::control_traj = reader.getInt("control_traj");
		//cout << "control_traj: " << Arm::control_traj << endl;
	}
	else
	{
		cout << "'control_traj' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("vr"))
	{
		Arm::vr = reader.getInt("vr");
		//cout << "vr: " << Arm::vr << endl;
	}
	else
	{
		cout << "'vr' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("ml"))
	{
		Arm::ml = reader.getInt("ml");
		//cout << "ml: " << Arm::ml << endl;
	}
	else
	{
		cout << "'ml' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_arm"))
	{
		Arm::control_arm = reader.getInt("control_arm");
		//cout << "control_arm: " << Arm::control_arm << endl;
	}
	else
	{
		cout << "'control_arm' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_ini"))
	{
		Arm::control_ini = reader.getInt("control_ini");
		//cout << "control_ini: " << Arm::control_ini << endl;
	}
	else
	{
		cout << "'control_ini' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("plt"))
	{
		Arm::plt = reader.getInt("plt");
		//cout << "plt: " << Arm::plt << endl;
	}
	else
	{
		cout << "'plt' default not found" << endl;
		return 0;
	}
	return 1;
}
