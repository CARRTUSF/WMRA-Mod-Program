

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
#include "optimization.h"
#include "ConfigReader.h"

using namespace std;
using namespace math;

MotorController controller;
//MotorController Arm::control;


Arm::Arm(){
//    controller.initialize();
//	if(controller.isInitialized())
//		cout << "... Initialized ..." << endl;
//	else
//		cout << "Not Initialized ......" << endl;
//	initialized = 1;
}

bool Arm::initialize(){
	controller.initialize();
	if(!setDefaults())
		return 0;
	else if(controller.isInitialized())  // If controller class is initialized
	{
		initialized = 1;
		return 1;
	}
	else
		return 0;
}

bool Arm::setDefaults()
{
	ConfigReader reader;
	reader.parseFile("settings.conf");
	reader.setSection("WMRA_DEFAULTS");
	if(reader.keyPresent("dt"))
	{
		Arm::dt = reader.getDouble("dt");
		cout << "dt: " << Arm::dt << endl;
	}
	else
	{
		cout << "'dt' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_type"))
	{
		Arm::control_type = reader.getInt("control_type");
		cout << "control_type: " << Arm::control_type << endl;
	}
	else
	{
		cout << "'control_type' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_frame"))
	{
		Arm::control_frame = reader.getInt("control_frame");
		cout << "control_frame: " << Arm::control_frame << endl;
	}
	else
	{
		cout << "'control_frame' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_cart"))
	{
		Arm::control_cart = reader.getInt("control_cart");
		cout << "control_cart: " << Arm::control_cart << endl;
	}
	else
	{
		cout << "'control_cart' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_optim"))
	{
		Arm::control_optim = reader.getInt("control_optim");
		cout << "control_optim: " << Arm::control_optim << endl;
	}
	else
	{
		cout << "'control_optim' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_JLA"))
	{
		Arm::control_JLA = reader.getInt("control_JLA");
		cout << "control_JLA: " << Arm::control_JLA << endl;
	}
	else
	{
		cout << "'control_JLA' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_JLO"))
	{
		Arm::control_JLO = reader.getInt("control_JLO");
		cout << "control_JLO: " << Arm::control_JLO << endl;
	}
	else
	{
		cout << "'control_JLO' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_cont"))
	{
		Arm::control_cont = reader.getInt("control_cont");
		cout << "control_cont: " << Arm::control_cont << endl;
	}
	else
	{
		cout << "'control_cont' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_velocity"))
	{
		Arm::control_velocity = reader.getInt("control_velocity");
		cout << "control_velocity: " << Arm::control_velocity << endl;
	}
	else
	{
		cout << "'control_velocity' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_traj"))
	{
		Arm::control_traj = reader.getInt("control_traj");
		cout << "control_traj: " << Arm::control_traj << endl;
	}
	else
	{
		cout << "'control_traj' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("vr"))
	{
		Arm::vr = reader.getInt("vr");
		cout << "vr: " << Arm::vr << endl;
	}
	else
	{
		cout << "'vr' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("ml"))
	{
		Arm::ml = reader.getInt("ml");
		cout << "ml: " << Arm::ml << endl;
	}
	else
	{
		cout << "'ml' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_arm"))
	{
		Arm::control_arm = reader.getInt("control_arm");
		cout << "control_arm: " << Arm::control_arm << endl;
	}
	else
	{
		cout << "'control_arm' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("control_ini"))
	{
		Arm::control_ini = reader.getInt("control_ini");
		cout << "control_ini: " << Arm::control_ini << endl;
	}
	else
	{
		cout << "'control_ini' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("plt"))
	{
		Arm::plt = reader.getInt("plt");
		cout << "plt: " << Arm::plt << endl;
	}
	else
	{
		cout << "'plt' default not found" << endl;
		return 0;
	}
return 1;
}

bool Arm::moveArm(vector<double> destinationAng){ // destinationAng: the destination pose. dt: the amount of time to reach this pose from the current position.
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
				for(int i = 0; i < destFlag.size(); i++)
				{
					if(abs(currentAng[i]-milestoneAng[i]) < (0.01) && !destFlag[i])
					{
						destFlag[i] = 1;
						controller.positionControl(i+1, milestoneAng[i]);
					}
				}
				for(int i = 0; i < currentAng.size(); i++)
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

bool Arm::milestone(vector<double> currentAng, vector<double> destinationAng, double dt) { // sets the joints to move from current position to destination arm pose, in dt time.
//	vector<double> currentAng;//, milestoneAng;
//	vector<bool> destFlag;
	int dFLAG = 0;
	clock_t start_time = clock();
	Matrix main_pos_T(4,4), main_destination_T(4,4);
	double main_dist, thresh = 2, temp;
	if(controller.isInitialized())
	{
		double temp_vel, temp_acc, dq;
		long temp_enc;
		for(int i = 0; i < 7; i++)
		{
			dq = abs(currentAng[i]-destinationAng[i])/controller.rad2Enc[i+1];
			if (abs(dq)>25000)
			{
				controller.setMaxVelocity(i+1, controller.encToAng(i+1, 170000));
				controller.setAccel(i+1, controller.encToAng(i+1, 75000));
			}
			else if(dq < 100)
			{
				controller.setMaxVelocity(i+1, controller.encToAng(i+1, 170000));
				controller.setAccel(i+1, controller.encToAng(i+1, 75000));
			}
			else
			{
				temp_vel=2*abs(dq)/0.3;
				temp_enc = temp_vel;
				controller.setMaxVelocity(i+1, controller.encToAng(i+1, temp_enc));
				temp_acc=2*abs(dq)/0.7;
				temp_enc = temp_acc;
				controller.setAccel(i+1, controller.encToAng(i+1, temp_enc));
			}
		}

		for(int i = 0; i < destinationAng.size(); i++)
		{
			//double temp;
			//currentAng.push_back(controller.readPos(i+1));
			//temp = (abs(destinationAng[i]-currentAng[i])/dt);
			//controller.setMaxVelocity(i+1, temp);
			controller.positionControl(i+1, (destinationAng[i])); //  + (destinationAng[i]-currentAng[i])));
			//destFlag.push_back(0);
			//milestoneAng.push_back(0);
		}	
/*
		while(dFLAG == 0)
		{
			for(int i = 0; i < destinationAng.size(); i++)
			{
				temp = abs(destinationAng[i]-controller.readPos(i+1));
				if(temp < thresh)
					destFlag[i] = 1;
			}
			dFLAG = 1;
			for(int i = 0; i < destFlag.size(); i++)
			{
				if(destFlag[i] == 0)
				{
					dFLAG = 0;
				}
			}
		}
*/
		
		
/*
		main_pos_T = kinematics(currentAng);
		main_destination_T = kinematics(destinationAng);
		
		// Finding the distance from the current gripper pose to the destination gripper pose.
		main_dist = sqrt((pow((main_pos_T(0,3)-main_destination_T(0,3)),2) + pow((main_pos_T(1,3)-main_destination_T(1,3)),2) + pow((main_pos_T(2,3)-main_destination_T(2,3)),2)));


		for(int i = 0; i < destFlag.size(); i++)
		{
			controller.positionControl(i+1, destinationAng[i]);
		}

		while (main_dist > thresh)
		{
			for(int i = 0; i < destFlag.size(); i++)
			{
				if(abs(currentAng[i]-destinationAng[i]) < (0.01) && !destFlag[i])
				{
					destFlag[i] = 1;
					controller.positionControl(i+1, destinationAng[i]);
				}
			}
			for(int i = 0; i < currentAng.size(); i++)
			{
				currentAng[i] = controller.readPos(i+1);
			}			
			main_pos_T = kinematics(currentAng);
			main_dist = sqrt((pow((main_pos_T(0,3)-main_destination_T(0,3)),2) + pow((main_pos_T(1,3)-main_destination_T(1,3)),2) + pow((main_pos_T(2,3)-main_destination_T(2,3)),2)));
		}
*/
	}
	else
	{
		cout << "ERROR: controller not initialized (Arm.cpp)" << endl;
		return 0;
	}
	//while(((clock()-start_time)/CLOCKS_PER_SEC) > dt)
	//{}
	return 1; // Movement complete
}


/*******************************
// type: 1=Arm, 2=Wheelchair, 3=Both
// dx: distance on X axis destination is from the current location
// dy: distance on Y axis destination is from the current location
// dz: distance on Z axis destination is from the current location
// pitch: change in pitch from current location
// yaw: change in yaw from current location
// roll: change in roll from current location
*******************************/
//bool Arm::autonomous(WMRA::Pose destPose)
bool Arm::autonomous(int type, int dx, int dy, int dz, double pitch, double yaw, double roll) 
{


	// Unknown (not sure if these veriables are used)
	double totalTime, distance, detJoa;
	int n;
	vector<double> currentLoc, destinationLoc, delta;
	Matrix currentLoc_T(4,4), destination_T(4,4), destination_Rotation_T(4,4), temp_rotation(4,4), Ta(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T67(4,4);
	Matrix Joa(6,7), Ttnew(4,4), jointAng_Mat(7,1);
	float ***Tt; // pointer to a 3D array containing 'n' 4x4 Transformation matricies

	delta.resize(6);
	currentLoc.resize(8);
	destinationLoc.resize(8);
	
	cout << "Thread Started, Checking WMRA Controller Initialization" << endl;
	if(controller.isInitialized())	// If WMRA controller connection has been initialized start loop
	{
		cout << "WMRA-2 Controller Initialized" << endl;
		for(int i = 0; i < currentLoc.size(); i++)		// Sets the current location to a 1x8 vector
		{
			currentLoc[i] = controller.readPos(i+1);
		}
		currentLoc_T = kinematics(currentLoc,Ta,T01,T12,T23,T34,T45,T56,T67);
		//cout << "Movement Initialization Sequence" << endl;
		// Sets the current location to a 1x8 vector
		for(int i = 0; i < currentLoc.size(); i++)		
		{
			currentLoc[i] = controller.readPos(i+1);
		}
		
		currentLoc_T = kinematics(currentLoc,Ta,T01,T12,T23,T34,T45,T56,T67);
		// Destination transformation matrix using input angles
		temp_rotation = WMRA_rotz(pitch)*WMRA_roty(yaw)*WMRA_rotx(roll);
		destination_Rotation_T = currentLoc_T*temp_rotation;

		// Destination Transformation Matrix Td [4x4]
		destination_T(0,0) = destination_Rotation_T(0,0);	destination_T(0,1) = destination_Rotation_T(0,1);	destination_T(0,2) = destination_Rotation_T(0,2);	destination_T(0,3) = currentLoc_T(0,3) + dx;
		destination_T(1,0) = destination_Rotation_T(1,0);	destination_T(1,1) = destination_Rotation_T(1,1);	destination_T(1,2) = destination_Rotation_T(1,2);	destination_T(1,3) = currentLoc_T(1,3) + dy;
		destination_T(2,0) = destination_Rotation_T(2,0);	destination_T(2,1) = destination_Rotation_T(2,1);	destination_T(2,2) = destination_Rotation_T(2,2);	destination_T(2,3) = currentLoc_T(2,3) + dz;
		destination_T(3,0) = currentLoc_T(3,0);				destination_T(3,1) = currentLoc_T(3,1);				destination_T(3,2) = currentLoc_T(3,2);				destination_T(3,3) = currentLoc_T(3,3);

		distance = sqrt(pow(destination_T(0,3)-currentLoc_T(0,3),2) + pow(destination_T(1,3)-currentLoc_T(1,3),2) + pow(destination_T(2,3)-currentLoc_T(2,3),2));

		totalTime = distance/Arm::control_velocity;
		n = ceil(totalTime/Arm::dt); // Number of iterations rounded up.
		Arm::dt = totalTime/n;

		vector<Matrix> wayPoints = WMRA_traj(3,currentLoc_T,destination_T,n+1);
      //Tt=WMRA_traj(3,currentLoc_T,destination_T,n+1); // Generating all the transformation matricies for each milestone(n).
		//cout << "Trajectory initialized" << endl;
		cout << "n = " << n << endl;
		// Main movement loop where each 4x4 milestone matrix is converted into jacobian angles for the 7 arm joints
		for(int cur_milestone = 0; cur_milestone < n; cur_milestone++)
		{
			// Sets the current location to a 1x8 vector
			for(int i = 0; i < currentLoc.size(); i++)		
			{
				currentLoc[i] = controller.readPos(i+1);
			}
			currentLoc_T = kinematics(currentLoc,Ta,T01,T12,T23,T34,T45,T56,T67);
			
			Ttnew.Null(4,4);
         ttnew = wayPoints[cur_milestone]
			for (int j=0; j<4; j++){
				for (int k=0; k<4; k++){
					Ttnew(j,k)=Tt[cur_milestone][j][k];	        			
				}
			}

			// Calculating the 6X7 Jacobian of the arm in frame 0:
			WMRA_J07(T01, T12, T23, T34, T45, T56, T67, Joa, detJoa);
			WMRA_delta(delta, currentLoc_T , Ttnew);
			jointAng_Mat = WMRA_Opt(Joa, detJoa, delta, currentLoc);
			
			for(int i = 0; i < 7; i++)
			{
				destinationLoc[i] = currentLoc[i] + jointAng_Mat(i,0);
			}
			
			//cout << "Moving to Milestone " << cur_milestone << endl;
			Arm::milestone(currentLoc, destinationLoc, Arm::dt);
			Sleep(1000*Arm::dt);
		}
	}
	else
		return 0;
	return 1;
}