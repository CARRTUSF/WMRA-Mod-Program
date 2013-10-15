
#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <windows.h>
#include "matrix.h" 
#include "MotorController.h"
#include "kinematics.h"
#include "Arm.h"
#include "Utility.h"
#include "jacobian.h"
#include "optimization.h"

#define PI 3.14159265

using namespace std;


extern int wmraCont; //return value from WMRA thread
extern int movement; //start movement flag
extern double pitch, roll, yaw;
extern double dx, dy, dz;
extern int velocity;


extern Arm control;

int wmraControl()
{
	double dt = 0.05, totalTime, distance, detJoa;
	int n;
	vector<double> currentLoc, destinationLoc, delta;
	Matrix currentLoc_T(4,4), destination_T(4,4), destination_Rotation_T(4,4), temp_rotation(4,4), Ta(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T67(4,4);
	Matrix Joa(6,7), Ttnew(4,4), jointAng_Mat(7,1);
	float ***Tt; // pointer to a 3D array containing 'n' 4x4 Transformation matricies

	delta.resize(6);
	currentLoc.resize(8);
	destinationLoc.resize(8);

	cout << "Thread Started, Checking WMRA Controller Initialization" << endl;
	if(control.initialize())	// If WMRA controller connection has been initialized start loop
	{
		cout << "WMRA-2 Controller Initialized" << endl;
		for(int i = 0; i < currentLoc.size(); i++)		// Sets the current location to a 1x8 vector
		{
			currentLoc[i] = control.controller.readPos(i+1);
		}
		currentLoc_T = kinematics(currentLoc,Ta,T01,T12,T23,T34,T45,T56,T67);
		while(wmraCont)			// wmraCont is initially set to 1, if set to 0 externally will end autonomous control
		{
			if(movement)		// movement is set externally to tell the function that it is ready for movement
			{
				//cout << "Movement Initialization Sequence" << endl;
				// Sets the current location to a 1x8 vector
				for(int i = 0; i < currentLoc.size(); i++)		
				{
					currentLoc[i] = control.controller.readPos(i+1);
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

				totalTime = distance/velocity;
				n = ceil(totalTime/dt); // Number of iterations rounded up.
				dt = totalTime/n;

				Tt=WMRA_traj(3,currentLoc_T,destination_T,n+1); // Generating all the transformation matricies for each milestone(n).
				//cout << "Trajectory initialized" << endl;
				cout << "n = " << n << endl;
				// Main movement loop where each 4x4 milestone matrix is converted into jacobian angles for the 7 arm joints
				for(int cur_milestone = 0; cur_milestone < n; cur_milestone++)
				{
					// Sets the current location to a 1x8 vector
					for(int i = 0; i < currentLoc.size(); i++)		
					{
						currentLoc[i] = control.controller.readPos(i+1);
					}
					currentLoc_T = kinematics(currentLoc,Ta,T01,T12,T23,T34,T45,T56,T67);
					
					Ttnew.Null(4,4);
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
					control.milestone(currentLoc, destinationLoc, dt);
					Sleep(1000*dt);
				}
				movement = 0;
			}
		}
	}
	else
		return 0;
	return 1;
}