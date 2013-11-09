
#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <Afxwin.h>
#include "matrix.h" 
#include "MotorController.h"
#include "Arm.h"
#include "Utility.h"
//#include "autonomous.h"

#define PI 3.14159265

using namespace std;

char *tempChar; //temporary char pointer passed to thread
Arm wmraArm;

int main()
{
	WMRA::Pose dest;
	int velocity;				// Max velocity of the gripper in movement

	bool endFlag = false;
	int choice;

	if(wmraArm.initialize()){
		//cout << "Controller intialized in main" << endl;
		while(!endFlag){
			cout << "\n\nSelect a destination" << endl;
			cout << "dx = ";
			cin >> dest.x;
			cout << "dy = ";
			cin >> dest.y;
			cout << "dz = ";
			cin >> dest.z;
			cout << "Pitch = ";
			cin >> dest.pitch;
			cout << "Roll = ";
			cin >> dest.roll;
			cout << "Yaw = ";
			cin >> dest.yaw;
			cout << endl;

			if(dest.yaw == 777)
				dest.yaw == dest.yaw;// No-Op, will loop through choices again
			else if(dest.yaw == 999)
			{
				wmraArm.closeDebug();
				endFlag = true; // Will break out of loop
			}
			else
				wmraArm.autonomous(dest,WMRA::ARM_FRAME); // Moves arm
		}
	}
	else
		cout << "Controller initalizaition failed in main" << endl;
	
	cout << "Ending Program" << endl;
}