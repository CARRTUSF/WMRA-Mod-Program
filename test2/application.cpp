#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <Afxwin.h>
#include "matrix.h" 
#include "MotorController.h"
#include "kinematics.h"
#include "Arm.h"
#include "Utility.h"
//#include "autonomous.h"

#define PI 3.14159265

using namespace std;

/*
int wmraThreadRet = 1;		// return value from WMRA thread
int wmraCont = 1;			// return value from WMRA thread
int movement;				// start movement flag
double pitch, roll, yaw;	// pitch, roll, yaw orientation of the gripper
double dx, dy, dz;			// X, Y, Y destination of the gripper
int velocity;				// Max velocity of the gripper in movement
*/
char *tempChar; //temporary char pointer passed to thread

Arm wmraArm;
/*
UINT wmraThread (LPVOID pParam)
{
	cout << "Starting WMRA control thread" << endl;
	//thread for moving WMRA
	wmraThreadRet = wmraControl(); //calls main WMRA program
	//AfxEndThread(0);
	return 0;
}
*/
int main()
{
	double pitch, roll, yaw;	// pitch, roll, yaw orientation of the gripper
	double dx, dy, dz;			// X, Y, Y destination of the gripper
	int velocity;				// Max velocity of the gripper in movement

	bool endFlag = 0;
	int choice;

	if(wmraArm.initialize())
		cout << "Controller intialized in main" << endl;
	else
		cout << "Controller initalizaition failed in main" << endl;

	//AfxBeginThread (wmraThread, tempChar);
	
	while(!endFlag)
	{
		cout << "Select a destination" << endl;
		cout << "dx = ";
		cin >> dx;
		cout << "dy = ";
		cin >> dy;
		cout << "dz = ";
		cin >> dz;
		cout << "Pitch = ";
		cin >> pitch;
		cout << "Roll = ";
		cin >> roll;
		cout << "Yaw = ";
		cin >> yaw;

		cout << "Start Movement? 1 = yes, 0 = no" << endl;
		cout << "Decision: ";
		cin >> choice;
		if(choice == 1)
		{
			wmraArm.autonomous(dx,dy,dz,pitch,yaw,roll,0);
		}
		else
		{
			endFlag = 1;
		}
	}

	cout << "Ending Program" << endl;
}