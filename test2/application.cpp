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

int main()
{
   WMRA::Pose dest;
   int velocity;				// Max velocity of the gripper in movement

   bool endFlag = 0;
   int choice;

   if(wmraArm.initialize()){
      cout << "Controller intialized in main" << endl;
      while(!endFlag){
         cout << "Select a destination" << endl;
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

         cout << "Start Movement? 1 = yes, 0 = no" << endl;
         cout << "Decision: ";
         cin >> choice;
         if(choice == 1){
            wmraArm.autonomous(dest,0);
         }
         else{
            endFlag = 1;
         }
      }
   }
   else
      cout << "Controller initalizaition failed in main" << endl;

   cout << "Ending Program" << endl;
}