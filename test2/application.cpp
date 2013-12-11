
#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <Afxwin.h>
#include "matrix.h" 
#include "MotorController.h"
#include "Arm.h"
#include "Utility.h"


#define PI 3.14159265

using namespace std;

char *tempChar; //temporary char pointer passed to thread
Arm wmraArm;


WMRA::Pose getUserDest(){
   WMRA::Pose dest;
   double temp;
   cout << "\n\nSelect a destination" << endl;
   cout << "dx = ";
   cin >> dest.x;
   cout << "dy = ";
   cin >> dest.y;
   cout << "dz = ";
   cin >> dest.z;
   cout << "Pitch = ";
   cin >> temp;
   dest.pitch = degToRad(temp);;
   cout << "Roll = ";
   cin >> temp;
   dest.roll = degToRad(temp);
   cout << "Yaw = ";
   cin >> temp;
   dest.yaw = degToRad(temp);
   cout << endl;

   return dest;    
}



int main()
{
   int velocity;				// Max velocity of the gripper in movement
   bool endFlag = false;
   if(wmraArm.initialize()){
      //cout << "Controller intialized in main" << endl;
      WMRA::Pose readyPose = wmraArm.getPose();
      int cordframe;
      double temp;
      int option;

	  WMRA::Pose cupInfront, cup;

	  cupInfront.x = 650;
	  cupInfront.y = -100;
	  cupInfront.z = 475;
	  cupInfront.pitch = 0;
	  cupInfront.roll = 0;
	  cupInfront.yaw = 0;

	  cup.x = 850;
	  cup.y = -100;
	  cup.z = 450;
	  cup.pitch = 0;
	  cup.roll = 0;
	  cup.yaw = degToRad(-10);


	  cout << "go to park? 1=yes, 0=no" << endl;
	  cin >> endFlag;
	  if(endFlag)
	  {
		 wmraArm.ready2Park();
	  }

	  cout << "Ready for demo? 1=yes, 0=no" << endl;
	  cin >> endFlag;
	  if(endFlag)
	  {
		  wmraArm.park2Ready();
		  wmraArm.openGripper();
		  wmraArm.autonomous2(cupInfront, WMRA::ARM_FRAME_PILOT_MODE);
		  Sleep(6000);
		  wmraArm.autonomous2(cup, WMRA::ARM_FRAME_PILOT_MODE);
		  Sleep(3000);
		  wmraArm.closeGripper();
		  wmraArm.autonomous2(cupInfront, WMRA::ARM_FRAME_PILOT_MODE);
		  Sleep(4000);
		  wmraArm.autonomous2(readyPose, WMRA::ARM_FRAME_PILOT_MODE);
		  Sleep(8000);
		  wmraArm.autonomous2(cupInfront, WMRA::ARM_FRAME_PILOT_MODE);
		  Sleep(8000);
		  wmraArm.autonomous2(cup, WMRA::ARM_FRAME_PILOT_MODE);
		  Sleep(4000);
		  wmraArm.openGripper();
		  wmraArm.autonomous2(cupInfront, WMRA::ARM_FRAME_PILOT_MODE);
		  Sleep(4000);
		  wmraArm.autonomous2(readyPose, WMRA::ARM_FRAME_PILOT_MODE);
		  Sleep(8000);

      } //end of if statement

      cout << "About to exit program. Would you like to go to ready position? 1=Yes 0=No : " ;
      cin >> option;
      if(option ==1){
		  //wmraArm.autonomous2(readyPose, WMRA::ARM_FRAME_ABS);
		  wmraArm.toReady();
         Sleep(10000);
      }
     
   }
   else{
      cout << "Controller initalizaition failed in main" << endl;
   }

    cout << "Exiting Program..... Press any key to close this window." << endl;
    cin.get();
}