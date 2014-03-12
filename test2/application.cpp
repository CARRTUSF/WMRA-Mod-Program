
#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include "matrix.h" 
#include "MotorController.h"
#include "Arm.h"
#include "Utility.h"

#define PI 3.14159265

using namespace std;

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
   dest.pitch = degToRad(temp);
   cout << "Roll = ";
   cin >> temp;
   dest.roll = degToRad(temp);
   cout << "Yaw = ";
   cin >> temp;
   dest.yaw = degToRad(temp);
   cout << endl;

   return dest;    
}


void continuousSquare(WMRA::Pose curPos)
{
	//Request to begin test
	int length;
	cout << "Length? " << endl;
	cin >> length;

	int loopCount;
	cout << "Number of loops? " << endl;
	cin >> loopCount;

	int choice = 0;
	cout << "Begin Square Test? 1=Yes 0=No" << endl;
	cin >> choice;

	WMRA::Pose dest1,dest2,dest3,dest4;
	dest1 = dest2 = dest3 = dest4 = curPos;	
	
	dest2.x = dest1.x;
	dest2.y = dest1.y+length;

	dest3.x = dest2.x+length;
	dest3.y = dest2.y;
	
	dest4.x = dest3.x;
	dest4.y = dest3.y-length;

	// Move arm to starting position (dest1)
	wmraArm.autonomous(dest1, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
	Sleep(6000); // wait for motion end

	int delay = (int)(length/50 * 1000) + 20 ;

	while(loopCount > 0 && choice==1)
	{
		cout << "Loop: " << loopCount << endl;
		wmraArm.autonomous(dest2, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
		Sleep(delay); // wait for motion end
		wmraArm.autonomous(dest3, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
		Sleep(delay); // wait for motion end
		wmraArm.autonomous(dest4, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
		Sleep(delay); // wait for motion end
		wmraArm.autonomous(dest1, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
		Sleep(delay); // wait for motion end
		loopCount--;
	}
}

bool moveJoint()
{
	int choice, jointNum;
	double angle, angleRadians;
	
	cout << "Which Joint? ";
	cin >> jointNum;
	jointNum--;

	cout << "abs=0, rel=1? ";
	cin >> choice;

	cout << "Angle? ";
	cin >> angle;
	angleRadians = degToRad(angle);

	wmraArm.moveJoint(jointNum, angleRadians, choice);
	return 1;
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

      //BCI_motion();

      while(!endFlag){
         cout << "Current Position is :" << endl;
         WMRA::Pose pose = wmraArm.getPose();
         cout << "x = " << pose.x << ", y = " << pose.y << ", z = " << pose.z ; 
         cout << " ,yaw= " << radToDeg(pose.yaw) << " ,pitch= " << radToDeg(pose.pitch) << " ,roll= " << radToDeg(pose.roll) <<endl;

         cout << "Select an option (0 = Exit, 1 = Continue, 2 = Go to Ready, 3 = ready to park, 4 = park to ready, 5 = square, 6 = Move Joint) : "; 
         cin >> option;

         if(option==1){
            WMRA::Pose dest = getUserDest();            
            cout << "Which cordinate frame? 1=ABS, 2=REL, 3=Gripper, 7=skip: "; 
            cin >> cordframe; 
            if(cordframe==1){
               try{
                  wmraArm.autonomous(dest, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
               }
               catch(...){
                  cout << "haha" << endl;
               }
            }
            else if(cordframe==2){
               wmraArm.autonomous(dest, WMRA::ARM_FRAME_REL); // Moves arm
            }
            else if(cordframe==3){
               wmraArm.autonomous(dest, WMRA::GRIPPER_FRAME_REL); // Moves arm
            }
            else{
               cout << "skipping motion..." << endl;
               continue;
            }
            Sleep(10000); // wait for motion end
         }
         else if(option==2){
            wmraArm.autonomous(readyPose, WMRA::ARM_FRAME_PILOT_MODE);
            Sleep(10000);
         }
		 else if(option==3){
			 wmraArm.ready2Park();
		 }
		 else if(option==4){
			 wmraArm.park2Ready();
		 }
		 else if(option==5){
			 continuousSquare(wmraArm.getPose());
		 }
		 else if(option==6){
			 moveJoint();
		 }
         else if(option==0){
            wmraArm.closeDebug();
            endFlag = true;
         }
      } //end of while loop
      cout << "About to exit program. Would you like to go to ready position? 1=Yes 0=No : " ;
      cin >> option;
      if(option ==1){
         wmraArm.autonomous(readyPose, WMRA::ARM_FRAME_PILOT_MODE);
         Sleep(10000);
      }
   }
   else{
      cout << "Controller initalizaition failed in main" << endl;
   }

   cout << "Exiting Program..... Press any key to close this window." << endl;
   cin.get();
}