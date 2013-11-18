
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
      WMRA::Pose readyPose = wmraArm.getPose();
      int cordframe;
      double temp;
		while(!endFlag){
         cout << "Current Position is :" << endl;

         WMRA::Pose pose = wmraArm.getPose();
         cout << "x = " << pose.x << ", y = " << pose.y << ", z = " << pose.z ; 
         cout << " ,yaw= " << radToDeg(pose.yaw) << " ,pitch= " << radToDeg(pose.pitch) << " ,roll= " << radToDeg(pose.roll) <<endl;

         //WMRA::Pose destpose;
         //destpose = pose;
         //destpose.y += 400;
         //destpose.z += -200;
         //wmraArm.autonomous2(destpose, WMRA::ARM_FRAME_ABS);
         //Sleep(9000);
         //destpose = wmraArm.getPose();
         //destpose.yaw += degToRad(10);
         //wmraArm.autonomous2(destpose, WMRA::ARM_FRAME_ABS);
         //Sleep(5000);
         //WMRA::Pose pose2;
         //pose2.z = 150;
         //pose2.pitch = -30;
         //wmraArm.autonomous2(pose2, WMRA::GRIPPER_FRAME_REL);
         //Sleep(6000);
         //WMRA::Pose pose3 = pose;
         //pose3.x += -100;
         //pose3.yaw = -180;
         //wmraArm.autonomous2(pose3, WMRA::ARM_FRAME_ABS); //goto ready



         

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
         cout << "Which cordinate frame? 1=ABS, 2=REL, 3=Gripper : " ; 
         cin >> cordframe;
         

			if(dest.yaw == 555)
				wmraArm.toReady();
			else if(dest.yaw == 777)
				dest.yaw == dest.yaw;// No-Op, will loop through choices again
			else if(dest.yaw == 999)
			{
				wmraArm.closeDebug();
				endFlag = true; // Will break out of loop
			}
         else{

            if(cordframe== 1){
            wmraArm.autonomous2(dest, WMRA::ARM_FRAME_ABS); // Moves arm
            }
            if(cordframe== 2){
               wmraArm.autonomous2(dest, WMRA::ARM_FRAME_REL); // Moves arm
            }
            if(cordframe== 3){
               wmraArm.autonomous2(dest, WMRA::GRIPPER_FRAME_REL); // Moves arm
            }
         }
         Sleep(10000);

         cout << "Select an option (0=Stop 1=Continue 2=Go to Ready) : " << endl;
         int option;
         cin >> option;
         if(option == 2){
            wmraArm.autonomous2(readyPose, WMRA::ARM_FRAME_ABS);
         }
         else if(option == 0){
            endFlag = false;
         }
		}

	}
	else
		cout << "Controller initalizaition failed in main" << endl;
	
	cout << "Ending Program" << endl;
}