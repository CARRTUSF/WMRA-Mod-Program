
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

      while(!endFlag){
         cout << "Current Position is :" << endl;
         WMRA::Pose pose = wmraArm.getPose();
         cout << "x = " << pose.x << ", y = " << pose.y << ", z = " << pose.z ; 
         cout << " ,yaw= " << radToDeg(pose.yaw) << " ,pitch= " << radToDeg(pose.pitch) << " ,roll= " << radToDeg(pose.roll) <<endl;

         cout << "Select an option (0 = Exit  1 = Continue 2 = Go to Ready) : "; 
         cin >> option;
        
         if(option == 1){
            WMRA::Pose dest = getUserDest();            
            cout << "Which cordinate frame? 1=ABS, 2=REL, 3=Gripper 7=skip: " ; 
            cin >> cordframe; 
            if(cordframe== 1){
               try{
                  wmraArm.autonomous2(dest, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
               }
               catch(...){
                  cout << "haha" << endl;
               }
            }
            else if(cordframe== 2){
               wmraArm.autonomous2(dest, WMRA::ARM_FRAME_REL); // Moves arm
            }
            else if(cordframe== 3){
               wmraArm.autonomous2(dest, WMRA::GRIPPER_FRAME_REL); // Moves arm
            }
            else{
               cout << "skipping motion..." << endl;
               continue;
            }
            Sleep(10000); // wait for motion end
         }
         else if(option == 2){
            wmraArm.autonomous2(readyPose, WMRA::ARM_FRAME_ABS);
            Sleep(10000);
         }
         else if(option == 0){
            wmraArm.closeDebug();
            endFlag = true;
         }
      } //end of while loop
      cout << "About to exit program. WOuld you like to go to ready position? 1=Yes 0=No : " ;
      cin >> option;
      if(option ==1){
         wmraArm.autonomous2(readyPose, WMRA::ARM_FRAME_ABS);
         Sleep(10000);
      }
     
   }
   else{
      cout << "Controller initalizaition failed in main" << endl;
   }

    cout << "Exiting Program..... Press any key to close this window." << endl;
    cin.get();
}