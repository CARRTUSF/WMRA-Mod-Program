
#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include "tinythread.h"
#include "matrix.h" 
#include "MotorController.h"
#include "Arm.h"
#include "Utility.h"

#define PI 3.14159265

using namespace std;
using namespace tthread;

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


void socketComm(void * aArg){

   receiving_udpsocket socket1( "localhost:7000" );
   sockstream read_sock( socket1 );
   sending_udpsocket socket2( "localhost:7001" );
   sockstream output_sock( socket2 );
   WMRA::Pose curPose;

   if( !read_sock.is_open() ){
      cerr << "Could not open read socket for reading." << endl;
   }

   cout << "hey thread started " << endl;
   string temp_str_buf;
   char temp_buf[200];
   while(true){
      do{
         getline( read_sock, temp_str_buf );
      } while( temp_str_buf.find("GETPOS")== string::npos ); // keep reading until message is received
      //send position
      curPose = wmraArm.getPose();
      output_sock << "POSITION " << curPose.x << " " << curPose.y << " " << curPose.z << endl; 
   }
}



bool graspObject(WMRA::Pose objectPose) // This function assumes orientation to be 0,0,0
{
	WMRA::Pose prePose = objectPose;
	prePose.x = prePose.x-100.0;
	prePose.z = prePose.z+100.0; // Prepose will always be higher than grasping position

   WMRA::Pose graspPose = objectPose;
   graspPose.x = graspPose.x + 50;

   WMRA::Pose liftTable = graspPose;
   liftTable.z = liftTable.z +100;

   cout << "going to prepose" <<endl;
	wmraArm.autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
   Sleep(2000);
   cout << "Opening gripper " << endl;
   wmraArm.openGripper();
   Sleep(2000);
   cout << "Going to grasp pose" << endl;
	wmraArm.autonomous(graspPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to object location
   Sleep(2000);
   cout << "Closing Gripper" <<endl;
	wmraArm.closeGripper();
   Sleep(2000);

	//objectPose.z = objectPose.z + 100.0; // Raising object
   cout << "Raising Object" << endl;
	wmraArm.autonomous(liftTable, WMRA::ARM_FRAME_PILOT_MODE); // Raising object
   Sleep(10000);

   cout << "Going to prepose" << endl;
	wmraArm.autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
   Sleep(10000);

	return true;
}

/*
* Allows to control the arm through a socket connection.
* Parses GOTOPOSE x y z message and grasps a object at that pose
*/
void socketControl(void * aArg){

   receiving_udpsocket socket1( "localhost:7000" );
   sockstream read_sock( socket1 );
   sending_udpsocket socket2( "localhost:7001" );
   sockstream output_sock( socket2 );
   WMRA::Pose curPose = wmraArm.getPose();

   if( !read_sock.is_open() ){
      cerr << "Could not open read socket for reading." << endl;
   }

   cout << "hey thread started " << endl;
   string temp_str_buf;
   char temp_buf[200];
   while(true){
      do{
         getline( read_sock, temp_str_buf );
      } while( temp_str_buf.find("PICKUP")== string::npos ); // keep reading until message is received
      WMRA::Pose graspPose;
      graspPose.clear();
      sscanf(temp_str_buf.c_str(), "%s %lf %lf %lf", temp_buf, &graspPose.x, &graspPose.y, &graspPose.y);
      graspObject(graspPose);
      

      output_sock << "DONE" << endl;
      output_sock << "POSITION " << curPose.x << " " << curPose.y << " " << curPose.z << endl; 
   }
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

      //thread t(socketControl, 0); // start the communication thread
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

         cout << "Select an option (0 = Exit, 1 = Continue, 2 = Go to Ready, 3 = ready to park, 4 = park to ready, 5 = square, 6 = Move Joint 8 = Grasp Object) : "; 
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
       else if(option==8){
          WMRA::Pose dest = getUserDest();
			 graspObject(dest);
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