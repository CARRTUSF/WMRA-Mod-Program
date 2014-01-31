
#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <Afxwin.h>
#include "WmraTypes.h"
#include "tinythread.h"
#include "SockStream.h"
#include "matrix.h" 
#include "MotorController.h"
#include "Arm.h"
#include "Utility.h"


#define PI 3.14159265

using namespace std;
using namespace tthread;

char *tempChar; //temporary char pointer passed to thread
Arm wmraArm;


sending_udpsocket socket1( "localhost:6001" );
sockstream output_sock( socket1 );
receiving_udpsocket socket2( "localhost:6000" );
sockstream read_sock( socket2 );


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

int wait4Command(){
   /*  Sleep(5000);
   return;*/
   char temp_buf[200]; 
   int object_id = 0;
   string temp_str_buf;
   getline( read_sock, temp_str_buf );
   while( temp_str_buf.find("OBJECT")== string::npos ){
      getline( read_sock, temp_str_buf );
      Sleep(20);
   }
   sscanf(temp_str_buf.c_str() , "%s %d",temp_buf, &object_id);
   return object_id;
}

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



void BCI_motion(){

   cout << " started BCI loop" << endl;

   WMRA::Pose dest;
   WMRA::Pose readyPose = wmraArm.getPose();
   int object_id = 3;
   while(true){
      //object_id = wait4Command(); // wait for object id from VisualBCI
      if(true/*object_id ==3*/){
         //goto object
         dest.x = 650;
         dest.y = -100;
         dest.z = 475;
         dest.roll = 0;
         dest.pitch = 0;
         dest.yaw = degToRad(10);

         wmraArm.autonomous2(dest, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
         //Sleep(8000);

         //bringback to user

 /*        dest.x = 850;
         dest.y = -100;
         dest.z = 450;
         dest.roll = degToRad(0);
         dest.pitch = degToRad(0);
         dest.yaw = degToRad(10);*/

         dest.clear();
         dest.z=100;



         wmraArm.autonomous2(dest, WMRA::GRIPPER_FRAME_REL); // Moves arm
         wmraArm.closeGripper();

         //goto user
         dest.x = 0;
         dest.y = -250;
         dest.z = 480;
         dest.roll = degToRad(0);
         dest.pitch = degToRad(0);
         dest.yaw = degToRad(-150);

         wmraArm.autonomous2(dest, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
         Sleep(1000);
         wmraArm.openGripper();

         //got to ready

         wmraArm.autonomous2(readyPose, WMRA::ARM_FRAME_PILOT_MODE);
         Sleep(8000);

         output_sock << "DONE" << endl;
      }
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
	wmraArm.autonomous2(dest1, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
	Sleep(6000); // wait for motion end

	int delay = (int)(length/50 * 1000) + 20 ;

	while(loopCount > 0 && choice==1)
	{
		cout << "Loop: " << loopCount << endl;
		wmraArm.autonomous2(dest2, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
		Sleep(delay); // wait for motion end
		wmraArm.autonomous2(dest3, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
		Sleep(delay); // wait for motion end
		wmraArm.autonomous2(dest4, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
		Sleep(delay); // wait for motion end
		wmraArm.autonomous2(dest1, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
		Sleep(delay); // wait for motion end
		loopCount--;
	}
}

int main()
{

   int velocity;				// Max velocity of the gripper in movement
   bool endFlag = false;
   if(wmraArm.initialize()){
      //cout << "Controller intialized in main" << endl;
      thread t(socketComm, 0); // start the communication thread
      //CreateThread(NULL, 0, &socketComm, (LPVOID*)NULL, 0, NULL);
      //set tooltip
      Matrix tooltip;
      tooltip.Unit(4);
      tooltip(0,1) = -20;
      tooltip(0,2) = -135;
      wmraArm.setTooltipTransform(tooltip);
      WMRA::Pose readyPose = wmraArm.getPose();
      int cordframe;
      double temp;
      int option;  

   

      // BCI_motion();

      while(!endFlag){
         cout << "Current Position is :" << endl;
         WMRA::Pose pose = wmraArm.getPose();
         cout << "x = " << pose.x << ", y = " << pose.y << ", z = " << pose.z ; 
         cout << " ,yaw= " << radToDeg(pose.yaw) << " ,pitch= " << radToDeg(pose.pitch) << " ,roll= " << radToDeg(pose.roll) <<endl ;

         cout << "Select an option (0 = Exit, 1 = Continue, 2 = Go to Ready, 3 = ready to park, 4 = park to ready, 5 = square) : "; 
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
            wmraArm.autonomous2(readyPose, WMRA::ARM_FRAME_PILOT_MODE);
            Sleep(10000);
         }
		 else if(option == 3){
			 wmraArm.ready2Park();
		 }
		 else if(option == 4){
			 wmraArm.park2Ready();
		 }
		 else if(option == 5){
			 continuousSquare(wmraArm.getPose());
		 }
         else if(option == 0){
            wmraArm.closeDebug();
            endFlag = true;
         }
      } //end of while loop
      cout << "About to exit program. Would you like to go to ready position? 1=Yes 0=No : " ;
      cin >> option;
      if(option ==1){
         wmraArm.autonomous2(readyPose, WMRA::ARM_FRAME_PILOT_MODE);
         Sleep(10000);
      }
      //t.join();

   }
   else{
      cout << "Controller initalizaition failed in main" << endl;
   }

   cout << "Exiting Program..... Press any key to close this window." << endl;
   cin.get();
   return 0;
   
}