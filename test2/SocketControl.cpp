#include "SocketControl.h"
#include "SockStream.h"
#include "Arm.h"



SocketControl::SocketControl(Arm* robot)
{
	robotArm = robot;
	t = new thread(socketListenReply,this);
}


SocketControl::~SocketControl(void)
{
}

void SocketControl::socketListenReply(void * aArg){


   SocketControl* self = (SocketControl*)aArg ;
   receiving_udpsocket socket1( "0.0.0.0:7500" );
   sockstream read_sock( socket1 );
   sending_udpsocket socket2( "localhost:7501" );
   sockstream output_sock( socket2 );
   

   if( !read_sock.is_open() ){
      cerr << "Could not open read socket for reading." << endl;
   }

   cout << "hey thread started " << endl;
   string temp_str_buf;
   char temp_buf[200];
   while(true){
      do{
         getline( read_sock, temp_str_buf );
      } while( temp_str_buf.find("COMMAND")== string::npos ); // keep reading until message is received
	  self->selectAction(temp_str_buf);// selcts the action and calls appropriate function
     
      //graspObject(graspPose);


      output_sock << "DONE" << endl;
      //output_sock << "POSITION " << curPose.x << " " << curPose.y << " " << curPose.z << endl; 
   }
}

string SocketControl::selectAction(string cmd){
	//
	cmd.erase(0,8); //erase "COMMAND " from the begining
	if(cmd.find("PICKUP") != string::npos){
		graspObject(cmd);
	}
	return "DONE";
}

bool SocketControl::graspObject(string cmd) // This function assumes orientation to be 0,0,0
{

	
	double pos[3] = {0};
	char temp_buf[200];
	int numRead = sscanf(cmd.c_str(), "%s %lf %lf %lf", temp_buf, &pos[0], &pos[1], &pos[2]);


	WMRA::Pose objectPose;
	objectPose.clear();
	objectPose.x = pos[0];
	objectPose.y = pos[1];
	objectPose.z = pos[2];


   WMRA::Pose curPose = robotArm->getPose();
   WMRA::Pose prePose = objectPose;
   prePose.x = prePose.x-100.0;
   prePose.z = prePose.z+100.0; // Prepose will always be higher than grasping position

   WMRA::Pose graspPose = objectPose;
   graspPose.x = graspPose.x + 50;

   WMRA::Pose liftTable = graspPose;
   liftTable.z = liftTable.z +100;

   cout << "going to prepose" <<endl;
   robotArm->autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
   Sleep(2000);
   cout << "Opening gripper " << endl;
   robotArm->openGripper();
   Sleep(5000);
   cout << "Going to grasp pose" << endl;
   robotArm->autonomous(graspPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to object location
   Sleep(2000);
   cout << "Closing Gripper" <<endl;
   robotArm->closeGripper();
   Sleep(5000);

   //objectPose.z = objectPose.z + 100.0; // Raising object
   cout << "Raising Object" << endl;
   robotArm->autonomous(liftTable, WMRA::ARM_FRAME_PILOT_MODE); // Raising object
   Sleep(10000);

   cout << "Going to prepose" << endl;
   robotArm->autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
   Sleep(10000);

   return true;
}
