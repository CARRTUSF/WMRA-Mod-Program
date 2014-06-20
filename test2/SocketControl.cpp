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

void SocketControl::socketListenReply(void * aArg) {
	SocketControl* self = (SocketControl*)aArg ;
	receiving_udpsocket socket1( "0.0.0.0:7500" );
	sockstream read_sock( socket1 );
	sending_udpsocket socket2( "localhost:7501" );
	sockstream output_sock( socket2 );


	if(!read_sock.is_open()) {
		cerr << "Could not open read socket for reading." << endl;
	}

	cout << "hey thread started " << endl;
	string temp_str_buf;
	char temp_buf[200];
	while(true) {
		do{
			getline( read_sock, temp_str_buf );
		} while(temp_str_buf.find("COMMAND") == string::npos); // keep reading until message is received
		cout << "\n received command: " << string(temp_str_buf) << endl;
		self->selectAction(temp_str_buf);// selcts the action and calls appropriate function

		//graspObject(graspPose);

		output_sock << "DONE" << endl;
		//output_sock << "POSITION " << curPose.x << " " << curPose.y << " " << curPose.z << endl; 
	}
}

string SocketControl::selectAction(string cmd) {
	//
	cmd.erase(0,8); //erase "COMMAND " from the begining
	if(cmd.find("PICK_UP") != string::npos) {
		pickupObject(cmd);
	} else if(cmd.find("TRASH")!= string::npos) {
		trashObject(cmd);
	} else if(cmd.find("POUR")!= string::npos) {
		pourObject(cmd);
	} else if(cmd.find("BRING_TO_USER")!= string::npos) {
		bringObject(cmd);
	} else if(cmd.find("CAMERA_VIEW_CLOSE")!= string::npos) {
		cameraViewGripper(cmd);
	} else if(cmd.find("MOVE_ARM_TO")!= string::npos) {
		moveArmTo(cmd);
	}

	return "DONE";
}


bool SocketControl::moveArmTo(string cmd){
	double pos[3] = {0};
	char temp_buf[200];
	int numRead = sscanf(cmd.c_str(), "%s %lf %lf %lf", temp_buf, &pos[0], &pos[1], &pos[2]);

	WMRA::Pose objectPose;
	objectPose.clear();
	objectPose.x = pos[0];
	objectPose.y = pos[1];
	objectPose.z = pos[2];

	cout << "moving arm to " << pos[0] << "," << pos[1] <<"," << pos[2] << endl;
	robotArm->autonomous(objectPose, WMRA::ARM_FRAME_PILOT_MODE, true); // Move to object location

	return true;
}

bool SocketControl::pickupObject(string cmd) // This function assumes orientation to be 0,0,0
{
	double pos[6] = {0};
	char temp_buf[200];
	cmd.erase(0, 8);
	int numRead = sscanf(cmd.c_str(), "%lf %lf %lf", &pos[0], &pos[1], &pos[2]);

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
	//Sleep(2000);
	cout << "Opening gripper " << endl;
	robotArm->openGripper();
	//Sleep(5000);
	cout << "Going to grasp pose" << endl;
	robotArm->autonomous(graspPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to object location
	//Sleep(2000);
	cout << "Closing Gripper" <<endl;
	robotArm->closeGripper();
	//Sleep(5000);

	//objectPose.z = objectPose.z + 100.0; // Raising object
	cout << "Raising Object" << endl;
	robotArm->autonomous(liftTable, WMRA::ARM_FRAME_PILOT_MODE); // Raising object
	//Sleep(10000);

	cout << "Going to prepose" << endl;
	robotArm->autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
	//Sleep(10000);

	return true;
}

bool SocketControl::trashObject(string cmd) {
	double objPos[3] = {0};
	double objRot[3] = {0};
	double trashPos[3] = {0};
	cmd.erase(0, 6);
	int numRead = sscanf(cmd.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf", 
		&objPos[0], &objPos[1], &objPos[2],
		&objRot[0], &objRot[1], &objRot[2], 
		&trashPos[0], &trashPos[1], &trashPos[2]);

	return true;

}

bool SocketControl::pourObject(string cmd) {
	double objPos[3] = {0};
	double objRot[3] = {0};
	double destPos[3] = {0};
	double destRot[3] = {0};
	cmd.erase(0, 5);
	int numRead = sscanf(cmd.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
		&objPos[0], &objPos[1], &objPos[2],
		&objRot[0], &objRot[1], &objRot[2], 
		&destPos[0], &destPos[1], &destPos[2],
		&destRot[0], &destRot[1], &destRot[2]);
	return true;
}

bool SocketControl::bringObject(string cmd) {
	double objPos[3] = {0};
	double objRot[3] = {0};
	cmd.erase(0, 14);
	int numRead = sscanf(cmd.c_str(), "%lf %lf %lf %lf %lf %lf", 
		&objPos[0], &objPos[1], &objPos[2],
		&objRot[0], &objRot[1], &objRot[2]);
	return true;
}

bool SocketControl::cameraViewGripper(string cmd) {
	double objPose[6] = {0};
	char temp_buf[200];
	int numRead = sscanf(cmd.c_str(), "%s %lf %lf %lf %lf %lf %lf", 
		temp_buf, &objPose[0], &objPose[1], &objPose[2], &objPose[3], &objPose[4], &objPose[5]);
	if(numRead == 7){

		//calculate the destination position for the arm to get a better look at the object
		// minus x direction and look down towards the object from about 20 cm away.
		WMRA::Pose objectPose;
		objectPose.clear();
		objectPose.x = objPose[0] -30 ; // minus 30 in x direction ( x is forward dir of the wheelchair)
		objectPose.y = objPose[1]; 
		objectPose.z = objPose[2];
		objectPose.roll = objPose[3];
		objectPose.pitch = objPose[4];
		objectPose.yaw = objPose[4];

		//maybe rotate +10 in pitch

		cout << "Going to pose" << endl;
		robotArm->autonomous(objectPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to object location
		return true;
	}
	else{
		return false;
	}

}