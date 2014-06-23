#ifndef SOCKET_CONTROL_H
#define SOCKET_CONTROL_H

#pragma once

//namespace WMRA{
//	class Arm;
//};

class Arm;

#include <string>
#include "tinythread.h"
#include "WmraTypes.h"

class SocketControl
{
public:
	SocketControl(Arm* robot);
	~SocketControl(void);
private :
	tthread::thread* t;
	Arm* robotArm;
	static void socketListenReply(void * aArg);
	string selectAction(string cmd);
	bool pickupObject(string cmd);
	bool trashObject(string cmd);
	bool pourObject(string cmd);
	bool bringObject(string cmd);
	bool cameraViewGripper(string cmd);
	bool moveArmTo(string cmd);

private:
	WMRA::Pose waitingPose;
	WMRA::Pose intermediateWaitingPose;
};

#endif
