#ifndef SOCKET_CONTROL_H
#define SOCKET_CONTROL_H

#pragma once

//namespace WMRA{
//	class Arm;
//};

class Arm;

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
	bool graspObject(WMRA::Pose objectPose) ;
};

#endif

