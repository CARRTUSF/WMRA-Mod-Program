#pragma once

#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>
#include <time.h>
#include <windows.h> 
#include <WINDOWS.H>
#include <strsafe.h>
#include <STRSAFE.H>
#include "wmraLJ.h"

struct WheelchairPose{
    double x;
    double y;
    double phi;
};

class WheelchairControl
{
public:
	WheelchairControl(void);
	~WheelchairControl(void);
	bool StopWheelChair();
	WheelchairPose getPose();
	bool moveTo(double x,double y);
private:
	LJ_HANDLE lngHandleU6;
	static WheelchairPose currentPose; // contains the current pose for the wheelchair
	static bool writingPose;
	static int instances;
	bool moveToX(double x); // moves forward until the desired X value is reached
	bool moveToY(double y); // moves forward until the desired y value is reached
	bool rotateLeft(double deltaRadians);
	bool rotateRight(double deltaRadians);
	HANDLE startWCPoseCalculationthread();
	static DWORD WINAPI wheelchairPose(LPVOID pSelf);
};

