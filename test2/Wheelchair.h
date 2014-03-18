
#ifndef WHEELCHAIR_H
#define WHEELCHAIR_H

#include "wmraLJ.h"

#include <vector>
#include <time.h>
#include <iostream>
#include <cmath>            //Used for fmod()
#include <windows.h>

#define ROTATE    0
#define TRANSLATE 1

using namespace std;

// Command struct is how commands are issued to the wheelchair 
typedef struct {
    int move;           //ROTATE or TRANSLATE
    char direction;     //Movement based on 'x' or 'y' axis, OR a left 'l' or right 'r' turn
    double distance;    //Rotation in radians OR axis (x or y) translation from global starting location
} Command;

class Wheelchair{
public:
	Wheelchair();
	void Drive(vector<Command> Commands);
private:
	void WheelChairPoseGlobal(double* PreviousPose, double time, long* R_en, long* L_en,double* CurrentPose,double & xdot, double & phidot);
	void DoCalculations();
	double *Pose;
	double *CurrentPose;
	double X;
	double Y;
	double PPhi;
	double encoder1;
	double encoder2;
	clock_t currentTime;
};
#endif;
