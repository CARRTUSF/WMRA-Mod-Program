#pragma once

#include "PathFinder.h"
#include "wmraLJ.h"

#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <time.h>
#include <Windows.h>
#include <cmath>            //Used for fmod()

#define ROTATE    0
#define TRANSLATE 1

using namespace std;

// Command struct is how commands are issued to the wheelchair 
typedef struct {
    int move;           //ROTATE or TRANSLATE
    char direction;     //Movement based on 'x' or 'y' axis, OR a left 'l' or right 'r' turn
    double distance;    //Rotation in radians OR axis (x or y) translation from global starting location
} Command;

map<string, int> CreateMap();

double round(double r);

vector<Command> CalculateWaypoints(vector<PathNode> &Coords, string startingDirection);

void DisplayVectorCommands(vector<Command> C);

void WheelChairPoseGlobal(double* PreviousPose, double time, long* R_en, long* L_en,double* CurrentPose,double & xdot, double & phidot);

void InitializeLabJack(vector<Command> Commands);

vector<PathNode> TransformPoints(vector<PathNode> &input);

void Drive(vector<Command> Commands);

void DoCalculations();