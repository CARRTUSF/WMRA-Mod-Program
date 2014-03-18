//**********************************************************
// PathFinder.h - written by Chris Danna
// Implementation adapted from Patrick Lester's A* Pathfinding
// http://www.policyalmanac.org/games/aStarTutorial.htm
// 


#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>

#define WIDTH 31
#define HEIGHT 31
#define CLOSED 2
#define OPEN 1

class PathNode {
public :
    int x;
    int y;
    PathNode()
    {
        x = 0;
        y = 0;
    }
    PathNode(int x_val,int y_val)
    {
        x = x_val;
        y = y_val;
    }
};

class PathFinder
{
private:
    int obstacles[WIDTH][HEIGHT];           // Obstacle map generated from Kinect and BuildOccupancyGrid()
    
    int openList[WIDTH*HEIGHT + 2];         // Holds ID for openList nodes matched with openX and openY
    int openX [WIDTH*HEIGHT + 2];           // X location of a node in the openList
    int openY [WIDTH*HEIGHT + 2];           // Y location of a node in the openList
    
    int checkList[WIDTH+1][HEIGHT+1];       // This records whether a node is in the openList (1), closed list (2), or neither (0)
    
    PathNode parentGrid[WIDTH+1][HEIGHT+1]; // Stores the (x,y) value of the parent of each node
    
    int fCost[WIDTH*HEIGHT + 2];            // Stores the F cost of a node in the openList
    int gCost[WIDTH+1][HEIGHT+1];           // Stores the G cost for each node
    int hCost[WIDTH*HEIGHT + 2];            // Stores the H cost of a node in the openList
    
    bool found_path;
    
    std::vector<PathNode> Path;

public:
    PathFinder(int Obstacles[WIDTH][HEIGHT]);
    PathFinder();
    ~PathFinder();
    
    bool FindPath(int x_start, int y_start, int x_target, int y_target);
    
    void ClearData();
    
    std::vector<PathNode> GetPath();

    bool reachedGoal;
    
    void DisplayPath();
};

#endif // PATHFINDER_H
