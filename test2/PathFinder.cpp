#include "PathFinder.h"
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <fcntl.h>

PathFinder::PathFinder()
{
    reachedGoal = false;
}

PathFinder::~PathFinder()
{
}

PathFinder::PathFinder(int Obstacles[WIDTH][HEIGHT])
{
    for (int i = 0; i < WIDTH; i++) {
        for (int j = 0; j < HEIGHT; j++) {
            obstacles[i][j] = Obstacles[i][j];
            checkList[i][j] = 0;
        }
    }
    found_path = false;
}

bool PathFinder::FindPath(int x_start, int y_start, int x_target, int y_target){
    int numOpenNodes = 0;   // Number of nodes in the openList
    int newNodeID = 0;  // ID assigned to new nodes added to openList
    int newNodePlaceholder = 0;
    int u = 0;
    int v = 0;
    int temp;
    int parentX;        // x-value of current node's parent
    int parentY;        // y-value of current node's parent
    // Make sure I am not already at destination
    if (x_start == x_target && y_start == y_target) {
        found_path = false;
        std::cout << "Already at destination\n";
        reachedGoal = true;
        return found_path;
    }
    
    // Make sure target is not an obstacle (-1)
    if (obstacles[x_target][y_target] == -1) {
        found_path = false;
        std::cout << "Target is an obstacle\n";
        return found_path;
    }
    
    // Add Starting node to openList
    openList[1] = 1;
    openX[1] = x_start;
    openY[1] = y_start;
    numOpenNodes = 1;
    gCost[x_start][y_start] = 0;
    
    // Main loop for A* path finding algorithm
    while (1) {
        if (numOpenNodes > 0) {
            // Step 1: Move 1st node from openList to closed list, save x and y as parents to be used later
            parentX = openX[openList[1]];
            parentY = openY[openList[1]];
            checkList[parentX][parentY] = CLOSED;
            
            // Remove the first node in the array (lowest F Cost) and put new lowest cost at openList[1]
            // Start by losing first element in openList
            // Note that openList is in the form of a binary heap
            numOpenNodes--;
            openList[1] = openList[numOpenNodes+1];
            v = 1;
            
            // Sort the heap
            while (1) {
                u = v;
                // If both children exist
                if (2*u+1 <= numOpenNodes) {  
                    //Check if the F cost of the parent is greater than each child.
                    //Select the lowest of the two children.
                    if (fCost[openList[u]] >= fCost[openList[2*u]]) {
                        v = 2*u;
                    }
                    if (fCost[openList[v]] >= fCost[openList[2*u+1]]) {
                        v = 2*u+1;
                    }
                }
                else {
                    //if only child #1 exists
                    if (2*u <= numOpenNodes) {
                        //Check if the F cost of the parent is greater than child #1    
                        if (fCost[openList[u]] >= fCost[openList[2*u]]) {
                            v = 2*u;
                        }
                    }    
                }
                
                //if parent's F is > one of its children, swap them
                if (u != v) {
                    temp = openList[u];
                    openList[u] = openList[v];
                    openList[v] = temp;
                }
                //otherwise, exit loop
                else {
                    break;
                }
            }
            
            // Add adjacent available nodes to openList
            for (int y = parentY-1; y <= parentY+1; y++) {
                for (int x = parentX-1; x <= parentX+1; x++) {
                    // If node is not off of the map
                    if (x >= 0 && y>=0 && y <= HEIGHT && x <= WIDTH) {
                        // If not already on closed list
                        if (checkList[x][y] != CLOSED) {
                            // If not an obstacle
                            if (obstacles[x][y] != -1)  {
                                // CANNOT CUT CORNERS OF OBSTACLES!!
                                bool canCut = true;
                                // Check to see if North,South,East,West squares are obstacles.
                                // If they are, then the corner cannot be cut.
                                if (x == parentX - 1) {
                                    if (y == parentY - 1) {
                                        if (obstacles[parentX-1][parentY] == -1 || obstacles[parentX][parentY-1] == -1) {
                                            canCut = false;
                                        }
                                    }
                                    else if (y == parentY + 1) {
                                        if (obstacles[parentX][parentY+1] == -1 || obstacles[parentX-1][parentY] == -1) {
                                            canCut = false;
                                        }
                                    }
                                }
                                else if (x == parentX + 1) {
                                    if (y == parentY - 1) {
                                        if (obstacles[parentX][parentY-1] == -1 || obstacles[parentX+1][parentY] == -1) {
                                            canCut = false;
                                        }
                                    }
                                    else if (y == parentY + 1) {
                                        if (obstacles[parentX+1][parentY] == -1 || obstacles[parentX][parentY+1] == -1) {
                                            canCut = false;
                                        }
                                    }
                                }
                                // If the the node is accessible
                                if (canCut) {
                                    // If the node is not already in the openList, add it, and record the x and y values
                                    if (checkList[x][y] != OPEN) {
                                        newNodeID++;
                                        newNodePlaceholder = numOpenNodes + 1;
                                        openList[newNodePlaceholder] = newNodeID;
                                        openX[newNodeID] = x;
                                        openY[newNodeID] = y;
                                        
                                        // Find Costs
                                        int G;
                                        // If node is a diagonal
                                        if (x - parentX != 0 && y - parentY != 0) {
                                            G = 14;
                                        }
                                        else { 
                                            G = 10;
                                        }
                                        gCost[x][y] = gCost[parentX][parentY] + G;
                                        
                                        // Find  H Cost. We are using the Manhatten method
                                        hCost[openList[newNodePlaceholder]] = 10*(abs(x - x_target) + abs(y - y_target));
                                        fCost[openList[newNodePlaceholder]] = gCost[x][y] + hCost[openList[newNodePlaceholder]];
                                        // Store values into grid arrays
                                        parentGrid[x][y].x = parentX;
                                        parentGrid[x][y].y = parentY;
                                        
                                        // Sort the list
                                        while (newNodePlaceholder != 1) {
                                            // Check child's F Costs
                                            if (fCost[openList[newNodePlaceholder]] <= fCost[openList[newNodePlaceholder/2]]) {
                                                temp = openList[newNodePlaceholder/2];
                                                openList[newNodePlaceholder/2] = openList[newNodePlaceholder];
                                                openList[newNodePlaceholder] = temp;
                                                newNodePlaceholder = newNodePlaceholder/2;
                                            }
                                            else {
                                                break;
                                            }
                                        }
                                        
                                        numOpenNodes++;
                                        // update the checkList
                                        checkList[x][y] = OPEN;
                                    }
                                    // If node was already in the openList
                                    else { 
                                        // Find new cost of G
                                        int G;
                                        // If node is a diagonal
                                        if (x - parentX != 0 && y - parentY != 0) {
                                            G = 14;
                                        }
                                        else {
                                            G = 10;
                                        }
                                        int Gtest = gCost[parentX][parentY] + G;
                                        
                                        // If new path is shorter
                                        if (Gtest < gCost[x][y]) {
                                            // Change the parent node
                                            parentGrid[x][y].x = parentX;
                                            parentGrid[x][y].y = parentY;
                                            // Update the G Cost
                                            gCost[x][y] = Gtest;
                                            
                                            // Update the F Cost
                                            // Find the node in the openList
                                            for (int id = 1; id <= numOpenNodes; id++) {
                                                if (openX[openList[id]] == x && openY[openList[id]] == y) {
                                                    // Update the F Cost
                                                    fCost[openList[id]] = gCost[x][y] + hCost[openList[id]];
                                                    
                                                    // Make sure the lowest F Cost is still at the top of the openList
                                                    newNodePlaceholder = x;
                                                    while (newNodePlaceholder != 1) {

                                                        if (fCost[openList[newNodePlaceholder]] < fCost[openList[newNodePlaceholder/2]]) {
                                                            temp = openList[newNodePlaceholder/2];
                                                            openList[newNodePlaceholder/2] = openList[newNodePlaceholder];
                                                            openList[newNodePlaceholder] = temp;
                                                            newNodePlaceholder = newNodePlaceholder/2; 
                                                        }
                                                        else {
                                                            break;
                                                        }
                                                    }
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    
        else { //numOpenNodes == 0 
            std::cout << "could not find target\n";
            found_path = false;
            break;
        }
        
        // If target is in the openList, a path is found_path
        if (checkList[x_target][y_target] == OPEN) {
            found_path = true;
            break;
        }
    }
    
    // Trace the path back to the start
    if (found_path) {
        int i = 0;
        PathNode pathTracer[WIDTH*HEIGHT];
        pathTracer[i].x = x_target;
        pathTracer[i].y = y_target;
        do {  //While Tracer is not at the starting point
            // Add to the Path
            Path.push_back(pathTracer[i]);
            i++;
            //Get the Parent of each node
            int temp = parentGrid[pathTracer[i-1].x][pathTracer[i-1].y].x;
            pathTracer[i].y = parentGrid[pathTracer[i-1].x][pathTracer[i-1].y].y;
            pathTracer[i].x = temp;
        } while(pathTracer[i].x != x_start || pathTracer[i].y != y_start);
        
        // (15, 15) is the starting point!
        PathNode startnode(15,15);
        Path.push_back(startnode);

        // Vector holds the path in reverse order
        std::reverse(Path.begin(), Path.end());
    }
    return found_path;
}

void PathFinder::ClearData() {
    for (int i = 0; i < WIDTH; i++) {
        for (int j = 0; j < HEIGHT; j++) {
            obstacles[i][j] = -2;
            openList[i+j] = 0;
            openX[i+j] = 0;
            openY[i+j] = 0;
            checkList[i][j] = 0;
            parentGrid[i][j].x = 0;
            parentGrid[i][j].y = 0;
            fCost[i+j] = 0;
            gCost[i][j] = 0;
            hCost[i+j] = 0;
            found_path = false;
            Path.clear();
        }
    }
    
    
}

std::vector<PathNode> PathFinder::GetPath() {
    if (!(found_path)) {
        std::cout << "Path was not found\n\n";
    }
    return Path;
}

void PathFinder::DisplayPath()
{
    if (found_path) {
        std::cout << "Path to follow: ";
        for (size_t i = 0; i < Path.size(); i++) {
            std::cout << "(" << Path.at(i).x << "," << Path.at(i).y << ") ";
        }
        std::cout << std::endl << std::endl;
    }
    else {
        std::cout << "No path found!\n\n";
    }
}
