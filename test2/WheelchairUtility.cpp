#include "WheelchairUtility.h"

double *Pose = new double[3];
double *CurrentPose = new double[3];
double X = 0.0, Y = 0.0, PPhi = 0.0;
double encoder1=0.0, encoder2=0.0;
clock_t currentTime = 0;

// Map for converting Cardinal direction to degrees. N=0, E=90
map<string, int> CreateMap() {
    map<string, int> m;
    m["N"]  = 0;
    m["NE"] = 45;
    m["E"]  = 90;
    m["SE"] = 135;
    m["S"]  = 180;
    m["SW"] = 225;
    m["W"]  = 270;
    m["NW"] = 315;

    return m;
}

//round-off function
double round(double r) {
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

// Function    : CalculateWaypoints
// Returns     : vector of Commands to send to wheelchair
// Parameters  : vector of PathNodes and a starting direction
// Description : Analyzes a list of coordinates and generates a series of commands (Translation/Rotation and Distance)
//               to be issued to the wheelchair
vector<Command> CalculateWaypoints(vector<PathNode> &Coords, string startingDirection) {
    map<string, int> mapDir = CreateMap();
    vector<Command> commands;
    string prevDir = startingDirection;
    string nextDir;
    int prevDirDeg = 0, nextDirDeg = 0;//Directions converted to degrees
    int dx, dy;                     //dx and dy are the change in (x,y) between each PathNode
    int turn = 0;                   //ROTATION represented in degrees
    double turnRad = 0.0;           //turn in Radians
    double translation = 0.0;       //increasing translation when moving in one direction for more than 1 waypoint

    for (size_t i = 0; i < Coords.size() - 1; i++)
    {
        Command newCommand;
        nextDir = "";       //reset next direction
        dx = (Coords[i].x - Coords[i+1].x);
        dy = Coords[i].y - Coords[i+1].y;


        // Figure out where next direction is, i.e. where do we need to go
        // NOTE that these NSEW directions are based off wheelchair global frame, not Cardinal
        if (dx < 0)                //direction is somewhere NORTH
            nextDir = "N";
        else if (dx == 0)          //direction is EAST or WEST
            nextDir = "";
        else if (dx > 0)           //direction is somewhere SOUTH   
            nextDir = "S";
        if (dy > 0)                //direction is somewhere EAST
            nextDir += "E";
        else if (dy == 0)          //direction is NORTH or SOUTH
        { }  
        else if (dy < 0)           //direction is somewhere WEST
            nextDir += "W";

        if (prevDir.compare(nextDir) == 0) { //continue translation without rotation
            newCommand.move = TRANSLATE;
            double xDistance = Coords[i+1].x - Coords[0].x;         // Global translation distance for x
            double yDistance = Coords[i+1].y - Coords[0].y;         // Global translation distance for y
            

            if (yDistance == Coords[i].y - Coords[0].y) {
                //use x for global reference
                if (xDistance != 0) {
                    newCommand.direction = 'x';
                    newCommand.distance = xDistance;
                }
                else {                    //x and y values are starting location...
                    cout << "Error calculating next direction from ";
                    cout << "(" << Coords[i].x << ", " << Coords[i].y << ") ";
                    cout << "to (" + Coords[i+1].x << ", " + Coords[i+1].y << ") ";
                    cout << endl << endl;

                    commands.clear();
                    return commands;
                }
            }
            else {
                //use y for global reference
                newCommand.direction = 'y';
                newCommand.distance = yDistance;
            }
            
            commands.push_back(newCommand);
        }
        else {
            // Convert direction to degress using the map
            prevDirDeg = mapDir.find(prevDir)->second;
            nextDirDeg = mapDir.find(nextDir)->second;

            if (nextDirDeg - prevDirDeg < 180 && nextDirDeg - prevDirDeg > 0)
                newCommand.direction = 'r';                  //turn right
            else if (nextDirDeg - prevDirDeg > 180)
                newCommand.direction = 'l';                  //turn left
            else if (nextDirDeg - prevDirDeg < -180)
                newCommand.direction = 'r';                  //turn right
            else if (nextDirDeg - prevDirDeg > -180 && nextDirDeg - prevDirDeg < 0)
                newCommand.direction = 'l';                  //turn left

            // Convert new pose PHI to a usable number for WheelchairPose
            turn = mapDir.find(nextDir)->second;
            if (turn > 180 )
                turn = 360 - turn;
            if (newCommand.direction == 'r')
                turn = turn * -1;

            // convert turn to radians
            turnRad = turn * 3.14159265 / 180;

            // add command to commands vector
            newCommand.move = ROTATE;
            newCommand.distance = turnRad;
            commands.push_back(newCommand);


            // ROTATION found, now we need to know how far to translate after that
            newCommand.move = TRANSLATE;
            double xDistance = Coords[i+1].x - Coords[0].x;         // Global translation distance for x
            double yDistance = Coords[i+1].y - Coords[0].y;         // Global translation distance for y
            if (yDistance == Coords[i].y - Coords[0].y) {
                //use x for global reference
                if (xDistance != 0) {
                    newCommand.direction = 'x';
                    newCommand.distance = xDistance;
                }
                else {                    //x and y values are starting location...
                    cout << "Error calculating next direction from ";
                    cout << "(" << Coords[i].x << ", " << Coords[i].y << ") ";
                    cout << "to (" + Coords[i+1].x << ", " + Coords[i+1].y << ") ";
                    cout << endl << endl;

                    commands.clear();
                    return commands;
                }
            }
            else {
                //use y for global reference
                newCommand.direction = 'y';
                newCommand.distance = yDistance;
            }

            // add command to commands vector
            commands.push_back(newCommand);
        }

        prevDir = nextDir;
    }

    return commands;
}

// Function    : TransformPoints
// Returns     : vector of PathNodes that are used for input to CalculateWaypoints()
// Parameters  : vector of PathNodes and a starting direction
// Description : This function applies a transformation of a vector of PathNode coordinates from the 
//               traditional xy plane into a robots frame of reference. (Rotate axis 90 degrees about the
//               point (15, 15)
vector<PathNode> TransformPoints(vector<PathNode> &input) {
    vector<PathNode> output;
    double theta = -1.57079633; //90 degrees

    for (size_t i = 0; i < input.size(); i++) {
        PathNode newCoord;

        // Ghetto transformation
        newCoord.x = round((input[i].x * cos(theta)) - (input[i].y * sin(theta))) - 15;
        newCoord.y = round(((input[i].x * sin(theta)) + (input[i].y * cos(theta))) + 15);

        output.push_back(newCoord);
    }

    return output;
}

void DisplayVectorCommands(vector<Command> C) {
    vector<Command>::size_type i;
    cout << "Commands :" << endl;

    for (i = 0; i < C.size(); i++)
    {
        cout << C[i].move << "   " << C[i].direction << " = " << C[i].distance << endl;
    }
}

// Function    : WheelChairPoseGlobal
// Returns     : void
// Parameters  : PreviousPose of wheelchair, time between readings, R and L array of encoder readings, CurrentPose
// Description : Calculates the current pose of the wheelchair based on the encoder data
//               X = x-coordinate position        Y = y-coordinate position        PPhi = orientation angle
void WheelChairPoseGlobal(double* PreviousPose, double time, long* R_en, long* L_en,double* CurrentPose,double & xdot, double & phidot) {
	double L1 = 0.555; //55.9;
	double pi=3.141592654;
	double WheelRad = 0.17; //16.8;
	int RevEncoder_R = 1580;
	int RevEncoder_L = 1550;
	int Delta_R = 0;
	int Delta_L = 0;
	double Theta_R = 0;
	double Theta_L = 0;
	double Theta_RT = 0;
	double Theta_LT = 0;
	double Translation = 0;
	double phi1 = 0;
	double Pose_x = 0;
	double Pose_y = 0;

	// The encoder reading differance between previous reading and current reading
	Delta_R = R_en[0] - R_en[1];
	Delta_L = L_en[0] - L_en[1];

	// The rotated angle for the left and right wheel
	Theta_R = 2*pi*Delta_R / RevEncoder_R;
	Theta_L = 2*pi*Delta_L / RevEncoder_L;

	// The angular velocity for left and right wheel 
	//Theta_RT = Theta_R / time ;
	//Theta_LT = Theta_L / time ;

	// Translation of the wheelchair 
	Translation = (WheelRad/2)*(Theta_L + Theta_R);
	xdot = Translation/ time;

    //cout << "Delta_R = " << Delta_R << "    Delta_L = " << Delta_L << "    Trans = " << Translation << endl;

	// rotation angle of the wheelchiar
	phi1 = (WheelRad/L1)*(-Theta_L + Theta_R);
	
	phidot = phi1/time;

	if ( abs(phi1) <= 0.005) // striaght line movment rotation angle almost zero
	{
		Pose_x = Translation*cos(PreviousPose[2]) + PreviousPose[0];
		Pose_y = Translation*sin(PreviousPose[2]) + PreviousPose[1];
	}
	else // There is a rotation angle 
	{
		double r = Translation/phi1 - L1/2; // raduis of curvature
		Pose_x = (sin((pi/2)  + PreviousPose[2] + (phi1/2))*(r+(L1/2))*((sin(phi1)/cos(phi1/2)))) + PreviousPose[0];
		Pose_y = (-cos((pi/2) + PreviousPose[2] + (phi1/2))*(r+(L1/2))*((sin(phi1)/cos(phi1/2)))) + PreviousPose[1];
	}

	double Pose_phi = PreviousPose[2] + phi1; // the new oreintation angle

	// Set the values of the current wheelchair pose
	CurrentPose[0] = Pose_x;
	CurrentPose[1] = Pose_y;
	CurrentPose[2] = Pose_phi;
}

// Function    : Drive
// Returns     : void
// Parameters  : vector of Commands to issue the wheelchair
// Description : Analyzes each command and sends voltages to the wheelchairs controls
void Drive(vector<Command> Commands) {
    InitializeU6();
    cout << "Lab Jack U6 initialized..." << endl;
    InitializeTimersU6 ();
    cout << "Lab Jack Timers initialized..." << endl;

    cout << "Power the wheelchair ....." << endl;

    // Set the ideal value of the translation and rotation of the wheelchair
    SetDAC(0,2.47);
    SetDAC(1,2.47);
    Sleep(5000);

    double pi = 3.141592654;
	Pose[0]=0;
	Pose[1]=0;
	Pose[2]=0;
	CurrentPose[0]=0;
	CurrentPose[1]=0;
	CurrentPose[2]=0;

	currentTime=clock();
    
    cout << "GO\n";

    for (size_t i = 0; i < Commands.size(); i++) {
        cout << "i = " << i << endl;
        // Figure out what values to send to the encoder
        if (Commands[i].move == ROTATE) {
            double newPhi = Commands[i].distance;

            if (Commands[i].direction == 'l') {
                SetDAC(0, 2.10);
                SetDAC(1, 2.47);
                while (fmod(PPhi, (2*pi)) < newPhi) {
                    DoCalculations();
                    cout << "Angle = " << PPhi*180/pi << endl;
                }
            }
            else if (Commands[i].direction == 'r') {
                SetDAC(0, 2.85);
                SetDAC(1, 2.47);
                while(fmod(PPhi, (2*pi)) > newPhi) {
                    DoCalculations();
                    cout << "Angle = " << PPhi*180/pi << endl;
                }
            }
        }
        else { 
            //Moving Forward
            if (Commands[i].direction == 'x') {
                if (Commands[i].distance > X) {
                    // Moving in the + direction
                    while (X < Commands[i].distance) {
                        SetDAC(0, 2.47);
                        SetDAC(1, 2.85);
                        DoCalculations();
                        cout << "+ X: " << X << endl;
                    }
                }
                else {
                    // Moving in the - direction
                    while (X > Commands[i].distance) {
                        SetDAC(0, 2.47);
                        SetDAC(1, 2.85);
                        DoCalculations();
                        cout << "- X: " << X << endl;
                    }
                }
            }
            else {
                if (Commands[i].distance > Y) {
                    // Moving in the + direction
                    while (Y < Commands[i].distance) {
                        SetDAC(0, 2.47);
                        SetDAC(1, 2.85);
                        DoCalculations();
                        cout << "+ Y: " << Y << endl;
                    }
                }
                else {
                    // Moving in the - direction
                    while (Y > Commands[i].distance) {
                        SetDAC(0, 2.47);
                        SetDAC(1, 2.85);
                        DoCalculations();
                        cout << "- Y: " << Y << endl;
                    }
                }
            }
        }
    }
    //Done
    SetDAC(0,2.47);
	SetDAC(1,2.47);
	Sleep(1000);
    cout << "Done\n\n";
}

// Function    : DoCalculations
// Returns     : void
// Parameters  : 
// Description : Reads the wheelchair's wheel encoders and calls WheelChairPoseGlobal() with new encoder data
void DoCalculations() {
    long * R_en = new long[2];
    long * L_en = new long[2];
    clock_t prevTime=0;
    double penc1 = 0, penc2 = 0;   //previous encoder readings (penc1 is RIGHT, penc2 is LEFT)
    double Time = 0.0;
    double xdot = 0.0, phidot = 0.0;

    Sleep(1);

    //Update the current time and save the previous time
	prevTime = currentTime ;
	currentTime = clock() ;
	Time = (currentTime - prevTime) ;
	Time=Time/1000.0;

	//Update the encoder counts for the left and right wheels
	penc1 = encoder1;
	penc2 = encoder2;

	//Read values from encoders
	GetTimers(encoder1, encoder2);
	encoder1 = -1*encoder1;          // To get both of them as positive value

    //cout << "Encoder Readings:  R = " << encoder1 << "    L = " << encoder2 << endl;
			
	//Store in an array
	R_en[0] = encoder1;
	R_en[1] = penc1;
	L_en[0] = encoder2;
	L_en[1] = penc2;
			
	//Compute the linear velocity, angular velocity, and the pose of the wheelchiar
	WheelChairPoseGlobal(Pose, Time, R_en,  L_en, CurrentPose, xdot, phidot);

    //Save the current wheelchair pose
    Pose[0] = CurrentPose[0];
    Pose[1] = CurrentPose[1];
    Pose[2] = CurrentPose[2];

    // Update the global variable X, Y and Phi
    PPhi = CurrentPose[2];  // Orientation angle 
    X = CurrentPose[0];		// Translation
    Y = CurrentPose[1];
}
