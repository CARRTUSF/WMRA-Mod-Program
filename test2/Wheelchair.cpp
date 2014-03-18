
#include "Wheelchair.h"

Wheelchair::Wheelchair()
{
	InitializeU6();
    cout << "Lab Jack U6 initialized..." << endl;
    InitializeTimersU6();
    cout << "Lab Jack Timers initialized..." << endl;

    cout << "Power the wheelchair ....." << endl;

    // Set the ideal value of the translation and rotation of the wheelchair
    SetDAC(0,2.47);
    SetDAC(1,2.47);
    Sleep(5000);
}



// Function    : Drive
// Returns     : void
// Parameters  : vector of Commands to issue the wheelchair
// Description : Analyzes each command and sends voltages to the wheelchairs controls
void Wheelchair::Drive(vector<Command> Commands) {
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

// Function    : WheelChairPoseGlobal
// Returns     : void
// Parameters  : PreviousPose of wheelchair, time between readings, R and L array of encoder readings, CurrentPose
// Description : Calculates the current pose of the wheelchair based on the encoder data
//               X = x-coordinate position        Y = y-coordinate position        PPhi = orientation angle
void Wheelchair::WheelChairPoseGlobal(double* PreviousPose, double time, long* R_en, long* L_en,double* CurrentPose,double & xdot, double & phidot) {
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


// Function    : DoCalculations
// Returns     : void
// Parameters  : 
// Description : Reads the wheelchair's wheel encoders and calls WheelChairPoseGlobal() with new encoder data
void Wheelchair::DoCalculations() {
    long * R_en = new long[2];
    long * L_en = new long[2];
    clock_t prevTime=0;
    double penc1=0, penc2=0;   //previous encoder readings (penc1 is RIGHT, penc2 is LEFT)
    double Time=0.0;
    double xdot=0.0, phidot=0.0;

    Sleep(1);

    //Update the current time and save the previous time
	prevTime = currentTime;
	currentTime = clock ();
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