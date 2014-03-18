#include <iostream>
#include "WheelchairControl.h"


using namespace std;


int WheelchairControl::instances = 0;
WheelchairPose WheelchairControl::currentPose = {0,0,0};
bool WheelchairControl::writingPose = false;

WheelchairControl::WheelchairControl(void)
{
		InitializeU6();
		cout << "Lab Jack U6 initialized..." << endl;
		InitializeTimersU6 ();
		cout << "Lab Jack Timers initialized..." << endl;
		cout << "Power the wheelchair ....." << endl;
		// Set the ideal value of the translation and rotation of the wheelchair
		SetDAC(0,2.47);  // rotate
		SetDAC(1,2.47);  // translate
		Sleep(5000);
		//start the pose calculating function
		startWCPoseCalculationthread();	
	
}


WheelchairControl::~WheelchairControl(void)
{
}

bool WheelchairControl::StopWheelChair()
{
		SetDAC(0,2.47);
		SetDAC(1,2.47);
		Sleep(20);
		return true;
}

WheelchairPose WheelchairControl::getPose()
{
	while(writingPose); // dont read while value is being updated
	return currentPose;
}

bool WheelchairControl::rotateLeft(double deltaRadians)
{
	if(deltaRadians < M_PI/8) return false; // do not turn for small angles
	double TargetPhi = currentPose.phi + deltaRadians; // turning left , so the global angle will increase
	SetDAC(0,2.05);
	while( currentPose.phi < TargetPhi ) ; //cout << "current Pose: " << currentPose.phi << endl ;
	SetDAC(0,2.47);
	cout << "Rotate Left ended with Phi = " << currentPose.phi << endl;
	return true;
}

bool WheelchairControl::rotateRight(double deltaRadians)
{
	if(deltaRadians < M_PI/8) return false; // do not turn for small angles
	double TargetPhi = currentPose.phi - deltaRadians; // turning right, so the angle decreases
	SetDAC(0,2.85);
	while( currentPose.phi > TargetPhi ); // cout << "current Pose: " << currentPose.phi << endl ;
	SetDAC(0,2.47);
	cout << "Rotate Left ended with Phi = " << currentPose.phi << endl;
	return true;
}

bool WheelchairControl::moveTo(double targetX, double targetY)
{
	// figure out how much to turn to 
	// face the direction we need to go

    cout << "moving to : "  << targetX << " , " << targetY <<endl;

	double deltaX = targetX - currentPose.x ;
	double deltaY = targetY - currentPose.y  ;
	double desiredAngle = atan2(deltaY, deltaX);

	double curAngle = fmod(currentPose.phi, 2*M_PI);

	if(curAngle < -1*M_PI || curAngle > M_PI)
	{
		curAngle = -1 * (2*M_PI - abs(curAngle) );
	}

	// add offset PI to chane range to 0- 2*PI from -Pi to PI 
	double desiredAngle0 = desiredAngle + M_PI; 
	curAngle = curAngle + M_PI;


	if(curAngle > desiredAngle0 ) // 
	{
		if (curAngle - desiredAngle0 > M_PI )
		{
			// move to the right by 2*M_PI - (curAngle - desiredAngle0 ) 
			double TargetAngle = 2*M_PI - (curAngle - desiredAngle0 );
			rotateLeft(TargetAngle);

		}
		else
		{
			//move to the left
			double TargetAngle = curAngle - desiredAngle0 ;
			rotateRight(TargetAngle);
		}
	}

	else // curAngle < desiredAngle
	{
		if (desiredAngle0 - curAngle > M_PI )
		{
			// move to the left
			double TargetAngle = 2*M_PI - ( desiredAngle0 -curAngle );
			rotateRight(TargetAngle);
		}
		else
		{
			//move to the left
			double TargetAngle = desiredAngle0 -curAngle   ;
			rotateLeft(TargetAngle);
		}
	}


	// done rotatting

	//now translate.

	// NOTEFORLATER  choose best index to check for . ie X or Y

    curAngle = fmod(currentPose.phi, 2*M_PI);
	if(curAngle < -1*M_PI || curAngle > M_PI)
	{
		curAngle = -1 * (2*M_PI - abs(curAngle) );
	}

    // 0.785398163 = 45 degrees
    // 135 degrees = 2.35619449 radians


    if( (curAngle > 0.785398163 && curAngle < 2.35619449)  || (curAngle < -0.785398163  && curAngle > -2.35619449) )
    {
        moveToY(targetY);
    }
    else
    {
	    moveToX(targetX);
    }

	return true;

}


// this function assumes the wheelchair is already facing the correct direction
// and will move forward until the desired X direction in global frame is reached.
// if the wheelchair is not facing the correct way then this function will never terminate.
// Use with caution
bool WheelchairControl::moveToX(double targetX)
{
	cout << "MoveX Started  with Phi = " << currentPose.phi << endl;
	WheelchairPose cur = currentPose;
	if (targetX < cur.x)
	{
		SetDAC(0, 2.47);
        SetDAC(1, 2.85);
		while( currentPose.x > targetX  )
		{
			Sleep(20);
		}
		SetDAC(0, 2.47);
        SetDAC(1, 2.47);
		return true;
	}
	else if (targetX > cur.x)
	{
		SetDAC(0, 2.47);
        SetDAC(1, 2.85);
		while( currentPose.x < targetX  )
		{
			Sleep(20);
		}
		SetDAC(0, 2.47);
        SetDAC(1, 2.47);
		return true;
	}
	return false;
}

// this function assumes the wheelchair is already facing the correct direction
// and will move forward until the desired Y direction in global frame is reached.
// if the wheelchair is not facing the correct way then this function will never terminate.
// Use with caution
bool WheelchairControl::moveToY(double targetY)
{
	
	WheelchairPose cur = currentPose;
	if (targetY < cur.y)
	{
		SetDAC(0, 2.47);
        SetDAC(1, 2.85);
		while( targetY < currentPose.y)
		{
			Sleep(20);
		}
		SetDAC(0, 2.47);
        SetDAC(1, 2.47);
		return true;
	}
	else if (targetY > cur.y)
	{
		SetDAC(0, 2.47);
        SetDAC(1, 2.85);
		while( targetY > currentPose.y)
		{
			Sleep(20);
		}
		SetDAC(0, 2.47);
        SetDAC(1, 2.47);
		return true;
	}

	return false;

}

HANDLE WheelchairControl::startWCPoseCalculationthread()
{
    return CreateThread(NULL, 0, &WheelchairControl::wheelchairPose, (LPVOID*)this, 0, NULL);
}


DWORD WINAPI WheelchairControl::wheelchairPose(LPVOID pSelf)
{
	double X = 0.0;
	double Y = 0.0;
	double L1 = 0.555; //55.9;
	double WheelRad = 0.17; //16.8;
	int RevEncoder_R = 1580;
	int RevEncoder_L = 1550;

	double Theta_R = 0;
	double Theta_L = 0;
	double Theta_RT = 0;
	double Theta_LT = 0;
	double Translation = 0;
	double phi1 = 0;

	clock_t currentTime=0, prevTime=0; //current time to update during while loop and previous time


	double time=0, encoder=0;
	double encoder1=0, encoder2=0, R_en=0, L_en=0,  penc1=0, penc2=0; //encoder readings and previous encoder readings (enc1 is RIGHT, enc2 is LEFT);
	double xdot=0, phidot=0;
	
	double *Pose = new double[3];

	Pose[0]=0;
	Pose[1]=0;
	Pose[2]=0;

	while (true)
	{
		// read in the current pose
		Pose[0]= currentPose.x;
		Pose[1]= currentPose.y;
		Pose[2]= currentPose.phi;

		// 	Update the current time and save the previous time
		prevTime = currentTime;			//save previous time
		currentTime = clock ();			//update the current time
		time = (currentTime - prevTime) ; // time of the iteration
		time=time/1000.0;

		//Update the encoder counts for the left and right wheels
		penc1 = encoder1; //save previous position of the right wheel encoder
		penc2 = encoder2; //save previous position of rhe left wheel encoder

		//Read values from encoders
		GetTimers(encoder, encoder2);
		encoder1 = -1*encoder; // To get both of them as positive value

		// Cast the previous and current wheelencoders for right and left wheel in an array

		R_en = encoder1 - penc1 ;
		L_en = encoder2 - penc2;



		// compute the linear velocity, angular velocity, and the pose of the wheelchiar from encoders reading.
		// The rotated angle for the left and right wheel
		Theta_R = 2*M_PI*R_en / RevEncoder_R;
		Theta_L = 2*M_PI*L_en / RevEncoder_L;

		// Translation of the wheelchair 
		Translation = (WheelRad/2)*(Theta_L + Theta_R);
		xdot = Translation/ time;

		// rotation angle of the wheelchiar
		phi1 = (WheelRad/L1)*(-Theta_L + Theta_R);
		phidot = phi1/time;


		if ( abs(phi1) <= 0.005) // striaght line movment rotation angle almost zero
		{
			X = Translation*cos(Pose[2]) + Pose[0];
			Y = Translation*sin(Pose[2]) + Pose[1];


		}
		else // There is a rotation angle 
		{
			double r = Translation/phi1 - L1/2; // raduis of curvature
			X = (sin((M_PI/2)  + Pose[2] + (phi1/2))*(r+(L1/2))*((sin(phi1)/cos(phi1/2)))) + Pose[0];
			Y = (-cos((M_PI/2) + Pose[2] + (phi1/2))*(r+(L1/2))*((sin(phi1)/cos(phi1/2)))) + Pose[1];


		}

		double Phi = Pose[2] + phi1; // the new oreintation angle



		// return the values of the current wheelchair pose as the previuos Wheelchair Pose for next iteration
		Pose[0] = X;
		Pose[1] = Y;
		Pose[2] = Phi;

		// update current pose
		writingPose = true; // aquire lock on variable

		currentPose.x = X;
		currentPose.y = Y;
		currentPose.phi = Phi;

		writingPose = false; // release  lock on variable

		//Sleep(20); // Sleep for 20 milliseconds before the next iteration

	}

	return 0;

} 
