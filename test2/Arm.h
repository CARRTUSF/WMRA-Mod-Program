#ifndef ARM_H
#define ARM_H

#include <vector>
#include <fstream>
#include "SockStream.h"
#include <windows.h> <WINDOWS.H>
#include <strsafe.h> <STRSAFE.H>
#include "WmraTypes.h"
#include "MotorController.h"
#include "optimization.h"

class Arm{
public:
	Arm();
	//WMRA::Pose getPosition();
	bool initialize();
	bool setDefaults();
	bool moveArm(vector<double> destinationAng);
	bool milestoneDelta(vector<double> destinationAng, double dt);
	bool autonomous(WMRA::Pose dest, WMRA::CordFrame crodFr=WMRA::ARM_FRAME_REL);
   bool autonomous2(WMRA::Pose dest, WMRA::CordFrame crodFr=WMRA::ARM_FRAME_REL);
   bool openGripper();
   bool closeGripper();
	void closeDebug();
	bool toReady();
	bool ready2Park();
	bool park2Ready();
   /// sets the transformation from end of kinematic chain to a
   /// arbitary tooltip position. This method can be used to get the 
   /// end the position of ponts on the arm other than the standard gripper
   bool setTooltipTransform(Matrix t){
      tooltipTf = t;
      return true;
   }
   WMRA::Pose getPose();
	WMRA::JointValueSet getJointAngles();

private:


   bool autonomousMove(Matrix start, Matrix dest);
	double dt;	// the default time between milestones
	double dt_mod;	// the default time between milestones
	int control_type; // WMRA Control type; 0 = Simulation, 1 = Wheelchair Only, 2 = Arm Only, 3 = Both Arm and Wheelchair. [Prev=WCA]
	int control_frame; // WMRA Control frame;  2 = Wheelchair Frame [Prev=coord]
	int control_cart; // WMRA Control cartesian type; 1 = Position and Orientation [Prev cart]
	int control_optim; // WMRA Control Optimization type; 1 = SR-I [Prev=optim]
	int control_JLA; //                    ; 1 = JLA applied, 0 = JLA not applied
	int control_JLO; // WMRA Joint Limit Obstacle Avoidence; 1 = JLO applied, 0 = JLO not applied
	int control_cont; // WMRA Control method; 1 = Position, 2 = Velocity, 3 = Spaceball
	double control_velocity; // WMRA Desired Gripper Velocity; default=50
   double maxAngularVelocity;
	int control_traj; // WMRA Control Trajectory type; 1 = Polynomial with Blending
	int vr, ml; // Debug: Not sure what these are for, something to do with animations default: vr=0, ml=0
	int control_arm; // Arm(run) the WMRA; 1 = Armed, 0 = Disarmed
	int control_ini; // Does the arm need to move from park to ready position; 0=no, 1=yes
	int plt; // Debug: Not sure what this veriable is for, something to do with simulation results; 1 = no results

   Matrix gripperInitRotDiff;
   Matrix tooltipTf;
	WMRA::JointValueSet readyPosition; //joint angles for ready position

	std::ofstream xyz_way; // Waypoint XYZ values
	std::ofstream xyz_sent; // command XYZ values
	std::ofstream xyz_cont; // command XYZ values
   std::ofstream jointVel; // joint velocity values
	bool initialized;
	MotorController controller; 
};
#endif;