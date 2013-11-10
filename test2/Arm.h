#ifndef ARM_H
#define ARM_H

#include <vector>
#include <fstream>
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
	bool autonomous(WMRA::Pose dest, WMRA::CordFrame crodFr=WMRA::ARM_FRAME);
	void closeDebug();
	WMRA::JointValueSet getJointAngles();

private:
	double dt;	// the default time between milestones
	double dt_mod;	// the default time between milestones
	int control_type; // WMRA Control type; 0 = Simulation, 1 = Wheelchair Only, 2 = Arm Only, 3 = Both Arm and Wheelchair. [Prev=WCA]
	int control_frame; // WMRA Control frame;  2 = Wheelchair Frame [Prev=coord]
	int control_cart; // WMRA Control cartesian type; 1 = Position and Orientation [Prev cart]
	int control_optim; // WMRA Control Optimization type; 1 = SR-I [Prev=optim]
	int control_JLA; //                    ; 1 = JLA applied, 0 = JLA not applied
	int control_JLO; // WMRA Joint Limit Obstacle Avoidence; 1 = JLO applied, 0 = JLO not applied
	int control_cont; // WMRA Control method; 1 = Position, 2 = Velocity, 3 = Spaceball
	int control_velocity; // WMRA Desired Gripper Velocity; default=50
	int control_traj; // WMRA Control Trajectory type; 1 = Polynomial with Blending
	int vr, ml; // Debug: Not sure what these are for, something to do with animations default: vr=0, ml=0
	int control_arm; // Arm(run) the WMRA; 1 = Armed, 0 = Disarmed
	int control_ini; // Does the arm need to move from park to ready position; 0=no, 1=yes
	int plt; // Debug: Not sure what this veriable is for, something to do with simulation results; 1 = no results

	WMRA::JointValueSet readyPosition; //joint angles for ready position

	std::ofstream xyz_way; // Waypoint XYZ values
	std::ofstream xyz_sent; // command XYZ values
	std::ofstream xyz_cont; // command XYZ values
	bool initialized;
	static MotorController controller; 
	Opt opt;
};
#endif;