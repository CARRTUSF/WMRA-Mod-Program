/**
This file contains declarations of data types used by the WMRA program

Written by Indika Pathirage
Distributed under GPL2 or higher

**/

#include <vector>
#include "matrix.h"

using namespace math;

namespace WMRA{

   
   class Pose{
      int x;
      int y;
      int z;
      double yaw;
      double pitch;
      double roll;    
      Pose(int _x, int _y, int _z, double _yaw, double _pitch, double _roll){
         x = _x; y = _y; z = _z; pitch = _pitch; yaw = _yaw ; roll = _roll;
      }
   };

   struct WheelChairPose{
		int x,y;
		float angle;
		matrix<double> transformation;
	};

   class JointValueSet{
      vector<double> Joint;
      JointValueSet(){
         Joint.resize(8);
      }
      double operator[](int i){
         if(i < Joint.size())  return Joint[i];
      }
   };
}
