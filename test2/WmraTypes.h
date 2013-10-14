#ifndef WMRATYPES_H
#define WMRATYPES_H

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
   public:
      int x;
      int y;
      int z;
      double yaw;
      double pitch;
      double roll;    
      Pose(int _x =0, int _y =0, int _z=0, double _yaw=0, double _pitch=0, double _roll=0){
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
#endif;
