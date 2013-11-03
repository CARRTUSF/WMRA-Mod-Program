#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H


#include "matrix.h"  
#include "optimization.h"
#include "kinematics.h"
#include "Utility.h"
#include <limits>

Matrix WMRA_Opt(int i, double JLA, double JLO, Matrix Jo, double detJo, Matrix dq, vector<double> delta, double dt, vector<double> cur);
void WMRA_Jlimit(Matrix& qmin, Matrix& qmax);
Matrix WMRA_Opt(Matrix Jo, double detJo, vector<double> dx, vector<double> q);

#endif;