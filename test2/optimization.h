#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H


#include "matrix.h"  
#include "optimization.h"
#include "kinematics.h"
#include "Utility.h"
#include <limits>

Matrix WMRA_Opt(int i, float JLA, float JLO, Matrix Jo, float detJo, Matrix dq, vector<double> delta, float dt, vector<double> cur);
void WMRA_Jlimit(Matrix& qmin, Matrix& qmax);
Matrix WMRA_Opt(Matrix Jo, float detJo, vector<double> dx, vector<double> q);

#endif;