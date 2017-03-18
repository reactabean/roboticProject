// Library for trajectory
// By Andrew Nichol, March 2017

#ifndef TRAJ_H
#define TRAJ_H


#ifndef ENSC488_H
#define ENSC488_H
#include "../ensc-488.h"
#endif /* ENSC488_H */

#include <time.h>
#include <iostream> // this is for cout


struct cubicCoef{
	double a,b,c,d;
};

struct cubicJoints {
	cubicCoef theta1,theta2,d3,theta4;
};

bool movetraj(JOINT &start, JOINT &first, JOINT &second,JOINT &third,JOINT &final, double diftime);

//need to write
calcCubicPos(JOINT &start,cubicJoints &block);
calcCubicVel(JOINT &start, &block);
calcCubicAcc(JOINT &start, &block);
checkCubicvalues(JOINT &position,JOINT &velocity,JOINT &acceleration);


#endif //!TRAJ
