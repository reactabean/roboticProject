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
#include <conio.h>
#include <windows.h> //this is for sleep
#include <fstream> // for exporting data
#include "../Constants.h" // this is for the joint limits
#include "../ControllerandEmulator/controller.h"
using namespace std;


struct cubicCoef{
	double a,b,c,d;
};

struct cubicJoints {
	cubicCoef theta1,theta2,d3,theta4;
};

bool movetraj(JOINT &start, JOINT &first, JOINT &second,JOINT &third,JOINT &final, double diftime,bool manualControl);

void printJointToFile(ofstream &outputFile, JOINT &toPrint, double time);
cubicCoef * calculateCubicSpline(cubicJoints &jointArray, int size);
void printCubicCoef(ofstream &outputFile, cubicCoef myCoef);
void printSplineToFile(ofstream &outputFile, cubicJoints myJoint);

//need to write
void calcCubicPos(JOINT &pos,cubicJoints &block, double time);
void calcCubicVel(JOINT &vel,cubicJoints&block, double time, double timeslice);
void calcCubicAcc(JOINT &acc,cubicJoints &block, double time, double timeslice);
bool checkCubicValues(JOINT &position,JOINT &velocity,JOINT &acceleration);


#endif //!TRAJ