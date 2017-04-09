// Library for dynamic simulator
// By Monica Li, March 2017

#ifndef DYNAMICSIM_H
#define DYNAMICSIM_H

#ifndef ENSC488_H
#define ENSC488_H
#include "../ensc-488.h"
#endif /* ENSC488_H */

#include <stdio.h> //this is for printf 
#include <iostream> // this is for cout
#include <time.h>
#include <windows.h> //this is for sleep
#include <math.h> //this is for sin, cos
#include "../Comp1/Comp1.h"
#include "../Traj/Traj.h" 
using namespace std;

// According to exercise 6.2
void update(JOINT &tau, JOINT &pos, JOINT &vel, JOINT &acc, double period, ofstream& myfile);
void printPosVelAccToFile(ofstream &outputFile, JOINT &pos, JOINT &vel, JOINT &acc, double time, JOINT &tau);
void Ffun (JOINT &F, JOINT &velocity);
HomoMat Mfun(JOINT &position, JOINT &velocity);
void Vfun(JOINT &V, JOINT &position, JOINT &velocity);
void Gfun( JOINT &G);
void jointposMaxing( JOINT&pos, JOINT&position);
void torquecheck( JOINT& tau);

#endif //!DYNAMICSIM_H
