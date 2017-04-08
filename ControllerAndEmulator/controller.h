// Library for Controller
// By Andrew Nichol, March 2017

#ifndef CONT_H
#define CONT_H


#ifndef ENSC488_H
#define ENSC488_H
#include "../ensc-488.h"
#endif /* ENSC488_H */


#include <time.h>
#include <iostream> // this is for cout
#include <conio.h>
#include "../DynamicSim/DynamicSim.h"
#include <windows.h> //this is for sleep
#include "../Constants.h" // this is for the joint limits
using namespace std;

//this funtion will take in a input and using control scheme drive a motion, it will keep trying to achieve the given position in a timespan.
//inputs: time and initial velocity 
bool moveCont(JOINT &conf, JOINT &vel, JOINT &acc, JOINT &tVel);


//TODO:remove comments below 
//note to self
//need to somehow retain state between commands/
//should have a timeout
//should keep within a threshold

#endif //!TRAJ