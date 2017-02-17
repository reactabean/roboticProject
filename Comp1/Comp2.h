// Library for Forward and Inverse Kinematics
// By Monica Li, Feb 2017

#ifndef COMP2_H
#define COMP2_H

#include <conio.h>
#include "Comp1.h"
#include "ensc-488.h"

// KIN: Compute the forward kinematics of the planar 3R robot
// Input: Joint variables 
// Output: Wrist Frame 
frameParam_t KIN(JOINT &conf);
 
// WHERE: Compute the forward kinematics of the planar 3R robot
// Input: Joint variables 
// Output: Tool Frame 
frameParam_t WHERE(JOINT &conf);


#endif // !COMP2_H
