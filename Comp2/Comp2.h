// Library for Forward and Inverse Kinematics
// By Monica Li, Feb 2017

#ifndef COMP2_H
#define COMP2_H

#include <tuple>	// For multiple return values
#include "../Comp1/Comp1.h"

#ifndef ENSC488_H
#define ENSC488_H
#include "../ensc-488.h"
#endif /* ENSC488_H */

//Define structure parameters
#define L1 405
#define L2 70
#define L3 195
#define L4 142
#define L5 140
#define L6 80
#define L7 130
#define L8 10
#define Lmax 410

// KIN: Compute the forward kinematics of the robot
// Input: Joint variables 
// Output: Wrist Frame 
frameParam_t KIN(JOINT &conf);
 
// WHERE: Compute the forward kinematics of the robot
// Input: Joint variables 
// Output: Tool Frame 
frameParam_t WHERE(JOINT &conf);

#endif // !COMP2_H
