// Library for Forward and Inverse Kinematics
// By Monica Li, Feb 2017

#ifndef COMP2_H
#define COMP2_H

#include <tuple>	// For multiple return values
#include "Comp1.h"
#include "ensc-488.h"

// KIN: Compute the forward kinematics of the robot
// Input: Joint variables 
// Output: Wrist Frame 
frameParam_t KIN(JOINT &conf);
 
// WHERE: Compute the forward kinematics of the robot
// Input: Joint variables 
// Output: Tool Frame 
frameParam_t WHERE(JOINT &conf);

// INVKIN: Calculate inverse kinematics for the robot
// Input: Wrist Frame specifed relative to the station
// Output: Nearest solution, second solution, flag of solution
std::tuple<JOINT, JOINT, bool> INVKIN(frameParam_t &wrist);

// SOLVE: Calculate inverse kinematics for the robot
// Input: Tool Frame specifed relative to the station
// Output: Nearest solution, second solution, flag of solution
std::tuple<JOINT, JOINT, bool> SOLVE(frameParam_t &tool);

#endif // !COMP2_H
