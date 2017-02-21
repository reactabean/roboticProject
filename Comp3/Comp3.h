// Library for Forward and Inverse Kinematics
// By Adrian Fettes, Feb 2017

#ifndef COMP3_H
#define COMP3_H

#include "../Comp2/Comp2.h"

#ifndef ENSC488_H
#define ENSC488_H
#include "../ensc-488.h"
#endif /* ENSC488_H */

struct jointReturn {
	JOINT config1;
	JOINT config2;
	bool configValid;
};

// INVKIN: Compute the inverse kinematics of the robot
// Input: Wrist Frame 
// Output: Array containing 2 joint variables

jointReturn INVKIN(frameParam_t &wrist);

// SOLVE: Calculate inverse kinematics for the robot
// Input: Tool Frame specifed relative to the station
// Output: Array containing 2 joint variables
jointReturn SOLVE(frameParam_t &tool);

#endif //!COMP3_H