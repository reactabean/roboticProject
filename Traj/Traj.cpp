// Library for Forward and Inverse Kinematics
// By Andrew Nichol, March 2017

#include "Traj.h"


//this function is to traverse between a set of inputted joints using a cubic spline
bool movetraj(JOINT &start, JOINT &first, JOINT &second,JOINT &third,JOINT &final, double diftime)
{
	//todo: dump ideal positions to csv
	//todo: note I am setting the velocity at via points to be zero this is a bit ugly maybe rethink cubic spline interpolation
	//todo: clean up this function
	//todo: finish writing this funtion
	//todo: test this funtion
	
	//local variables
	JOINT position,velocity,acceleration;
	double timeslice,time;
    cubicCoef temp;
	bool error;
	cubicJoints block1,block2,block3,block4;
	time_t before, after;

	//evenly distributed time needed to traverse between via points
	timeslice =  diftime / 4;
	
	//STAGE 1 INTERPOLATION :calculate coefficents cubic inbetween via points 
	//note a block correponds to the space betweent via points

	//block1========================
	temp.a = start[0];
	temp.b = 0;
	temp.c =3*(first[0]-start[0]);
	temp.d =-2*(first[0]-start[0]);
	block1.theta1 = temp;

	temp.a = start[1];
	temp.b = 0;
	temp.c =3*(first[1]-start[1]);
	temp.d =-2*(first[1]-start[1]);
    block1.theta2 = temp;
	
	temp.a = start[2];
	temp.b = 0;
	temp.c =3*(first[2]-start[2]);
	temp.d =-2*(first[2]-start[2]);
	block1.d3 = temp;

	temp.a = start[3];
	temp.b = 0;
	temp.c =3*(first[3]-start[3]);
	temp.d =-2*(first[3]-start[3]);
	block1.theta4 = temp;

	//block2===========================
	temp.a = first[0];
	temp.b = 0;
	temp.c =3*(second[0]-first[0]);
	temp.d =-2*(second[0]-first[0]);
	block2.theta1 = temp;

	temp.a = first[1];
	temp.b = 0;
	temp.c =3*(second[1]-first[1]);
	temp.d =-2*(second[1]-first[1]);
    block2.theta2 = temp;
	
	temp.a = first[2];
	temp.b = 0;
	temp.c =3*(second[2]-first[2]);
	temp.d =-2*(second[2]-first[2]);
	block2.d3 = temp;

	temp.a = first[3];
	temp.b = 0;
	temp.c =3*(second[3]-first[3]);
	temp.d =-2*(second[3]-first[3]);
	block2.theta4 = temp;

	//block3 ===========================
	temp.a = second[0];
	temp.b = 0;
	temp.c =3*(third[0]-second[0]);
	temp.d =-2*(third[0]-second[0]);
	block3.theta1 = temp;

	temp.a = start[1];
	temp.b = 0;
	temp.c =3*(third[1]-second[1]);
	temp.d =-2*(third[1]-second[1]);
    block3.theta2 = temp;
	
	temp.a = start[2];
	temp.b = 0;
	temp.c =3*(third[2]-second[2]);
	temp.d =-2*(third[2]-second[2]);
	block3.d3 = temp;

	temp.a = start[3];
	temp.b = 0;
	temp.c =3*(third[3]-second[3]);
	temp.d =-2*(third[3]-second[3]);
	block3.theta4 = temp;

	//block4 ===========================
	temp.a = third[0];
	temp.b = 0;
	temp.c =3*(final[0]-third[0]);
	temp.d =-2*(final[0]-third[0]);
	block4.theta1 = temp;

	temp.a = start[1];
	temp.b = 0;
	temp.c =3*(final[1]-third[1]);
	temp.d =-2*(final[1]-third[1]);
    block4.theta2 = temp;
	
	temp.a = start[2];
	temp.b = 0;
	temp.c =3*(final[2]-third[2]);
	temp.d =-2*(final[2]-third[2]);
	block4.d3 = temp;

	temp.a = start[3];
	temp.b = 0;
	temp.c =3*(final[3]-third[3]);
	temp.d =-2*(final[3]-third[3]);
	block4.theta4 = temp;

	//STAGE 2: SAMPLE AND TRAVERSE PATH   // !!!!this stage has not been fully implemented!!!!
	///=======================================needs work below this=============================

	before = clock();
	after = clock();

	//block1
	while (diftime > difftime (after,before)){
	
	//todo: dump current position to csv

	//calculate t to use in the cubic position calcualtion - goes from 0 to one over the block
	time = (difftime (after,before) / diftime); //does diff time have enough resolution?

	//calulate move inputs and check they are not violating limits
	calcCubicPos(position,block1,time);
	calcCubicVel(position,block1,time);
	calcCubicAcc(acceleration,block1,time);
	checkCubicvalues(position,velocity,acceleration);

	//send move
	MoveWithConfVelAcc(position,position,acceleration);
	
	//wait sample time
	Sleep(20);

	//update clock
	after = clock();		
	}
	
	//block2
	while (diftime > difftime (after,before)){
	//todo: fill in
	}

	//block3
	while (diftime > difftime (after,before)){
	//todo: fill in
	}

	//block4
	while (diftime > difftime (after,before)){
	//todo: fill in
	}

	return error;
}


void calcCubicPos(JOINT &start,cubicJoints &block, double t){
	// = a + bt + ct^2 + bt^3
	start[0] = block.theta1.a+block.theta1.b*t + block.theta1.c*t*t + block.theta1.d*t*t*t;  
	start[1] = block.theta2.a+block.theta2.b*t + block.theta2.c*t*t + block.theta2.d*t*t*t;  
	start[2] = block.d3.a+block.d3.b*t + block.d3.c*t*t + block.d3.d*t*t*t;  
	start[3] = block.theta4.a+block.theta4.b*t + block.theta4.c*t*t + block.theta4.d*t*t*t;  
}

void calcCubicVel(JOINT &start, cubicJoints &block, double t){
	// = b + ct + bt^2
	start[0] = block.theta1.b + block.theta1.c*t + block.theta1.d*t*t;  
	start[1] = block.theta2.b + block.theta2.c*t + block.theta2.d*t*t;  
	start[2] = block.d3.b + block.d3.c*t + block.d3.d*t*t;  
	start[3] = block.theta4.b + block.theta4.c*t + block.theta4.d*t*t;  
}

void calcCubicAcc(JOINT &start, cubicJoints &block, double t){
	// = c + dt
	start[0] =  block.theta1.c + block.theta1.d*t;  
	start[1] =  block.theta2.c + block.theta2.d*t;  
	start[2] =  block.d3.c + block.d3.d*t*t*t;  
	start[3] =  block.theta4.c+ block.theta4.d*t;  
}

void checkCubicvalues(JOINT &position,JOINT &velocity,JOINT &acceleration){
	//check and make sure they do not violate joint limits
	//Todo: write the function
}

//for reference--- delete on final copy
//joint Limits
//[-150, 150]
//[-100, 100]
//[-200,-100]
//[-160,160]

//vel limits
//[-150, 150]
//[-150, 150]
//[-50, 50]
//[-150, 150]

//acc limits
//[-600 600]
//[-600 600]
//[-200 200]
//[-600 600]