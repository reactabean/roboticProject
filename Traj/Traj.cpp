// Library for Forward and Inverse Kinematics
// By Andrew Nichol, March 2017

#include "Traj.h"



bool movetraj(JOINT &start, JOINT &first, JOINT &second,JOINT &third,JOINT &final, double diftime)
{
	JOINT position,velocity,acceleration;
	double timeslice,time;
    cubicCoef temp;
	bool error;
	cubicJoints block1,block2,block3,block4;
	timeslice =  diftime / 4;
	time_t before, after;
	
	//todo: note I am setting the velocity at via points to be zero this is a bit ugly 
	//calculate coefficents cubic inbetween via points --interpolation stage

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

	 //block3
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

	 //block4

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

	//start traversal //remember to dump to csv file for proper output
	
	///=======================================needs work below this=============================
	before = clock();
	after = clock();

	//block1
	while (diftime > difftime (after,before)){
	
	//dump current position to csv

	//calculate t to use in the cubic position calcualtion - goes from 0 to one over the block
	time = (difftime (after,before) / diftime);

	//calulate move inputs and check they are not violating limits
	calcCubicPos(position,block1);
	calcCubicVel(position,block1);
	calcCubicAcc(acceleration,block1);
	checkCubicvalues(position,velocity,acceleration);

	//dump ideal positions to csv

	//send move
	MoveWithConfVelAcc(position,position,acceleration);
	
	

	//wait sample time
		//sleep for 20ms
	
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

