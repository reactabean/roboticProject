// Library for Forward and Inverse Kinematics
// Created by Andrew Nichol, editted by Monica Li, March 2017


#include "Traj.h"
using namespace std;


//this function is to traverse between a set of inputted  joints using a cubic spline
bool movetraj(JOINT &start, JOINT &first, JOINT &second,JOINT &third,JOINT &final, double diftime,bool manualControl)
{
	//todo: dump ideal positions to csv
	//todo: note I am setting the velocity at via points to be zero this is a bit ugly maybe rethink cubic spline interpolation
	//todo: clean up this function
	//todo: finish writing this funtion
	//todo: test this funtion
	
	//local variables
	JOINT position,velocity,acceleration, currentPosition;
	JOINT finalvel, finalacc;
	JOINT actualVel = {0,0,0,0};
	double timeslice,time;
    cubicCoef temp;
	bool error;
	cubicJoints block1,block2,block3,block4;
	cubicJoints jointSet[4];
	time_t before, after;

	ofstream myfile,emufile,controlfile;
	myfile.open("currentPosition.csv");
	emufile.open("simulatorOutput.csv");
	myfile << "theta1, theta2, d3, theta4, time(ms) \n";
	emufile << "time(ms), theta1, theta2, d3, theta4, thetadot1, thetadot2, ddot3, thetadot4, thetadotdot1, thetadotdot2, ddotdot3, thetadotdot4  \n";

	//evenly distributed time needed to traverse between via points
	timeslice =  diftime / 4;	

	//At the end of trajectory, set the velocity and acceleration to 0
	finalvel[0] = 0;
	finalvel[1] = 0;
	finalvel[2] = 0;
	finalvel[3] = 0;
	finalacc[0] = 0;
	finalacc[1] = 0;
	finalacc[2] = 0;
	finalacc[3] = 0;
	
	//STAGE 1 INTERPOLATION :calculate coefficents cubic inbetween via points, with passing speed not as zero
	//note a block correponds to the space betweent via points

	//block1========================
	temp.a = start[0];
	temp.b = 0;
	temp.c = -(second[0]- start[0])/2 + 3 * (first[0] - start[0]);
	temp.d = (second[0] - start[0])/2 - 2 * (first[0] - start[0]);
	block1.theta1 = temp;

	temp.a = start[1];
	temp.b = 0;
	temp.c = -(second[1] - start[1])/2 + 3 * (first[1] - start[1]);
	temp.d = (second[1] - start[1])/2 - 2 * (first[1] - start[1]);
	block1.theta2 = temp;

	temp.a = start[2];
	temp.b = 0;
	temp.c = -(second[2] - start[2])/2 + 3 * (first[2] - start[2]);
	temp.d = (second[2] - start[2])/2 - 2 * (first[2] - start[2]);
	block1.d3 = temp;

	temp.a = start[3];
	temp.b = 0;
	temp.c = -(second[3] - start[3])/2 + 3 * (first[3] - start[3]);
	temp.d = (second[3] - start[3])/2 - 2 * (first[3] - start[3]);
	block1.theta4 = temp;

	//block2===========================
	temp.a = first[0];
	temp.b = (second[0] - start[0]) / 2;
	temp.c = -((third[0] - first[0]) / 2 + (second[0] - start[0])) + 3 * (second[0] - first[0]);
	temp.d = ((third[0] - first[0]) / 2 + (second[0] - start[0]) / 2) - 2 * (second[0] - first[0]);
	block2.theta1 = temp;

	temp.a = first[1];
	temp.b = (second[1] - start[1]) / 2;
	temp.c = -((third[1] - first[1]) / 2 + (second[1] - start[1])) + 3 * (second[1] - first[1]);
	temp.d = ((third[1] - first[1]) / 2 + (second[1] - start[1]) / 2) - 2 * (second[1] - first[1]);
	block2.theta2 = temp;

	temp.a = first[2];
	temp.b = (second[2] - start[2]) / 2;
	temp.c = -((third[2] - first[2]) / 2 + (second[2] - start[2])) + 3 * (second[2] - first[2]);
	temp.d = ((third[2] - first[2]) / 2 + (second[2] - start[2]) / 2) - 2 * (second[2] - first[2]);
	block2.d3 = temp;

	temp.a = first[3];
	temp.b = (second[3] - start[3]) / 2;
	temp.c = -((third[3] - first[3]) / 2 + (second[3] - start[3])) + 3 * (second[3] - first[3]);
	temp.d = ((third[3] - first[3]) / 2 + (second[3] - start[3]) / 2) - 2 * (second[3] - first[3]);
	block2.theta4 = temp;

	//block3 ===========================
	temp.a = second[0];
	temp.b = (third[0] - first[0]) / 2;
	temp.c = -((final[0] - second[0]) / 2 + (third[0] - first[0])) + 3 * (third[0] - second[0]);
	temp.d = ((final[0] - second[0]) / 2 + (third[0] - first[0]) / 2) - 2 * (third[0] - second[0]);
	block3.theta1 = temp;

	temp.a = second[1];
	temp.b = (third[1] - first[1]) / 2;
	temp.c = -((final[1] - second[1]) / 2 + (third[1] - first[1])) + 3 * (third[1] - second[1]);
	temp.d = ((final[1] - second[1]) / 2 + (third[1] - first[1]) / 2) - 2 * (third[1] - second[1]);
	block3.theta2 = temp;

	temp.a = second[2];
	temp.b = (third[2] - first[2]) / 2;
	temp.c = -((final[2] - second[2]) / 2 + (third[2] - first[2])) + 3 * (third[2] - second[2]);
	temp.d = ((final[2] - second[2]) / 2 + (third[2] - first[2]) / 2) - 2 * (third[2] - second[2]);
	block3.d3 = temp;

	temp.a = second[3];
	temp.b = (third[3] - first[3]) / 2;
	temp.c = -((final[3] - second[3]) / 2 + (third[3] - first[3])) + 3 * (third[3] - second[3]);
	temp.d = ((final[3] - second[3]) / 2 + (third[3] - first[3]) / 2) - 2 * (third[3] - second[3]);
	block3.theta4 = temp;

	//block4 ===========================
	temp.a = third[0];
	temp.b = (final[0] - second[0]) / 2;
	temp.c = -(final[0] - second[0]) + 3 * (final[0] - third[0]);
	temp.d = (final[0] - second[0]) / 2 - 2 * (final[0] - third[0]);
	block4.theta1 = temp;

	temp.a = third[1];
	temp.b = (final[1] - second[1]) / 2;
	temp.c = -(final[1] - second[1]) + 3 * (final[1] - third[1]);
	temp.d = (final[1] - second[1]) / 2 - 2 * (final[1] - third[1]);
	block4.theta2 = temp;

	temp.a = third[2];
	temp.b = (final[2] - second[2]) / 2;
	temp.c = -(final[2] - second[2]) + 3 * (final[2] - third[2]);
	temp.d = (final[2] - second[2]) / 2 - 2 * (final[2] - third[2]);
	block4.d3 = temp;

	temp.a = third[3];
	temp.b = (final[3] - second[3]) / 2;
	temp.c = -(final[3] - second[3]) + 3 * (final[3] - third[3]);
	temp.d = (final[3] - second[3]) / 2 - 2 * (final[3] - third[3]);
	block4.theta4 = temp;


	//STAGE 2: SAMPLE AND TRAVERSE PATH   // !!!!this stage has not been fully implemented!!!!
	///=======================================needs work below this=============================

	printJointToFile(myfile, start, 0);
	printJointToFile(myfile, first, 0);
	printJointToFile(myfile, second, 0);
	printJointToFile(myfile, third, 0);
	printJointToFile(myfile, final, 0);
	printSplineToFile(myfile, block1);
	printSplineToFile(myfile, block2);
	printSplineToFile(myfile, block3);
	printSplineToFile(myfile, block4);

	jointSet[0] = block1;
	jointSet[1] = block2;
	jointSet[2] = block3;
	jointSet[3] = block4;


	before = clock();
	after = clock();

	for (int section = 0; section < 4; section++) {
		//myfile << "block " << section+1 << " \n";
		myfile << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << endl; //mark block separation with blank lines
		while (timeslice > difftime(after, before)) {

			//todo: dump current joint position to csv
			GetConfiguration(currentPosition);

			//calculate t to use in the cubic position calcualtion - goes from 0 to one over the block
			time = difftime(after, before)/timeslice; //does diff time have enough resolution?

			printJointToFile(myfile, currentPosition, section*timeslice+difftime(after, before));
														  //calulate move inputs and check they are not violating limits
			calcCubicPos(position, jointSet[section], time);
			calcCubicVel(velocity, jointSet[section], time, timeslice/1000);
			calcCubicAcc(acceleration, jointSet[section], time, timeslice/1000);
			error = !checkCubicValues(position, velocity, acceleration);
			if (error) cout << "In block " << section+1 << endl;

		
			if (manualControl == true){
				moveCont(position, velocity, acceleration , actualVel,emufile);
			} else {
				//send move
				MoveWithConfVelAcc(position, velocity, acceleration);
				//wait sample time
				Sleep(10);
			}



			//update clock
			after = clock();
		}
		before = clock();
		after = clock();
	}

	MoveWithConfVelAcc(position, finalvel, finalacc);
	myfile.close();
	emufile.close();
	return error;
}


void calcCubicPos(JOINT &pos,cubicJoints &block, double t){
	// = a + bt + ct^2 + dt^3
	pos[0] = block.theta1.a+block.theta1.b*t + block.theta1.c*t*t + block.theta1.d*t*t*t;
	pos[1] = block.theta2.a+block.theta2.b*t + block.theta2.c*t*t + block.theta2.d*t*t*t;
	pos[2] = block.d3.a+block.d3.b*t + block.d3.c*t*t + block.d3.d*t*t*t;
	pos[3] = block.theta4.a+block.theta4.b*t + block.theta4.c*t*t + block.theta4.d*t*t*t;
}

void calcCubicVel(JOINT &vel, cubicJoints &block, double t, double timeslice){
	// = b + 2ct + 3dt^2
	vel[0] = (block.theta1.b + 2*block.theta1.c*t + 3*block.theta1.d*t*t)/timeslice;
	vel[1] = (block.theta2.b + 2*block.theta2.c*t + 3*block.theta2.d*t*t)/timeslice;
	vel[2] = (block.d3.b + 2*block.d3.c*t + 3*block.d3.d*t*t)/timeslice;
	vel[3] = (block.theta4.b + 2*block.theta4.c*t + 3*block.theta4.d*t*t)/timeslice;
}

void calcCubicAcc(JOINT &acc, cubicJoints &block, double t, double timeslice){
	// = 2c + 6dt
	acc[0] =  (2*block.theta1.c + 6*block.theta1.d*t)/(timeslice*timeslice);
	acc[1] =  (2*block.theta2.c + 6*block.theta2.d*t)/(timeslice*timeslice);
	acc[2] =  (2*block.d3.c + 6*block.d3.d*t)/(timeslice*timeslice);
	acc[3] =  (2*block.theta4.c+ 6*block.theta4.d*t)/(timeslice*timeslice); 
}

bool checkCubicValues(JOINT &position,JOINT &velocity,JOINT &acceleration){
	//check and make sure they do not violate joint limits
	//Todo: write the function
	bool clearPos, clearVel, clearAcc;
	clearPos = 0;
	clearVel = 0; 
	clearAcc = 0;

	if ((position[0] <= HIGHTHEATA1) && (position[0] >= LOWTHEATA1)
		&& (position[1] <= HIGHTHEATA2) && (position[1] >= LOWTHEATA12)
		&& (position[2] <= HIGHDISTANCE3) && (position[2] >= LOWDISTANCE3)
		&& (position[3] <= HIGHTHEATA4) && (position[3] >= LOWTHEATA4))
	{
		clearPos = 1;
	}
	else {
		clearPos = 0;
		cout << "Violating joint limits at " << position[0] << "," << position[1] << "," << position[2] << "," << position[3] << endl;
	}

	if ((velocity[0] <= HIGHTHEATAVEL1) && (velocity[0] >= LOWTHEATAVEL1)
		&& (velocity[1] <= HIGHTHEATAVEL2) && (velocity[1] >= LOWTHEATAVEL2)
		&& (velocity[2] <= HIGHDISTANCEVEL3) && (velocity[2] >= LOWDISTANCEVEL3)
		&& (velocity[3] <= HIGHTHEATAVEL4) && (velocity[3] >= LOWTHEATAVEL4))
	{
		clearVel = 1;
	}
	else {
		clearVel = 0;
		cout << "Violating velocity limits at " << velocity[0] << "," << velocity[1] << "," << velocity[2] << "," << velocity[3] << endl;
	
	}

	if ((acceleration[0] <= HIGHTHEATAACC1) && (acceleration[0] >= LOWTHEATAACC1)
		&& (acceleration[1] <= HIGHTHEATAACC2) && (acceleration[1] >= LOWTHEATAACC2)
		&& (acceleration[2] <= HIGHDISTANCEACC3) && (acceleration[2] >= LOWDISTANCEACC3)
		&& (acceleration[3] <= HIGHTHEATAACC4) && (acceleration[3] >= LOWTHEATAACC4))
	{
		clearAcc = 1;
	}
	else {
		clearAcc = 0;
		cout << "Violating acceleration limits at " << acceleration[0] << "," << acceleration[1] << "," << acceleration[2] << "," << acceleration[3] << endl;
	}
	return (clearPos & clearVel & clearAcc);
}

void printJointToFile(ofstream &outputFile, JOINT &toPrint, double time) {
		outputFile << toPrint[0] << "," << toPrint[1] << "," << toPrint[2] << "," << toPrint[3] << "," << time << endl;
}

void printSplineToFile (ofstream &outputFile, cubicJoints myJoint) {

	printCubicCoef(outputFile, myJoint.theta1);
	printCubicCoef(outputFile, myJoint.theta2);
	printCubicCoef(outputFile, myJoint.d3);
	printCubicCoef(outputFile, myJoint.theta4);
}

void printCubicCoef(ofstream &outputFile, cubicCoef myCoef) {

	outputFile << myCoef.a << "," << myCoef.b << "," << myCoef.c << "," << myCoef.d << endl;

}

cubicCoef * calculateCubicSpline(cubicJoints &jointArray, int size) {
	cubicCoef temp;
	cubicCoef * returnCoefs;
	returnCoefs = new cubicCoef[size];

	// FIX ME
	return returnCoefs; // care for mem leak
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