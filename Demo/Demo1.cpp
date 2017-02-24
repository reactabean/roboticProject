//DEMO 1 script for ENSC 488 Project
//Andrew Nichol,Feb 2017

#include <iostream> // for cout
#include <conio.h>

#ifndef ENSC488_H
#define ENSC488_H
#include "../ensc-488.h"
#endif /* ENSC488_H */

#include "../Comp1/Comp1.h"
#include "../Comp2/Comp2.h"
#include "../Comp3/Comp3.h"
using namespace std;

//DEFINE PARAMATERS
 //Home Configuration
#define HOMETHEATA1	    0
#define HOMETHEATA2	    0
#define HOMEDISTANCE3   -100
#define HOMETHEATA4	    0


//
///////////////////

//prototype
frameParam_t inputCartCoordinates();
void inputJointCoordinates(JOINT &temp);
bool checkJointCoordinates(JOINT &temp);
double jointDistance(JOINT &temp);
void printJoint(JOINT &temp);

int main(int argc, char* argv[])
{	
	frameParam_t toolFrame;
	JOINT jointParam;
	jointReturn jointSolution;
	bool works;
	char selection;
	double distance1, distance2;

	while(1){
	cout << "============================================="<<endl ;
	cout << "==               DEMO 1					=="<<endl ;
	cout << "===Please enter a character for a choice====="<<endl ;
	cout << "'z' for zero position"<<endl ;
	cout << "'e' to enter joint position"<<endl ;
	cout << "'f' for forward kinematics on current position"<<endl;
	cout << "'i' to enter position perform inverse kinematics and move to position"<<endl;
	cout << "'c' to close the end effector" << endl;
	cout << "'o' to open the end effector" << endl;
	cout << "'x' to exit program"<<endl;
	cout << "============================================="<<endl ;
	cin>>selection;
	// test line: cout<< "echo:"<< selection<<"done" << endl;

	switch(selection){
		case 'z':
		jointParam[0] = HOMETHEATA1;
		jointParam[1] = HOMETHEATA2;
		jointParam[2] = HOMEDISTANCE3; 
		jointParam[3] = HOMETHEATA4;
		works = MoveToConfiguration(jointParam);
		if(works == 0){
			cout<<"set home position is impossible given joint contraints, CANCELLED ACTION"<<endl;
		}
		break;

		case 'e':
		inputJointCoordinates(jointParam);
		works = checkJointCoordinates(jointParam);

		if(works == 0){
			cout<<"Selected position is impossible given joint contraints, CANCELLED ACTION"<<endl;
		}else{
			works = MoveToConfiguration(jointParam);
			if(works == 0){
				cout<<"Command to move failed, CANCELLED ACTION"<<endl;
			}
		}
		break;

		case 'f'  :
		cout<<"------Kinematic analysis being done on current position------"<<endl;
		GetConfiguration(jointParam);
		toolFrame = WHERE(jointParam);
		cout << "The Joint positions: [" << jointParam[0] << ", " << jointParam[1] << ", " << jointParam[2] << ", " << jointParam[3] << "] " << endl;
		
		//eliminate small values from display
		if ( (toolFrame.x  > -0.01) && (toolFrame.x < 0.01)) {toolFrame.x =0;}
		if ( (toolFrame.y  > -0.01) && (toolFrame.y < 0.01)) {toolFrame.y =0;}
		if ( (toolFrame.z  > -0.01) && (toolFrame.z < 0.01)) {toolFrame.z =0;}
		if ( (toolFrame.theta  > -0.001) && (toolFrame.theta < 0.001)) {toolFrame.theta = 0;}

		cout << "Relative to Station is: x = " << toolFrame.x << " y = " << toolFrame.y << " z= " << toolFrame.z << " theta =" << RAD2DEG(toolFrame.theta) << endl;
		break;

		case 'i'  :
		jointSolution = INVKIN(inputCartCoordinates());

		if (jointSolution.configValid == 0) {
			cout << "Selected position is outside of workspace, CANCELLED ACTION" << endl;
		}
		else {
			cout << "Solution 1 is: " << endl;
			printJoint(jointSolution.config1);
			cout << "Solution 2 is: " << endl;
			printJoint(jointSolution.config2);

			if (checkJointCoordinates(jointSolution.config1)) {
				distance1 = jointDistance(jointSolution.config1);
			}
			else {
				distance1 = 99999999;
			}
			if (checkJointCoordinates(jointSolution.config2)) {
				distance2 = jointDistance(jointSolution.config2);
			}
			else {
				distance2 = 99999999;
			}

			cout << "Moving to Solution " << (distance1 > distance2) + 1 << endl;
			for (int i = 0; i < 4; i++) {
				jointParam[i] = (distance1 > distance2) ? jointSolution.config2[i] : jointSolution.config1[i];
			}
			if (!checkJointCoordinates(jointParam)) {
				cout << "Both solutions outside the joint limits, CANCELLED ACTION" << endl;
			} else if (!MoveToConfiguration(jointParam)) {
				cout << "Command to move failed, CANCELLED ACTION" << endl;
			}

		}

		break;

		case 'c':
			works = Grasp(true);

			if (works == 0) {
				cout << "closing grasp failed" << endl;
			}
			break;

		case 'o':
			works = Grasp(false);

			if (works == 0) {
				cout << "opening grasp failed" << endl;
			}
			break;

		case 'x'  :
		return 0;
		break;

   default : 
      cout << "__________you did not enter a valid character_________"<<endl;;
}
	}
	return 0;
}


frameParam_t inputCartCoordinates(){
	frameParam_t temp;
	cout << "please enter x coordinate relative to base frame"<<endl;
	cin>>temp.x;
		cout<<"entered digit:"<<temp.x<<endl; //test funtion remove on final build
	cout<< "please enter y coordinate relative to base frame"<<endl;
	cin>>temp.y;
		cout<<"entered digit:"<<temp.y<<endl; //test funtion remove on final build
	cout<< "please enter z coordinate relative to base frame"<<endl;
	cin>>temp.z;
		cout<<"entered digit:"<<temp.z<<endl; //test funtion remove on final build
	cout << "please enter theta(degrees) coordinate relative to base frame"<<endl;
	cin>>temp.theta;
		cout<<"entered digit:"<<temp.theta<<endl; //test funtion remove on final build
	return temp;
}

void inputJointCoordinates(JOINT &temp){
	cout << "please enter theta1(degrees)"<<endl;
	cin>>temp[0];
		cout<<"entered digit:"<<temp[0]<<endl; //test funtion remove on final build
	cout<< "please enter theta2(degrees)"<<endl;
	cin>>temp[1];
	  	cout<<"entered digit:"<<temp[1]<<endl; //test funtion remove on final build
	cout<< "please enter distance3(millimeters)"<<endl;
	cin>>temp[2];
	  	cout<<"entered digit:"<<temp[2]<<endl; //test funtion remove on final build
	cout << "please enter theta4(degrees)"<<endl;
	cin>>temp[3];
	  	cout<<"entered digit:"<<temp[3]<<endl; //test funtion remove on final build
}


bool checkJointCoordinates(JOINT &temp){
	//returns true should the coordinates be within bounds set by paramaters
	bool clear;
	clear = 0;
	if ((temp[0] <=HIGHTHEATA1 ) && ( temp[0] >=LOWTHEATA1) 
		&& (temp[1] <=HIGHTHEATA2 ) && ( temp[1] >= LOWTHEATA12) 
		&& (temp[2] <= HIGHDISTANCE3) && ( temp[2] >= LOWDISTANCE3) 
		&& (temp[3] <= HIGHTHEATA4) && ( temp[3] >=LOWTHEATA4 ))
	{
		clear =1 ;
	}
	return clear;
}

double jointDistance(JOINT &temp) {
	JOINT current;
	double distance;
	GetConfiguration(current);
	distance = abs(temp[0] - current[0]) + abs(temp[1] - current[1]) + abs(temp[3] - current[3]);
	return distance;
}

void printJoint(JOINT &temp) {
	cout << "theta 1 = " << temp[0] << "   theta 2 = " << temp[1] << "   distance 3 = " << temp[2] << "   theta 3 = " << temp[3] << endl;
}


//mark rubric----------------------------------------------
//Functioning code for basic forward kinematics  : MET
//30
//Functioning code for  basic  inverse kinematics: MET
//35
//Functioning joint limits check in Forw/Inv Kin (before issuing move command) : MET
//5
//Functioning multiple solutions and choosing the nearest one to move to : MET
//15
//Functioning outside workspace (no solution exists) : MET
//5
//Theory: forward kinematics equations : NOT MET
//5
//Theory: inverse kinematics solution: NOT MET