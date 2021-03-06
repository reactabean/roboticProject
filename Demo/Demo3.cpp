//DEMO 3 script for ENSC 488 Project
//Andrew Nichol,Adrian fettes, Monica Li,April 2017

#include <iostream> // for cout
#include <conio.h>

#ifndef ENSC488_H
#define ENSC488_H
#include "../ensc-488.h"
#endif /* ENSC488_H */

#include "../Comp1/Comp1.h"
#include "../Comp2/Comp2.h"
#include "../Comp3/Comp3.h"
#include "../Traj/Traj.h"
#include "../DynamicSim/DynamicSim.h"
using namespace std;

//DEFINE PARAMATERS
 //Home Configuration
#define HOMETHEATA1	    0
#define HOMETHEATA2	    0
#define HOMEDISTANCE3   -100
#define HOMETHEATA4	    0

///////////////////

//prototype
frameParam_t inputCartCoordinates();
void inputJointCoordinates(JOINT &temp);
bool checkJointCoordinates(JOINT &temp);
double jointDistance(JOINT &temp);
void printJoint(JOINT &temp);
double inputtime();
bool cartToJoint(JOINT &temp, JOINT &tempStart);
double jointDistanceToViaPoint(JOINT &temp, JOINT &tempStart); 
bool inputTorque(JOINT &temp);
double quickTraj(JOINT &joint1,JOINT &joint2,JOINT &joint3,JOINT &finaljoint);

int main(int argc, char* argv[])
{	
	frameParam_t toolFrame;
	JOINT jointParam, joint1,joint2,joint3,finaljoint;
	JOINT tau;
	JOINT position, velocity, acceleration;
	jointReturn jointSolution;
	bool works;
	char selection, type;
	double distance1, distance2;
	double diftime;
	double period;
	ofstream torquefile;
	
	while(1){
	cout << "============================================="<<endl ;
	cout << "==               DEMO 3					=="<<endl ;
	cout << "===Please enter a character for a choice====="<<endl ;
	cout << "'z' for zero position"<<endl ;
	cout << "'e' to enter joint position"<<endl ;
	cout << "'f' for forward kinematics on current position"<<endl;
	cout << "'c' to close the end effector" << endl;
	cout << "'o' to open the end effector" << endl;
	cout << "'i' to enter position perform inverse kinematics and move to position"<<endl;
	cout << "'j' to enter 4 positions and move through planned trajectory"<<endl;
	cout << "'t' to apply constant torque/forces to the joints" <<endl;
	cout << "'x' to exit program"<<endl;
	cout << "============================================="<<endl ;

	cin >> selection;
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

		case 'j'  :
			//input stage
			works = 0;

			//start frame
			GetConfiguration(jointParam);
				
			cout << "if you want a demo trajectory type '1' else type any character " << endl;
			cin >> type;

			// below is the demo trajectory for use in testing
			if (type == '1'){
				diftime  = quickTraj(joint1,joint2,joint3,finaljoint);
			} else{

				//first frame
				cout << "----entering first frame----" << endl;
				if (!cartToJoint(joint1, jointParam)){
					break;
				}

				//second frame
				cout << "----entering second frame----" << endl;
				if (!cartToJoint(joint2, joint1)) {
					break;
				}
			
				//third frame
				cout << "----entering third frame----" << endl;
				if (!cartToJoint(joint3, joint2)) {
					break;
				}
			
				//final frame
				cout << "----entering finaljoint frame----" << endl;
				if (!cartToJoint(finaljoint, joint3)) {
					break;
				}

				//time required frame
				diftime = inputtime();
			}

			//travel stage
			works = movetraj(jointParam,joint1,joint2,joint3,finaljoint,diftime,true);
			break;

		case 't'  :

			torquefile.open("pureTorqueOutput.csv");
			torquefile << "time(ms), theta1, theta2, d3, theta4, thetadot1, thetadot2, ddot3, thetadot4, thetadotdot1, thetadotdot2, ddotdot3, thetadotdot4 , tau1, tau2, tau3, tau4 \n";

			cout << "----entering four torques to joints----" << endl;
			works = inputTorque(tau);
			cout << "----entering the duration in second----" << endl;
			cin >> period;
			velocity[0] = 0;
			velocity[1] = 0;
			velocity[2] = 0;
			velocity[3] = 0;
			if (works == 0){
			cout << "Torque entered is outside the limits, CANCELLED ACTION" << endl;
			}else{
			update(tau, position, velocity, acceleration, 1000*period,torquefile,clock());
			}
			torquefile.close();
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

bool inputTorque(JOINT &temp) {
	bool works;
	works = 1 ;

	cout << "please enter torque for joint 1" << endl;
	cin >> temp[0];
	cout << "please enter torque for joint 2" << endl;
	cin >> temp[1];
	cout << "please enter torque for joint 3" << endl;
	cin >> temp[2];
	cout << "please enter torque for joint 4" << endl;
	cin >> temp[3];

	//check of values
	if (abs(temp[0])>ROTARYTORQUE | abs(temp[1])>ROTARYTORQUE | abs(temp[2])>LINEARTORQUE | abs(temp[3])>ROTARYTORQUE){
	works =0 ;
	}
	return works;
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

double inputtime(){
	double time;
	cout << "please enter time requirements (seconds)"<<endl;
	cin>>time;
	return time * 1000;
	//todo: implement some input checking 	
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

double jointDistanceToViaPoint(JOINT &temp, JOINT &tempStart) {
	double distance;
	distance = abs(temp[0] - tempStart[0]) + abs(temp[1] - tempStart[1]) + abs(temp[3] - tempStart[3]);
	return distance;
}

void printJoint(JOINT &temp) {
	cout << "theta 1 = " << temp[0] << "   theta 2 = " << temp[1] << "   distance 3 = " << temp[2] << "   theta 3 = " << temp[3] << endl;
}

bool cartToJoint(JOINT &temp, JOINT &tempStart) {
	//this funtion basically shortens down the inverse kin process for the purpose of trajectory planning
	bool error;
	jointReturn jointSolution;
	double distance1, distance2;

	error = 1;
	cout << "===Inputing trajectory frame===" << endl;
	jointSolution = INVKIN(inputCartCoordinates());


	if (jointSolution.configValid == 0) {
		cout << "Selected position is outside of workspace, CANCELLED ACTION" << endl;
		error =  0;
	}
	else {
		
		if (checkJointCoordinates(jointSolution.config1)) {
			distance1 = jointDistanceToViaPoint(jointSolution.config1, tempStart);
		}
		else {
			distance1 = 99999999;
		}
		if (checkJointCoordinates(jointSolution.config2)) {
			distance2 = jointDistanceToViaPoint(jointSolution.config2, tempStart);
		}
		else {
			distance2 = 99999999;
		}

		for (int i = 0; i < 4; i++) {
			temp[i] = (distance1 > distance2) ? jointSolution.config2[i] : jointSolution.config1[i];
		}

		if (!checkJointCoordinates(temp)) {
			cout << "Both solutions outside the joint limits, CANCELLED ACTION" << endl;
			error =  0;
		}
    
	}

	return error;
}

double quickTraj(JOINT &joint1,JOINT &joint2,JOINT &joint3,JOINT &finaljoint){

	//change as desired!
	joint1[0] =0;
	joint1[1] =0;
    joint1[2] =-175;
	joint1[3] =0;

	joint2[0] =0;
	joint2[1] =0;
	joint2[2] =-175;
	joint2[3] =0;

	joint3[0] =0;
	joint3[1] =0;
	joint3[2] =-175;
	joint3[3] =0;

	finaljoint[0] =0;
	finaljoint[1] =0;
	finaljoint[2] =-175;
	finaljoint[3] =0;

	//1 seconds
	return 1*1000;
}



//mark rubric----------------------------------------------

//Functioning code for dynamic simulation: NOT MET
//25
//Integrated functioning code for Inv Kin+Traj Plan + Controller + Dynamic Simulation: NOT MET
//50
//appropriate use of timers
//5
//Joint limits check for via pts (before issuing move command) : NOT MET
//5
//Choosing the nearest invkin solution to move to for each via pt : NOT MET 
//5
//Functioning limit checks for max velocity and max acceleration : NOT MET
//5
//Functioning limit checks for joint torques : NOT MET
//5
//Appropriate plots : NOT MET
//10
//Theory: Dynamic simulation and controller properly documented (as asked for in demo sheet): NOT MET
//5