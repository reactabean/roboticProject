//DEMO 1 script for ENSC 488 Project
//Andrew Nichol, 2017

#include <iostream> // for cout
#include <conio.h>
#include "ensc-488.h"
#include "Comp1.h"
using namespace std;

//DEFINE PARAMATERS
 //Home Configuration
#define HOMETHEATA1	    0
#define HOMETHEATA2	    0
#define HOMEDISTANCE3   -100
#define HOMETHEATA4	    0
 //link Paramater limits
#define HIGHTHEATA1	    150
#define LOWTHEATA1     -150

#define HIGHTHEATA2	    100
#define LOWTHEATA12	   -100

#define HIGHDISTANCE3  -100 
#define LOWDISTANCE3   -200 

#define HIGHTHEATA4	    160
#define LOWTHEATA4 	   -160

//
///////////////////

//prototype
frameParam_t inputCartCoordinates();
void inputJointCoordinates(JOINT &temp);
bool checkJointCoordinates(JOINT &temp);

int main(int argc, char* argv[])
{	
	frameParam_t frameParam;
	JOINT jointParam;
	bool works;
	char selection;

	while(1){
	cout << "============================================="<<endl ;
	cout << "==               DEMO 1					=="<<endl ;
	cout << "===Please enter a character for a choice====="<<endl ;
	cout << "'z' for zero position"<<endl ;
	cout << "'e' to enter joint position"<<endl ;
	cout << "'f' for forward kinematics on current position"<<endl;
	cout << "'i' to enter position preform inverse kinematics and move to position"<<endl;
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
		//TODO: Fill
		break;

		case 'i'  :
		//TODO: Fill
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
	//	cout<<"entered digit:"<<temp[0]<<endl; //test funtion remove on final build
	cout<< "please enter theta2(degrees)"<<endl;
	cin>>temp[1];
	//	cout<<"entered digit:"<<temp[1]<<endl; //test funtion remove on final build
	cout<< "please enter distance3(millimeters)"<<endl;
	cin>>temp[2];
	//	cout<<"entered digit:"<<temp[2]<<endl; //test funtion remove on final build
	cout << "please enter theta4(degrees)"<<endl;
	cin>>temp[3];
	//	cout<<"entered digit:"<<temp[3]<<endl; //test funtion remove on final build
}


bool checkJointCoordinates(JOINT &temp){
	//returns true should the coordinates be within bounds set by paramaters
	bool clear;
	clear = 0;
	if ((temp[0] <HIGHTHEATA1 ) && ( temp[0] >LOWTHEATA1) 
		&& (temp[1] <HIGHTHEATA2 ) && ( temp[1] > LOWTHEATA12) 
		&& (temp[2] < HIGHDISTANCE3) && ( temp[2] >LOWDISTANCE3) 
		&& (temp[3] < HIGHTHEATA4) && ( temp[3] >LOWTHEATA4 ))
	{
		clear =1 ;
	}
	return clear;
}


//mark rubric----------------------------------------------
//Functioning code for basic forward kinematics  : NOT MET
//30
//Functioning code for  basic  inverse kinematics: NOT MET
//35
//Functioning joint limits check in Forw/Inv Kin (before issuing move command) : NOT MET
//5
//Functioning multiple solutions and choosing the nearest one to move to : NOT MET
//15
//Functioning outside workspace (no solution exists) : NOT MET
//5
//Theory: forward kinematics equations : NOT MET
//5
//Theory: inverse kinematics solution: NOT MET