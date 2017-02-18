//DEMO 1 script for ENSC 488 Project
//Andrew Nichol, 2017

#include <iostream> // for cout
#include <stdio.h> // for getchar
using namespace std;

//just for testing remove later
struct frameParam_t{
double x;
double y;
double z;
double theta;
};
//prototype
frameParam_t inputCoordinates();

int main(int argc, char* argv[])
{	
	frameParam_t currentframe;
	char selection;
	while(1){
	cout << "============================================="<<endl ;
	cout << "==               DEMO 1					=="<<endl ;
	cout << "===Please enter a character for a choice====="<<endl ;
	cout << "'z' for zero position"<<endl ;
	cout << "'e' to enter emulator position"<<endl ;
	cout << "'f' for forward kinematics on current position"<<endl;
	cout << "'i' to enter position preform inverse kinematics and move to position"<<endl;
	cout << "'x' to exit program"<<endl;
	cout << "============================================="<<endl ;
	cin>>selection;
	// test line: cout<< "echo:"<< selection<<"done" << endl;

	switch(selection){
		case 'z':
		//TODO: Fill
		break;

		case 'e':
		currentframe = inputCoordinates();
		//TODO:FINISH
		//NOTE: Their should be a sanity check on the variables entered
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


frameParam_t inputCoordinates(){
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