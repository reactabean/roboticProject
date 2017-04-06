// Library for Controller
// Created by Andrew Nichol, april 2017

#include "controller.h"
using namespace std;

bool moveCont(JOINT &dConf, JOINT &dVel, JOINT &dAcc){
	//local variables
	bool error;
	JOINT tConf,tVel; // true position values
	time_t before, after;
	int kV[4] = { KV1, KV2, KV3, KV4 };
	int kP[4] = { KP1, KP2, KP3, KP4 };
	JOINT m,v,g,f 
	JOINT torque = { 0, 0, 0, 0 };
	
	//todo:need to assign proper values to tConf and tVel prior to running loop
	//tConf = { 0, 0, 0, 0 };
	//tVel  = { 0, 0, 0, 0 };
	
	//initialize error state as false
	error = false;

	//get current program time count
	before = clock();
	after = clock();


	while (CONTROLLERTIME > difftime(after, before)){
		
		M(tConf,m);
		V(tConf,tVel,V);
		G(tconf,g);
		F(tConf,tVel);

		for (int i = 0; i < 4; i++) {
			//non partitioned stage
			torque[i] = (kP[i]*(dConf[i] - tConf[i]) + kV[i]*(dVel[i] - tVel[i]) + dAcc[i]);
			//partitioned stage
			torque[i] = torque[i]*m[i] + v[i] +g[i] + f[i];
		}
		
		//todo: should have error check on torque values

		//update new points with emulator
		emulator(tConf,tVel,torque);
		//wait sample time
		Sleep(CONTROLLERSAMPTIME);
		//update timer
		after = clock();
	}
	return error;
}