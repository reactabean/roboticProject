// Library for Controller
// Created by Andrew Nichol, april 2017

#include "controller.h"
using namespace std;


bool moveCont(JOINT &dConf, JOINT &dVel, JOINT &dAcc, double timeToMove,JOINT &tVel){
	//tVel are passed in to act like initial values
	//local variables
	bool error;
	JOINT tConf;
	time_t before, after;
	int kV[4] = { KV1, KV2, KV3, KV4 };
	int kP[4] = { KP1, KP2, KP3, KP4 };
	JOINT m,v,g,f 
	JOINT torque = { 0, 0, 0, 0 };
	
	//initialize error state as false
	error = false;

	//get current position before loop 
	error = GetConfiguration(tConf);

	//get current program time count
	before = clock();
	after = clock();

	//execute for 20 secs
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
		//wait sample time - set to 2ms
		Sleep(CONTROLLERSAMPTIME);
		//update timer
		after = clock();
	}
	return error;
}