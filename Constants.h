//link Paramater limits
#define HIGHTHEATA1	    150
#define LOWTHEATA1     -150

#define HIGHTHEATA2	    100
#define LOWTHEATA12	   -100

#define HIGHDISTANCE3  -100 
#define LOWDISTANCE3   -200 

#define HIGHTHEATA4	    160
#define LOWTHEATA4 	   -160
// velocity limits
#define HIGHTHEATAVEL1	    150
#define LOWTHEATAVEL1      -150

#define HIGHTHEATAVEL2	    150
#define LOWTHEATAVEL2	   -150

#define HIGHDISTANCEVEL3    50 
#define LOWDISTANCEVEL3    -50 

#define HIGHTHEATAVEL4	    150
#define LOWTHEATAVEL4 	   -150

// acceleration limits
#define HIGHTHEATAACC1	    600
#define LOWTHEATAACC1      -600

#define HIGHTHEATAACC2	    600
#define LOWTHEATAACC2	   -600

#define HIGHDISTANCEACC3    200 
#define LOWDISTANCEACC3    -200 

#define HIGHTHEATAACC4   600
#define LOWTHEATAACC4   -600

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

//DEFINED TORQUE LIMITS
#define ROTARYTORQUE 16
#define LINEARTORQUE 45

//Define structure parameters
#define L1 405
#define L2 70
#define L3 195
#define L4 142
#define L5 140
#define L6 80
#define L7 130
#define L8 10
#define L9 30
#define Lmax 410

#define l1 0.405
#define l2 0.07
#define l3 0.195
#define l4 0.142
#define l5 0.14
#define l6 0.08
#define l7 0.13
#define l8 0.01
#define l9 0.03
#define lmax 0.41

#define frictCoeff 0.5
#define deltaT 5

#define M1 1.7
#define M2 1.0
#define M3 1.7
#define M4 1.0

#define gravity 9.8

//DEFINED CONTROLLER PARAMETERS
#define CONTROLLERTIME   10 // 10 ms given to controller to achieve desired pos, velocity, and acceleration. 
#define CONTROLLERSAMPTIME 2 //2 ms given between each controller 'tick'

//set using values given in assignment text
#define KP1 175 
#define KP2 110 
#define KP3 40 
#define KP4 20 

//should be 2*SQRT(KPi)
#define KV1 26.5 
#define KV2 10.5 
#define KV3 12.65 
#define KV4 9 