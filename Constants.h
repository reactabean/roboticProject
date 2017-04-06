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

//Define structure parameters
#define L1 405
#define L2 70
#define L3 195
#define L4 142
#define L5 140
#define L6 80
#define L7 130
#define L8 10
#define Lmax 410

//DEFINED CONTROLLER PARAMETERS
#define CONTROLLERTIME   10 // 10 ms given to controller to achieve desired pos, velocity, and acceleration. 
#define CONTROLLERSAMPTIME 2 //2 ms given between each controller 'tick'

//should be large as possible to reduce settling time
#define KP1 1 //todo: set properly 
#define KP2 1 //todo: set properly
#define KP3 1 //todo: set properly 
#define KP4 1 //todo: set properly 

//should be 2*SQRT(KPi)
#define KV1 1 //todo: set properly 
#define KV2 1 //todo: set properly
#define KV3 1 //todo: set properly 
#define KV4 1 //todo: set properly 
