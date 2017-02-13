//library for robot matrix representation
//by Andrew Nichol 2017

#ifndef COMP1_H
#define COMP1_H

//may not need some of this
#include <stdio.h> //this is for printf 
#include <iostream> // this is for cout
#include <math.h> // this is for arc2tan 
using namespace std;

#define PI 3.14

struct frameParam_t{
double x;
double y;
double z;
double theta;
};

class HomoMat {
double homoMatrix [4][4];

public:
	//necessary functions
	HomoMat();
	void UTOI(double x, double y, double z, double angleDeg);
	frameParam_t ITOU(HomoMat & orig);
	//HomoMat TMULT(HomoMat A, HomoMat &B);
	HomoMat TINVERT(HomoMat NonInvert);
	//HomoMat operator * (HomoMat &rightmat);
	//extra functions
	void PRINTT(); //prints out entire array
	//ACCESST(int row ,int column) //allow access to element
	//MUTATET(int row, int column, int value) //allow modifiable access to element

};


#endif /* COMP1_H */


