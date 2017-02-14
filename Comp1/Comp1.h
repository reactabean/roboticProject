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
	//necessary functions--
	//constructor
	HomoMat();
	//creates a homogenoeus matrix using inputed paramater
	void UTOI(double x, double y, double z, double angleDeg);
	//returns x,y,z,theta from the homogeneous matrix
	frameParam_t ITOU();
	//HomoMat TMULT(HomoMat A, HomoMat &B);
	HomoMat TINVERT(HomoMat NonInvert);
	//facilitates multiplication between homogeneous matrices
	HomoMat operator * (HomoMat &rightmat);

	//extra functions--
	//prints out entire array
	void PRINTT(); 
	//allow access to element
	double ACCESST(int row ,int column); 
	//allow modifiable access to element
	void MUTATET(int row, int column, int value); 

};


#endif /* COMP1_H */


