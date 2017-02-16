//library for robot matrix representation
//by Andrew Nichol 2017

#ifndef COMP1_H
#define COMP1_H

//may not need some of this
#include <stdio.h> //this is for printf 
#include <iostream> // this is for cout
#include <math.h> // this is for arc2tan 
using namespace std;

// Define PI as accurately as possible
#define PI acos(-1.0)
// In our matrix operations we lose accuracy due to truncation
// We can assume two values are the same if their difference is less then the tolarance
#define TOLERANCE 0.00000001

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
	//constructor 2
	HomoMat(double x, double y, double z, double theta);
	//creates a homogenoeus matrix using inputed paramater
	void UTOI(double x, double y, double z, double angleDeg);
	//returns x,y,z,theta from the homogeneous matrix
	frameParam_t ITOU();
	//inverts homogeneous matrix 
	HomoMat TINVERT();
	//facilitates multiplication between homogeneous matrices
	HomoMat operator * (HomoMat &rightmat);

	//extra functions--
	//prints out entire array
	void PRINTT(); 
	//allow access to element
	double ACCESST(int row ,int column); 
	//allow modifiable access to element
	void MUTATET(int row, int column, double value); 

};


#endif /* COMP1_H */


