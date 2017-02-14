//library for robot matrix representation
//by Andrew Nichol 2017

#include "Comp1.h"

HomoMat::HomoMat(){
for (int row =0; row<4; row = row+1){
	for (int column =0; column<4; column = column+1){
		homoMatrix[row][column]=0;
	}
}
homoMatrix[3][3]=1;

}


frameParam_t HomoMat::ITOU(){
	frameParam_t temp;
	double c,s; 

	temp.x = homoMatrix [0][3]
	temp.y = homoMatrix [1][3]
	temp.z = homoMatrix [2][3]

	c = homoMatrix[0][0]
	s = homoMatrix[1][0]

	if c == 0 {
		if s == 0 {
			temp.theta =0;
		}
		else if s== 1 {
			temp.theta =  PI/2;
		}
		else if s == -1 {
			temp.theta = -1*PI/2;
		}
		return;
	}

	if s == 0 {
		if c == 0 {
			temp.theta = 0;
		}
		else if c== 1 {
			temp.theta = 0;
		}
		else if c == -1 {
			temp.theta = PI;
		}
		return;
	}

	temp.theta = atan2(s/c);

}

HomoMat HomoMat::TINVERT(){
	Homomat temp;
	double temp,x,y,z;

	//first invert matrices
	temp = homoMatrix [1,0];
	homoMatrix [1,0] = homoMatrix [0,1];
	homoMatrix [0,1] = temp;

	temp = homoMatrix [2,0];
	homoMatrix [2,0] = homoMatrix [0,2];
	homoMatrix [0,2] = temp;

	temp = homoMatrix [2,1];
	homoMatrix [2,1] = homoMatrix [1,2];
	homoMatrix [1,2] = temp;

	//now invert the translation
	x = homoMatrix [0][3];
	y = homoMatrix [1][3];
	z = homoMatrix [2][3];

    homoMatrix [0,3] = -1*(x*homoMatrix[0,0]+y*homoMatrix[0,1]+z*homoMatrix[0,2]);
    homoMatrix [1,3] = -1*(x*homoMatrix[1,0]+y*homoMatrix[1,1]+z*homoMatrix[1,2]);
    homoMatrix [2,3] = -1*(x*homoMatrix[2,0]+y*homoMatrix[2,1]+z*homoMatrix[2,2]);
}



HomoMat HomoMat::operator * (Homomat &rightmat){
	HomoMat temp;

	for (int row =0; row<4; row = row+1){
		for (int column =0; column<4; column = column+1){
			//homoMatrix[row][column]=0;
			temp.homoMatrix[row][column] = (homoMatrix[row][0]*rightmat[0][column]) + (homoMatrix[row][1]*rightmat[1][column]) + (homoMatrix[row][2]*rightmat[2][column]) + (homoMatrix[row][3]*rightmat[3][column]);
		}

	}
	return temp
}


void HomoMat::PRINTT(){//prints out entire array

	for (int row =0; row<4; row = row+1){
		for (int column =0; column<4; column = column+1){
			//homoMatrix[row][column]=0;
			cout<<homoMatrix[row][column]<<',';
		}
		cout<<'\n'<<endl;
	}

}


void HomoMat::UTOI(double x, double y, double z, double angleDeg){
	homoMatrix [0][3] = x;
	homoMatrix [1][3] = y;
	homoMatrix [2][3] = z;

	homoMatrix [0][0] = cos(PI*angleDeg/180);
	homoMatrix [0][1] = -1*sin(PI*angleDeg/180);
	homoMatrix [1][0] = sin(PI*angleDeg/180);
	homoMatrix [1][1] = cos(PI*angleDeg/180);
	homoMatrix [2][2] = 1;

}

double ACCESST(int row ,int column){
	return homoMatrix[row,column];
} 

void MUTATET(int row, int column, double value){
	homoMatrix[row,column] = value;
}


