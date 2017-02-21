//library for robot matrix representation
//by Andrew Nichol ,Feb 2017

#include "Comp1.h"

HomoMat::HomoMat(){
for (int row =0; row<4; row = row+1){
	for (int column =0; column<4; column = column+1){
		homoMatrix[row][column]=0;
		if (row == column) {
			homoMatrix[row][column] = 1;
		}
	}
}
}

HomoMat::HomoMat(double x, double y, double z, double theta) {
	for (int row = 0; row<4; row = row + 1) {
		for (int column = 0; column<4; column = column + 1) {
			homoMatrix[row][column] = 0;
		}
	}
	homoMatrix[3][3] = 1;

	UTOI(x, y, z, theta);
}

frameParam_t HomoMat::ITOU(){
	frameParam_t temp;
	double c,s; 

	temp.x = homoMatrix [0][3];
	temp.y = homoMatrix [1][3];
	temp.z = homoMatrix [2][3];

	c = homoMatrix[0][0];
	s = homoMatrix[1][0];

	if (c == 0) {
		if (s == 0) {
			temp.theta =0;
		}
		return temp;
	}

	temp.theta = atan2(s,c);

	if (temp.theta < 0) {
		temp.theta = temp.theta + 2 * PI;
	}

	return temp;
}

HomoMat HomoMat::TINVERT(){
	//[ Rt| -Rt*p]

	HomoMat temp;
	double x,y,z;
	int i, k;


	//first invert matrices
	for (i = 0; i < 3; i++) {
		for (k = 0; k < 3; k++) {
			temp.homoMatrix[i][k] = homoMatrix[k][i];
		}
	}

	//now invert the translation
	x = homoMatrix [0][3];
	y = homoMatrix [1][3];
	z = homoMatrix [2][3];

    temp.homoMatrix [0][3] = -1*(x*temp.homoMatrix [0][0]+y*temp.homoMatrix [0][1]+z*temp.homoMatrix [0][2]);
    temp.homoMatrix [1][3] = -1*(x*temp.homoMatrix [1][0]+y*temp.homoMatrix [1][1]+z*temp.homoMatrix [1][2]);
    temp.homoMatrix [2][3] = -1*(x*temp.homoMatrix [2][0]+y*temp.homoMatrix [2][1]+z*temp.homoMatrix [2][2]);

    return temp;
}

HomoMat HomoMat::operator * (HomoMat &rightmat){
	HomoMat temp;

	for (int row =0; row<4; row = row+1){
		for (int column =0; column<4; column = column+1){
			//homoMatrix[row][column]=0;
			temp.homoMatrix[row][column] = (homoMatrix[row][0]*rightmat.homoMatrix[0][column]) + (homoMatrix[row][1]*rightmat.homoMatrix[1][column]) + (homoMatrix[row][2]*rightmat.homoMatrix[2][column]) + (homoMatrix[row][3]*rightmat.homoMatrix[3][column]);
		}
	}
	return temp;
}


void HomoMat::PRINTT(){//prints out entire array

	for (int row =0; row<4; row = row+1){
		cout<<'|';
		for (int column =0; column<3; column = column+1){
			//homoMatrix[row][column]=0;
			cout<<homoMatrix[row][column]<<',';
		}
		cout<<'|'<<homoMatrix[row][3]<<'|'<<'\n'<<endl;
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

double HomoMat::ACCESST(int row ,int column){
	return homoMatrix[row][column];
} 

void HomoMat::MUTATET(int row, int column, double value){
	homoMatrix[row][column] = value;
}



