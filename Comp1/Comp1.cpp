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

/*
frameParam_t HomoMat::ITOU(HomoMat & orig){

}
HomoMat HomoMat::TINVERT(HomoMat NonInvert){

}
*/


HomoMat HomoMat::operator * (Homomat &rightmat){
	HomoMat temp;

	for (int row =0; row<4; row = row+1){
		for (int column =0; column<4; column = column+1){
			//homoMatrix[row][column]=0;
			temp.homoMatrix[row][column] = (homoMatrix[row][0]*rightmat[0][column]) + (homoMatrix[row][1]*rightmat[1][column]) + (homoMatrix[row][2]*rightmat[2][column]) + (homoMatrix[row][3]*rightmat[3][column]);
		}

	}

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
