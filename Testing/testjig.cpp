
#include "../Comp1/Comp1.h"
#include "../Comp2/Comp2.h"
#include "../Comp3/Comp3.h"

bool testMatrixEquality(double matrix1[4][4], HomoMat matrix2);
bool testInit();
bool testUTOIandITOU(double x, double y, double z, double theta);
bool testTINVERTandMULTIPLY(HomoMat toTest);
bool isCloseToEqual(double a, double b);
bool testKIN(JOINT &conf);
bool testWHERE(JOINT &conf);

int main()
{

HomoMat test;
std::cout.precision(10);
testInit();
testUTOIandITOU(1.5, 2, 3, 45);
testUTOIandITOU(-1, 0, 2, 0);
testUTOIandITOU(-1.4, 2.2, 3, 90);
testUTOIandITOU(0, 0, 0, 180);
testUTOIandITOU(1, 1, 1, 270);
testTINVERTandMULTIPLY(HomoMat(1, 2, 3, 44));
testTINVERTandMULTIPLY(HomoMat(1.213, 2.2, -3, 180));


JOINT q1 = { 0, 0, -100, 0 };
JOINT q2 = {90, -45, -200, 45 };
JOINT q3 = { RAD2DEG(.91), RAD2DEG(.793), -200, 45 };
testKIN(q1);
testWHERE(q2);
testWHERE(q3);

frameParam_t a;
a.x = 100;
a.y = 295;
a.z = 130;
a.theta = DEG2RAD(0);
jointReturn J = INVKIN(a);

a.x = -142;
a.y = 195;
a.z = 130;
a.theta = 2.35;
J = INVKIN(a);
getchar();

return 0;
}

bool testMatrixEquality(double matrix1[4][4], HomoMat matrix2) {
	for (int i = 0; i < 4; i++) {
		for (int k = 0; k < 4; k++) {
			if (!isCloseToEqual(matrix1[i][k], matrix2.ACCESST(i, k))) {
				cout << "i is " << i << "k is " << k << " first is " << matrix1[i][k] << " second is " << matrix2.ACCESST(i, k);
				return 0;
			}
		}
	}

	return 1;
}

bool testInit() {
	// Tests the default values of HomoMat initialization
	HomoMat test;

	// Currently we initialize as the identity matrix, change this definition if that changes
	double initialMatrix[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	
	if (testMatrixEquality(initialMatrix, test)) {
		cout << "Init test success \n";
		return 1;
	}
	else {
		cout << "Init test fail \n";
		return 0;
	}
	return 0;
}

bool testUTOIandITOU(double x, double y, double z, double theta) {
	// Tests the conversion of UTOI and back to ITOU with input parameters
	HomoMat test;
	test.UTOI(x, y, z, theta);
	frameParam_t params = test.ITOU();
	cout << "Testing ITOU with parameters x = " << x << " y = " << y << " z = " << z << " theta = " << theta << endl;

	if (x == params.x && y == params.y && z == params.z && theta == (params.theta*180)/PI) {
		cout << "testITOU successful \n";
		return 1;
	}
	else {
		cout << "testITO fail with output parameters x = " << params.x << " y = " << params.y << " z = " << params.z << " theta = " << params.theta << endl;
		return 0;
	}
	return 0;
}

bool testTINVERTandMULTIPLY(HomoMat toTest) {
	HomoMat Inverted = toTest.TINVERT();
	frameParam_t ITOU1 = Inverted.ITOU();
	frameParam_t ITOU2 = toTest.ITOU();

	cout << "Testing INVERSION with parameters x = " << ITOU2.x << " y = " << ITOU2.y << " z = " << ITOU2.z << " theta = " << ITOU2.theta << endl;

	HomoMat result = toTest*Inverted;
	double initialMatrix[4][4] = { { 1, 0, 0, 0 },{ 0, 1, 0, 0 },{ 0, 0, 1, 0 },{ 0, 0, 0, 1 } };

	// Test to see if multiplying our original matrix by its inversion yields the identity matrix
	if (testMatrixEquality(initialMatrix, result)) {
		cout << "Invert test success \n";
		return 1;
	}
	else {
		cout << "Invert test fail \n";
		return 0;
	}
	return 0;
}

bool isCloseToEqual(double a, double b) {
	if (abs(a - b) < TOLERANCE) {
		return 1;
	} 
	return 0;
}

bool testKIN(JOINT &conf) {
	frameParam_t Wrist_Frame = KIN(conf);
	cout << "The position of Wrist Frame for configuration " << conf[0] << ", " << conf[1] << ", " << conf[2] << ", " << conf[3] << ", " 
		<< "relative to Station is x = " << Wrist_Frame.x << " y = " << Wrist_Frame.y << " z= " << Wrist_Frame.z << " theta =" << Wrist_Frame.theta << endl;
	return 0;
}

bool testWHERE(JOINT &conf) {
	frameParam_t Tool_Frame = WHERE(conf);
	cout << "The position of Wrist Frame for configuration " << conf[0] << ", " << conf[1] << ", " << conf[2] << ", " << conf[3] << ", "
		<< "relative to Station is x = " << Tool_Frame.x << " y = " << Tool_Frame.y << " z= " << Tool_Frame.z << " theta =" << Tool_Frame.theta << endl;
	return 0;
}