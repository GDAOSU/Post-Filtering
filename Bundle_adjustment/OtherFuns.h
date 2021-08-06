#pragma once
class COtherFuns
{
public:
	COtherFuns();
	~COtherFuns();

public:
	int ComputeTriangleArea(double x[3], double y[3], double &area);
	int Swap(int &a, int &b);

	/////////////////////////////////////Absolute Orientation////////////////////
	int eejcb(double a[], int N, double v[], double eps, int jt);
	bool DirectAbsoluteOrientationbyQuaternion(double *X, double *Y, double *Z, double *X_new, double *Y_new, double *Z_new,
		int Num, double &theta, double T[2]);

	int ExpandGroundOffSetByTwoOrderEquation(double *offset, int num);
	/////////////////////////////////////END/////////////////////////////////////
};

