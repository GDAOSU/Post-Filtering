#pragma once
#include "Eigen/Dense" 
using namespace Eigen;

class CSimple_Matrix_Computation
{
public:
	CSimple_Matrix_Computation();
	~CSimple_Matrix_Computation();

public:
	int TransposeMatrix(double *pData, int Rows, int Cols, double *pDataT);
	int MultiMatrix(double *A, double *B, double *AB, int Rows, int Cols, int pubs);
	bool LeastSquares(double *A, double *L, double *X, int Rows, int Cols, double &alpha);
	bool LeastSquares_adapt(double *A, double *L, double *X, int Rows, int Cols, double &alpha);
	bool InverseMatrix(double *pData, int n, double *pinvData);
	int GetLinearFun(double *A, int rowA, int colA, double *L, double *A2, double *L2);
	int Matrix_Add(double *A, double *B, double *C, int rows, int cols);
	void jacobi(double *a, int n, double *d, double *v, int *nrot);
	bool svd(double *a, int m, int n, double *u, double *d, double *v);
};

