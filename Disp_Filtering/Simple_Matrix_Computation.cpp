#include "stdafx.h"
#include "Simple_Matrix_Computation.h"

#define EPS 1.0e-12
#define ROTATE(a, i, j, k, l, n) g=a[i*n+j]; h=a[k*n+l]; a[i*n+j] = g-s*(h+g*tau); \
  a[k*n+l] = h+s*(g-h*tau);


CSimple_Matrix_Computation::CSimple_Matrix_Computation()
{
}


CSimple_Matrix_Computation::~CSimple_Matrix_Computation()
{
}

int CSimple_Matrix_Computation::Matrix_Add(double *A, double *B, double *C, int rows, int cols)
{
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols;j++)
		{
			C[i*cols + j] = A[i*cols + j] + B[i*cols + j];
		}
	return 1;
}

int CSimple_Matrix_Computation::TransposeMatrix(double *pData, int Rows, int Cols, double *pDataT)
{
	if (pData == NULL || pDataT == NULL)
		return false;

	if (Rows < 1 || Cols < 1)
	{
		return false;
	}

	memset(pDataT, 0, sizeof(double)*Rows*Cols);

	int i = 0, j = 0;
	for (i = 0; i < Cols; i++)
		for (j = 0; j < Rows; j++)
			pDataT[i*Rows + j] = pData[j*Cols + i];

	return 1;
}

bool CSimple_Matrix_Computation::InverseMatrix(double *pData, int n, double *pinvData)
{
	if ((pData == NULL) || (n < 1) || pinvData == NULL)
		return false;

	// 	if (n == 2)
	// 	{
	// 		pinvData[0] = pData[3] / (pData[0] * pData[3] - pData[1] * pData[2]);
	// 		pinvData[1] = -pData[1] / (pData[0] * pData[3] - pData[1] * pData[2]);
	// 		pinvData[2] = -pData[2] / (pData[0] * pData[3] - pData[1] * pData[2]);
	// 		pinvData[3] = pData[0] / (pData[0] * pData[3] - pData[1] * pData[2]);
	// 	}
	// 	else
	// 	{
	// 		memcpy(pinvData, pData, sizeof(double)*n*n);
	// 
	// 		int iMaxR = 0;
	// 		int iMaxC = 0;
	// 		int* pRow = new int[n];
	// 		int* pCol = new int[n];
	// 		double dTmp = 0;
	// 		double dMaxNum = 0;
	// 		int i = 0;
	// 		int j = 0;
	// 
	// 		for (int t = 0; t < n; t++)
	// 		{
	// 			dMaxNum = 0;
	// 			for (i = t; i < n; i++)
	// 			{
	// 				for (j = t; j < n; j++)
	// 				{
	// 					if (fabs(pinvData[i*n + j]) > dMaxNum)
	// 					{
	// 						iMaxR = i;
	// 						iMaxC = j;
	// 						dMaxNum = fabs(pinvData[i*n + j]);
	// 					}
	// 				}
	// 			}
	// 
	// 			pRow[t] = iMaxR;
	// 			pCol[t] = iMaxC;
	// 
	// 			if (dMaxNum < EPSION)
	// 			{
	// 				delete[] pRow;
	// 				delete[] pCol;
	// 				return false;
	// 			}
	// 
	// 			if (iMaxR != t)
	// 			{
	// 				for (i = 0; i < n; i++)
	// 				{
	// 					dTmp = pinvData[t*n + i];
	// 					pinvData[t*n + i] = pinvData[iMaxR*n + i];
	// 					pinvData[iMaxR*n + i] = dTmp;
	// 				}
	// 			}
	// 			if (iMaxC != t)
	// 			{
	// 				for (i = 0; i < n; i++)
	// 				{
	// 					dTmp = pinvData[i*n + t];
	// 					pinvData[i*n + t] = pinvData[i*n + iMaxC];
	// 					pinvData[i*n + iMaxC] = dTmp;
	// 				}
	// 			}
	// 
	// 			pinvData[t*n + t] = 1.0 / pinvData[t*n + t];
	// 			double dnn = pinvData[t*n + t];
	// 			for (i = 0; i < n; i++)
	// 			{
	// 				if (i != t)
	// 				{
	// 					pinvData[t*n + i] *= dnn;
	// 				}
	// 			}
	// 
	// 			for (i = 0; i < n; i++)
	// 			{
	// 				if (i != t)
	// 				{
	// 					double dit = pinvData[i*n + t];
	// 					if (fabs(dit) < EPSION)
	// 						continue;
	// 
	// 					for (j = 0; j < n; j++)
	// 					{
	// 						if (j != t)
	// 							pinvData[i*n + j] -= dit*pinvData[t*n + j];
	// 					}
	// 
	// 				}
	// 			}
	// 
	// 			for (i = 0; i < n; i++)
	// 			{
	// 				if (i != t)
	// 				{
	// 					pinvData[i*n + t] *= -dnn;
	// 				}
	// 			}
	// 		}
	// 
	// 		for (i = n - 1; i >= 0; i--)
	// 		{
	// 			if (pCol[i] != i)
	// 			{
	// 				iMaxR = pCol[i];
	// 				for (j = 0; j < n; j++)
	// 				{
	// 					dTmp = pinvData[iMaxR*n + j];
	// 					pinvData[iMaxR*n + j] = pinvData[i*n + j];
	// 					pinvData[i*n + j] = dTmp;
	// 				}
	// 			}
	// 
	// 			if (pRow[i] != i)
	// 			{
	// 				iMaxC = pRow[i];
	// 				for (j = 0; j < n; j++)
	// 				{
	// 					dTmp = pinvData[j*n + iMaxC];
	// 					pinvData[j*n + iMaxC] = pinvData[j*n + i];
	// 					pinvData[j*n + i] = dTmp;
	// 				}
	// 			}
	// 		}
	// 
	// 		if (pRow != NULL)
	// 		{
	// 			delete[] pRow;
	// 			pRow = NULL;
	// 		}
	// 		if (pCol != NULL)
	// 		{
	// 			delete[] pCol;
	// 			pCol = NULL;
	// 		}
	// 	}


	//////////////////////////////////////////////////////////////////////////test eigen
	Eigen::Map<Eigen::MatrixXd> map(pData, n, n);
	Eigen::MatrixXd inv = map.inverse();
	memcpy(pinvData, inv.data(), sizeof(double)*n*n);
	//////////////////////////////////////////////////////////////////////////

	return true;
}

void CSimple_Matrix_Computation::jacobi(double *a, int n, double *d, double *v, int *nrot)
{
	int j, iq, ip, ip_times_n, i;
	double tresh, theta, tau, t, sm, s, h, g, c, *b, *z, *vector();

	b = (double *)malloc(sizeof(double) *n);
	z = (double *)malloc(sizeof(double) *n);


	for (ip_times_n = 0, ip = 0; ip < n; ++ip, ip_times_n += n)
	{

		for (iq = 0; iq < n; ++iq)v[ip_times_n + iq] = 0.0;
		v[ip_times_n + ip] = 1.0;

		b[ip] = d[ip] = a[ip_times_n + ip];
		z[ip] = 0.0;
	}

	*nrot = 0;
	for (i = 0; i < 50; ++i)
	{
		sm = 0.0;

		for (ip_times_n = 0, ip = 0; ip < n - 1; ip++, ip_times_n += n)
			for (iq = ip + 1; iq < n; iq++)
				sm += fabs(a[ip_times_n + iq]);

		if (sm == 0.0)
		{
			free(b);
			free(z);
			return;
		}

		tresh = (i < 3) ? 0.2*sm / (n*n) : 0.0;

		for (ip_times_n = 0, ip = 0; ip < n - 1; ip++, ip_times_n += n)
		{
			for (iq = ip + 1; iq < n; ++iq)
			{
				g = 100.0*fabs(a[ip_times_n + iq]);

				if (i > 3 && g < EPS)
					a[ip_times_n + iq] = 0.0;

				else if (fabs(a[ip_times_n + iq]) > tresh)
				{
					h = d[iq] - d[ip];
					if (g < EPS)
						t = (fabs(a[ip_times_n + iq]) > EPS) ? (a[ip_times_n + iq]) / h : 0.0;
					else
					{
						theta = (fabs(h) < EPS) ? 0.0 : 0.5*h / (a[ip_times_n + iq]);
						t = 1.0 / (fabs(theta) + sqrt(1.0 + theta*theta));
						if (theta < 0.0)
							t = -t;
					}
					c = 1.0 / sqrt(1.0 + t*t);
					s = t*c;
					tau = s / (1.0 + c);

					h = t*a[ip_times_n + iq];
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					a[ip_times_n + iq] = 0.0;

					for (j = 0; j < ip; j++)
					{
						ROTATE(a, j, ip, j, iq, n);
					}
					for (j = ip + 1; j < iq; j++)
					{
						ROTATE(a, ip, j, j, iq, n);
					}
					for (j = iq + 1; j < n; j++)
					{
						ROTATE(a, ip, j, iq, j, n);
					}
					for (j = 0; j < n; j++)
					{
						ROTATE(v, j, ip, j, iq, n);
					}
					++(*nrot);
				}
			}
		}
		for (ip = 0; ip < n; ++ip)
		{
			b[ip] += z[ip];
			d[ip] = b[ip];
			z[ip] = 0.0;
		}
	}

	free(b);
	free(z);
	return;

}

bool CSimple_Matrix_Computation::svd(double *a, int m, int n, double *u, double *d, double *v)
{
	if (m <= 0 || n <= 0) return false;
	int i, j, k, nrot;
	double *aT = new double[m * n];
	double *a_aT = new double[m * m];
	double *aT_a = new double[n * n];

	if (!TransposeMatrix(a, m, n, aT)) return false;

	if (!MultiMatrix(a, aT, a_aT, m, m, n)) return false;
	if (!MultiMatrix(aT, a, aT_a, n, n, m)) return false;

	double *eigenvalues = new double[m];
	double *eigenvectors = new double[m * m];

	jacobi(a_aT, m, eigenvalues, eigenvectors, &nrot);

	jacobi(aT_a, n, d, v, &nrot);

	double tmp, *t = new double[n];
	for (i = 0; i < n - 1; i++)
	{
		tmp = d[i];
		for (k = 0; k < n; k++)
			t[k] = v[k * n + i];
		for (j = i + 1; j < n; j++)
		{
			if (d[j] > tmp)
			{
				d[i] = d[j];
				d[j] = tmp;
				tmp = d[i];
				for (k = 0; k < n; k++)
				{
					v[k * n + i] = v[k * n + j];
					v[k * n + j] = t[k];
				}
			}
		}
	}
	for (i = 0; i < n; i++)
		for (j = 0; j < m; j++)
		{
			if ((d[i] - eigenvalues[j]) < 0.00001 && (d[i] - eigenvalues[j]) > -0.00001)
			{
				for (k = 0; k < m; k++)
					u[k * n + i] = eigenvectors[k * m + j];
			}
		}

	delete eigenvalues;
	delete eigenvectors;
	delete aT;
	delete a_aT;
	delete aT_a;

	return true;
}

bool CSimple_Matrix_Computation::LeastSquares_adapt(double *A, double *L, double *X, int Rows, int Cols, double &alpha)
{
	if (A == NULL || L == NULL || X == NULL)
		return false;
	if (Rows < 1 || Cols < 1)
		return false;

	//////////////////////////////////////////////////////////////////////////try to normalize A and L
	// 	for (int i = 0; i < Rows*Cols;i++)
	// 	{
	// 		A[i] = A[i] * (1000/ sqrt(Rows));
	// 	}
	// 	for (int i = 0; i < Rows;i++)
	// 	{
	// 		L[i] = L[i] * (1000 / sqrt(Rows));
	// 	}
	//////////////////////////////////////////////////////////////////////////

	double *AT, *ATA, *invATA, *ATL;
	AT = new double[Cols*Rows];
	ATA = new double[Cols*Cols];
	invATA = new double[Cols*Cols];
	ATL = new double[Cols * 1];

	memset(AT, 0, sizeof(double)*Cols*Rows);
	memset(ATA, 0, sizeof(double)*Cols*Cols);
	memset(invATA, 0, sizeof(double)*Cols*Cols);
	memset(ATL, 0, sizeof(double)*Cols);

	if (!TransposeMatrix(A, Rows, Cols, AT))
		return false;
	if (!MultiMatrix(AT, A, ATA, Cols, Cols, Rows))
		return false;
	if (!MultiMatrix(AT, L, ATL, Cols, 1, Rows))
		return false;

	//////////////////////////////////////////////////////////////////////////
	//compute ridge parameter
	if (alpha == 0)
	{
		double XTX[1];
		double *XT = new double[Cols];
		memcpy(XT, X, sizeof(double)*Cols);
		MultiMatrix(XT, X, XTX, 1, 1, Cols);

		double *V = new double[Rows];
		double *AX = new double[Rows];
		MultiMatrix(A, X, AX, Rows, 1, Cols);

		for (int i = 0; i < Rows; i++)
		{
			V[i] = AX[i] - L[i];
		}

		double s = 0;
		for (int i = 0; i < Rows; i++)
		{
			s += V[i] * V[i];
		}
		s /= Rows;

		alpha = Cols*s / XTX[0];

		delete[]XT; XT = NULL;
		delete[]V; V = NULL;
		delete[]AX; AX = NULL;
	}
	//////////////////////////////////////////////////////////////////////////

	//Ridge estimate
	for (int i = 0; i < Cols; i++)
	{
		ATA[i*Cols + i] += alpha;
	}

	//try to use eigen
	MatrixXd Ge(Cols, Cols);
	VectorXd He(Cols);
	for (int i = 0; i < Cols; i++)
		for (int j = 0; j < Cols; j++)
		{
			Ge(i, j) = ATA[i*Cols + j];
		}
	for (int i = 0; i < Cols; i++)
	{
		He(i) = ATL[i];
	}
	VectorXd Xe = Ge.fullPivLu().solve(He);
	for (int i = 0; i < Cols; i++)
	{
		X[i] = Xe(i);
	}

	// 	if (!InverseMatrix(ATA, Cols, invATA))
	// 		return false;
	// 
	// 	if (!MultiMatrix(invATA, ATL, X, Cols, 1, Cols))
	// 		return false;

	if (AT != NULL)
	{
		delete[]AT;
		AT = NULL;
	}
	if (ATA != NULL)
	{
		delete[]ATA;
		ATA = NULL;
	}
	if (invATA != NULL)
	{
		delete[]invATA;
		invATA = NULL;
	}
	if (ATL != NULL)
	{
		delete[]ATL;
		ATL = NULL;
	}

	//////////////////////////////////////////////////////////////////////////recover A and L
	// 	for (int i = 0; i < Rows*Cols; i++)
	// 	{
	// 		A[i] = A[i] * (sqrt(Rows) / 1000);
	// 	}
	// 	for (int i = 0; i < Rows; i++)
	// 	{
	// 		L[i] = L[i] * (sqrt(Rows) / 1000);
	// 	}
	//////////////////////////////////////////////////////////////////////////

	return true;

}

bool CSimple_Matrix_Computation::LeastSquares(double *A, double *L, double *X, int Rows, int Cols, double &alpha) //A: Rows*Cols
{
	if (A == NULL || L == NULL || X == NULL)
		return false;
	if (Rows < 1 || Cols < 1)
		return false;

	double *AT, *ATA, *invATA, *ATL;
	AT = new double[Cols*Rows];
	ATA = new double[Cols*Cols];
	invATA = new double[Cols*Cols];
	ATL = new double[Cols * 1];

	memset(AT, 0, sizeof(double)*Cols*Rows);
	memset(ATA, 0, sizeof(double)*Cols*Cols);
	memset(invATA, 0, sizeof(double)*Cols*Cols);
	memset(ATL, 0, sizeof(double)*Cols);

	if (!TransposeMatrix(A, Rows, Cols, AT))
		return false;
	if (!MultiMatrix(AT, A, ATA, Cols, Cols, Rows))
		return false;

	if (!MultiMatrix(AT, L, ATL, Cols, 1, Rows))
		return false;

	//Ridge estimate
	//for (int i = 0; i < Cols; i++)
	//{
	//	ATA[i*Cols + i] += alpha;
	//}

	//try to use eigen
	MatrixXd Ge(Cols, Cols);
	VectorXd He(Cols);
	for (int i = 0; i < Cols; i++)
		for (int j = 0; j < Cols; j++)
		{
			Ge(i, j) = ATA[i*Cols + j];
		}
	for (int i = 0; i < Cols; i++)
	{
		He(i) = ATL[i];
	}
	VectorXd Xe = Ge.fullPivLu().solve(He);
	for (int i = 0; i < Cols; i++)
	{
		X[i] = Xe(i);
	}

	// 	if (!InverseMatrix(ATA, Cols, invATA))
	// 		return false;
	// 	if (!MultiMatrix(invATA, ATL, X, Cols, 1, Cols))
	// 		return false;

	if (AT != NULL)
	{
		delete[]AT;
		AT = NULL;
	}
	if (ATA != NULL)
	{
		delete[]ATA;
		ATA = NULL;
	}
	if (invATA != NULL)
	{
		delete[]invATA;
		invATA = NULL;
	}
	if (ATL != NULL)
	{
		delete[]ATL;
		ATL = NULL;
	}
	return true;
}

int CSimple_Matrix_Computation::MultiMatrix(double *A, double *B, double *AB, int Rows, int Cols, int pubs)
{
	if (A == NULL || B == NULL || AB == NULL)
		return false;

	if (Rows < 1 || Cols < 1 || pubs < 1)
		return false;
	double Temp = 0.0;
	int i, j, k;
	for (i = 0; i < Rows; i++)
		for (j = 0; j < Cols; j++)
		{
			for (k = 0; k < pubs; k++)
				Temp += A[i*pubs + k] * B[k*Cols + j];
			AB[i*Cols + j] = Temp;
			Temp = 0.0;
		}

	return 1;
}

int CSimple_Matrix_Computation::GetLinearFun(double *A, int rowA, int colA, double *L, double *A2, double *L2)
{
	double *At = new double[colA*rowA];

	TransposeMatrix(A, rowA, colA, At);
	MultiMatrix(At, A, A2, colA, colA, rowA);
	MultiMatrix(At, L, L2, colA, 1, rowA);

	//free memroy
	delete[]At; At = NULL;
	return 1;
}
