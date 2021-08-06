#include "stdafx.h"
#include "OtherFuns.h"
#include "math.h"

COtherFuns::COtherFuns()
{
}


COtherFuns::~COtherFuns()
{
}


int COtherFuns::Swap(int &a, int &b)
{
	int tmp;
	tmp = a;
	a = b;
	b = tmp;
	return 1;
}


int COtherFuns::ComputeTriangleArea(double x[3], double y[3], double &area)
{
	double a, b, c;
	a = sqrt((x[1] - x[0])*(x[1] - x[0]) + (y[1] - y[0])*(y[1] - y[0]));
	b = sqrt((x[2] - x[1])*(x[2] - x[1]) + (y[2] - y[1])*(y[2] - y[1]));
	c = sqrt((x[0] - x[2])*(x[0] - x[2]) + (y[0] - y[2])*(y[0] - y[2]));

	double p = (a + b + c) / 2.0;

	double area2 = p*(p - a)*(p - b)*(p - c);
	if (area2 < 0) area = 0;
	else
		area = sqrt(area2);

	return 1;
}

// int CBundleAdjust::eejcb(double a[], int N, double v[], double eps, int jt)
// {
// 	int   i, j, p, q, u, w, t, s, l;
// 	double   fm, cn, sn, omega, x, y, d;
// 
// 	l = 1;
// 	//   初始化   特征向量矩阵，   使其全为0 
// 	for (i = 0; i <= N - 1; i++)
// 	{
// 		v[i*N + i] = 1.0;
// 		for (j = 0; j <= N - 1; j++)
// 		{
// 			if (i != j)
// 			{
// 				v[i*N + j] = 0.0;
// 			}
// 		}
// 	}
// 	while (1 == 1)   //   循环 
// 	{
// 		fm = 0.0;
// 		for (i = 0; i <= N - 1; i++)   //   找出,矩阵a(特征值),中除对角线外   其他元素的最大绝对值 
// 		{                        //   这个最大值是位于a[p][q]   ,等于   fm 
// 			for (j = 0; j <= N - 1; j++)
// 			{
// 				d = fabs(a[i*N + j]);
// 				if ((i != j) && (d > fm))
// 				{
// 					fm = d;
// 					p = i;
// 					q = j;
// 				}
// 			}
// 		}
// 		if (fm < eps)      //   精度复合要求 
// 		{
// 			//将特征值和对应的特征向量进行由小到达的排序
// 			for (i = 0; i < N - 1; i++)
// 				for (j = i + 1; j < N; j++)
// 				{
// 					double temp;
// 					if (a[i*N + i] < a[j*N + j])
// 					{
// 						temp = a[i*N + i];
// 						a[i*N + i] = a[j*N + j];
// 						a[j*N + j] = temp;
// 
// 						int k;
// 						for (k = 0; k < N; k++)
// 						{
// 							temp = v[k*N + i];
// 							v[k*N + i] = v[k*N + j];
// 							v[k*N + j] = temp;
// 						}
// 					}
// 				}
// 			return(1);   //正常返回 
// 		}
// 		if (l > jt)      //   迭代次数太多 
// 		{
// 			return(-1);   //   失败返回 
// 		}
// 		l = l + 1;         //   迭代计数器 
// 		u = p*N + q;
// 		w = p*N + p;
// 		t = q*N + p;
// 		s = q*N + q;
// 		x = -a[u];
// 		y = (a[s] - a[w]) / 2.0;   //   x   y的求法不同 
// 		omega = x / sqrt(x*x + y*y);   //sin2θ 
// 		//   tan2θ=   x   /   y   =   -2.0   *   a   /   (a[s]-a[w]) 
// 		if (y < 0.0)
// 		{
// 			omega = -omega;
// 		}
// 		sn = 1.0 + sqrt(1.0 - omega*omega);
// 		sn = omega / sqrt(2.0*sn);//   sinθ 
// 		cn = sqrt(1.0 - sn*sn);            //   cosθ 
// 
// 		fm = a[w];   //   变换前的a[w]   a[p][p] 
// 		a[w] = fm*cn*cn + a[s] * sn*sn + a[u] * omega;
// 		a[s] = fm*sn*sn + a[s] * cn*cn - a[u] * omega;
// 		a[u] = 0.0;
// 		a[t] = 0.0;
// 
// 		//   一下是旋转矩阵,旋转了了p行,q行,p列,q列 
// 		//   但是四个特殊点没有旋转(这四个点在上述语句中发生了变化) 
// 		//   其他不在这些行和列的点也没变 
// 		//   旋转矩阵,旋转p行和q行 
// 		for (j = 0; j <= N - 1; j++)
// 		{
// 			if ((j != p) && (j != q))
// 			{
// 				u = p*N + j;
// 				w = q*N + j;
// 				fm = a[u];
// 				a[u] = fm*cn + a[w] * sn;
// 				a[w] = -fm*sn + a[w] * cn;
// 			}
// 		}
// 
// 		//   旋转矩阵,旋转p列和q列 
// 		for (i = 0; i <= N - 1; i++)
// 		{
// 			if ((i != p) && (i != q))
// 			{
// 				u = i*N + p;
// 				w = i*N + q;
// 				fm = a[u];
// 				a[u] = fm*cn + a[w] * sn;
// 				a[w] = -fm*sn + a[w] * cn;
// 			}
// 		}
// 
// 		//   记录旋转矩阵         特征向量 
// 		for (i = 0; i <= N - 1; i++)
// 		{
// 			u = i*N + p;
// 			w = i*N + q;
// 			fm = v[u];
// 			v[u] = fm*cn + v[w] * sn;
// 			v[w] = -fm*sn + v[w] * cn;
// 		}
// 	}
// 	return 1;
// }
// bool CBundleAdjust::DirectAbsoluteOrientationbyQuaternion(double *X, double *Y, double *Z, double *X_new, double *Y_new, double *Z_new,
// 	int Num, double &theta, double T[2])
// {
// 	if (X == NULL || Y == NULL || Z == NULL || X_new == NULL || Y_new == NULL || Z_new == NULL) return false;
// 
// 	//坐标重心化
// 	double Xc = 0.0, Yc = 0.0, Zc = 0.0;
// 	double Xc_new = 0.0, Yc_new = 0.0, Zc_new = 0.0;
// 
// 	int i = 0;
// 	for (i = 0; i < Num; i++)
// 	{
// 		Xc += X[i] / Num;
// 		Yc += Y[i] / Num;
// 		Zc += Z[i] / Num;
// 
// 		Xc_new += X_new[i] / Num;
// 		Yc_new += Y_new[i] / Num;
// 		Zc_new += Z_new[i] / Num;
// 	}
// 
// 	double *Xn, *Yn, *Zn;
// 	double *Xn_new, *Yn_new, *Zn_new;
// 	Xn = new double[Num];
// 	Yn = new double[Num];
// 	Zn = new double[Num];
// 	Xn_new = new double[Num];
// 	Yn_new = new double[Num];
// 	Zn_new = new double[Num];
// 
// 	for (i = 0; i < Num; i++)
// 	{
// 		Xn[i] = X[i] - Xc;
// 		Yn[i] = Y[i] - Yc;
// 		Zn[i] = Z[i] - Zc;
// 
// 		Xn_new[i] = X_new[i] - Xc_new;
// 		Yn_new[i] = Y_new[i] - Yc_new;
// 		Zn_new[i] = Z_new[i] - Zc_new;
// 	}
// 
// 	//计算theta
// 	theta = 0;
// 	for (i = 0; i < Num;i++)
// 	{
// 		double sin_value;
// 		sin_value = (Xn[i] * Yn_new[i] - Yn[i] * Xn_new[i]) / (Xn[i] * Xn[i] + Yn[i] * Yn[i]);
// 
// 		theta += asin(sin_value);
// 	}
// 	theta /= Num;
// 
// 	double R[4];
// 	R[0] = cos(theta); R[1] = -sin(theta);
// 	R[2] = sin(theta); R[3] = cos(theta);
// 
// 	double Tc[2], RTc[2];
// 	Tc[0] = Xc; Tc[1] = Yc;
// 	MultiMatrix(R, Tc, RTc, 2, 1, 2);
// 
// 	T[0] = Xc_new - RTc[0];
// 	T[1] = Yc_new - RTc[1];
// 	
// 
// // 	T[0] = Xcenter_new[0] - Xcenter2[0];
// // 	T[1] = Xcenter_new[1] - Xcenter2[1];
// // 	T[2] = Xcenter_new[2] - Xcenter2[2];
// 
// 	//释放内存
// 	delete[]Xn; Xn = NULL;
// 	delete[]Yn; Yn = NULL;
// 	delete[]Zn; Zn = NULL;
// 	delete[]Xn_new; Xn_new = NULL;
// 	delete[]Yn_new; Yn_new = NULL;
// 	delete[]Zn_new; Zn_new = NULL;
// 
// 	return 1;
// }
// 