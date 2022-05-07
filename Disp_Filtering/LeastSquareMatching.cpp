#include "stdafx.h"
#include "LeastSquareMatching.h"


#define InValidValue -99

CLeastSquareMatching::CLeastSquareMatching()
{
}


CLeastSquareMatching::~CLeastSquareMatching()
{
}

int CLeastSquareMatching::ComputeSysBiasBasedonLeastSquareMatching(
	float *img1, int w1, int h1, 
	float *img2, int w2, int h2,
	int *&feax1, int *&feay1, int &ptNum, float *&dx, float *&dy, int GridSize, int MchSize)
{
	//1. make the intensigy scale of the two images consistent
	IntensityScaleMatchingBetweenTwoImgs(img1, w1, h1, img2, w2, h2);

	//2. extract feature points from the first image
	float temp_mask[9] = {
		0.25, 0.5, 0.25,
		0.5, -3, 0.5, 
		0.25, 0.5, 0.25
	};
	int size = 3;

	float *img_filter;
	LaplaceFiltering(img1, w1, h1, temp_mask, size, img_filter);

	vector<int> fx, fy;
	ExtractFeaturePoints(img_filter, w1, h1, GridSize, fx, fy);

	//3. compute sys bias between corresponding points
	float *Dx, *Dy;
	LeastSquareMatching(img1, w1, h1, img2, w2, h2, MchSize,
		fx, fy,
		Dx, Dy);

	//4. output result
	ptNum = 0;
	int totalNum = (int)fx.size();
	for (int i = 0; i < totalNum;i++)
	{
		if (Dx[i]!=InValidValue)
		{
			ptNum++;
		}
	}

	feax1 = new int[ptNum];
	feay1 = new int[ptNum];

	dx = new float[ptNum];
	dy = new float[ptNum];

	int I = 0;
	for (int i = 0; i < totalNum; i++)
	{
		if (Dx[i] != InValidValue)
		{
			feax1[I] = fx[i];
			feay1[I] = fy[i];

			dx[I] = Dx[i];
			dy[i] = Dy[i];
			I++;
		}
	}

	//free memory
	delete[]img_filter; img_filter = NULL;
	fx.clear();
	fy.clear();
	vector<int>().swap(fx);
	vector<int>().swap(fy);
	delete[]Dx; Dx = NULL;
	delete[]Dy; Dy = NULL;
	return 1;
}

int CLeastSquareMatching::LaplaceFiltering(float *img, int w, int h, float *mask, int size, float *&img_filter)
{
	img_filter = new float[w*h];
	memcpy(img_filter, img, sizeof(float)*w*h);

	float *img_mask = new float[size*size];

	int x, y;
	int halfsize = size / 2;
	for (y = 0; y < h; y++)
		for (x = 0; x < w;x++)
		{
			int sx, sy, ex, ey;
			sx = x - halfsize; ex = x + halfsize;
			sy = y - halfsize; ey = y + halfsize;

			int tx, ty;
			for (ty = sy; ty <= ey; ty++)
				for (tx = sx; tx <= ex; tx++)
				{
					if (ty >= 0 && ty < h && tx >= 0 && tx < w)
						img_mask[(ty - sy)*size + (tx - sx)] = img[ty*w + tx];
					else
						img_mask[(ty - sy)*size + (tx - sx)] = 0;
				}

			double filter_result = 0;
			for (ty = 0; ty < size; ty++)
				for (tx = 0; tx < size;tx++)
				{
					filter_result += img_mask[ty*size + tx] * mask[ty*size + tx];
				}

			img_filter[y*w + x] = (float)filter_result;
		}

	//free memory
	delete[]img_mask; img_mask = NULL;

	return 1;
}

int CLeastSquareMatching::IntensityScaleMatchingBetweenTwoImgs(float *img1, int w1, int h1, float *img2, int w2, int h2)
{
	//intensity center of the two images
	double Ic1 = 0, Ic2 = 0;

	int x, y;
	for (y = 0; y < h1; y++)
		for (x = 0; x < w1;x++)
		{
			Ic1 += img1[y*w1 + x];
		}
	Ic1 /= (w1*h1);

	for (y = 0; y < h2; y++)
		for (x = 0; x < w2; x++)
		{
			Ic2 += img2[y*w2 + x];
		}
	Ic2 /= (w2*h2);

	//intensity var of the two images
	double var1 = 0, var2 = 0;
	for (y = 0; y < h1; y++)
		for (x = 0; x < w1; x++)
		{
			var1 += (img1[y*w1 + x] - Ic1)*(img1[y*w1 + x] - Ic1);
		}
	var1 /= (w1*h1);

	for (y = 0; y < h2; y++)
		for (x = 0; x < w2; x++)
		{
			var2 += (img2[y*w2 + x] - Ic2)*(img2[y*w2 + x] - Ic2);
		}
	var2 /= (w2*h2);

	//scale parameter I1 = a*I2+b
	float a, b;
	a = (float)sqrt(var1 / var2);
	b = (float)(Ic1 - a*Ic2);

	//adjust the intensity of I2
	for (y = 0; y < h2; y++)
		for (x = 0; x < w2; x++)
		{
			img2[y*w2 + x] = a*img2[y*w2 + x] + b;
		}

	return 1;
}

int CLeastSquareMatching::ExtractFeaturePoints(float *img, int w, int h, int GridSize, vector<int> &feax, vector<int> &feay)
{
	if (GridSize>w || GridSize>h)
	{
		return 0;
	}

	int x, y;
	for (y = 1; y < h - GridSize; y += GridSize)
		for (x = 1; x < w - GridSize;x+=GridSize)
		{
			int sx, sy, ex, ey;
			sx = x; 
			sy = y;
			ex = x + GridSize - 1;
			ey = y + GridSize - 1;


			int tx, ty;
			float max_value = 0;
			int max_x = sx, max_y = sy;
			for (ty = sy; ty <= ey; ty++)
				for (tx = sx; tx <= ex; tx++)
				{
					if (max_value<fabs(img[ty*w+tx]))
					{
						max_value = fabs(img[ty*w + tx]);
						max_x = tx;
						max_y = ty;
					}
				}

			feax.push_back(max_x);
			feay.push_back(max_y);
		}

	return 1;
}

int CLeastSquareMatching::Bilinear_interpolation(float *img, int w, int h, float ix, float iy, float &intensity)
{
	int px, py;
	px = (int)ix; py = (int)iy;
	float u, v;
	u = ix - px; v = iy - py;

	float I1, I2, I3, I4;
	if (px >= 0 && px < w && py >= 0 && py < h)
		I1 = img[py*w + px];
	else
		I1 = 0;

	if (px + 1 >= 0 && px + 1 < w && py >= 0 && py < h)
		I2 = img[py*w + (px + 1)];
	else
		I2 = 0;

	if (px + 1 >= 0 && px + 1 < w && py + 1 >= 0 && py + 1 < h)
		I3 = img[(py + 1)*w + (px + 1)];
	else
		I3 = 0;

	if (px >= 0 && px < w && py + 1 >= 0 && py + 1 < h)
		I4 = img[(py + 1)*w + px];
	else
		I4 = 0;

	intensity = (1 - u)*(1 - v)*I1 + (1 - u)*v*I4 + u*(1 - v)*I2 + u*v*I3;

	return 1;
}

int CLeastSquareMatching::ComputeImgGradient(float *img, int w, int h, float x, float y, char direction, float &gradient)
{
	gradient = 0;
	if (direction == 'x')
	{
		float x2, x1;
		x2 = x + 1;
		x1 = x - 1;
		float Ix2, Ix1;
		Bilinear_interpolation(img, w, h, x2, y, Ix2);
		Bilinear_interpolation(img, w, h, x1, y, Ix1);
		gradient = (Ix2 - Ix1) / 2;
	}
	else
	{
		float y2, y1;
		y2 = y + 1;
		y1 = y - 1;
		float Iy2, Iy1;
		Bilinear_interpolation(img, w, h, x, y2, Iy2);
		Bilinear_interpolation(img, w, h, x, y1, Iy1);
		gradient = (Iy2 - Iy1) / 2;
	}
	return 1; 
}

int CLeastSquareMatching::LeastSquareMatching_singlePoint(float *img1, int w1, int h1, float *img2, int w2, int h2, int WinSize,
	int feax, int feay, float &dx, float &dy)
{
	int ptNum = WinSize*WinSize;
	float *A, *L, X[2];
	A = new float[ptNum * 2];
	L = new float[ptNum];

	float *AT, *ATA, *invATA, *ATL;
	AT = new float[2 * ptNum];
	ATA = new float[2 * 2];
	invATA = new float[2 * 2];
	ATL = new float[2 * 1];

	memset(AT, 0, sizeof(float) * 2 * ptNum);
	memset(ATA, 0, sizeof(float)*2*2);
	memset(invATA, 0, sizeof(float)*2*2);
	memset(ATL, 0, sizeof(float)*2);

	int x, y;
	int sx, sy, ex, ey;
	int halfsize = WinSize / 2;

	sx = feax - halfsize; sy = feay - halfsize;
	ex = feax + halfsize; ey = feay + halfsize;

	X[0] = 1; X[1] = 1;
	dx = 0; dy = 0;
	int iter = 0;
	while ((fabs(X[0])>0.00001 || fabs(X[1])>0.00001) && iter<10)
	{
		int I = 0;
		for (y = sy; y <= ey; y++)
			for (x = sx; x <= ex; x++)
			{
				float gx, gy;
				ComputeImgGradient(img2, w2, h2, x + dx, y + dy, 'x', gx);
				ComputeImgGradient(img2, w2, h2, x + dx, y + dy, 'y', gy);
				A[I * 2 + 0] = gx;
				A[I * 2 + 1] = gy;

				float I1 = img1[y*w1 + x];
				float I2;
				Bilinear_interpolation(img2, w2, h2, x + dx, y + dy, I2);
				L[I] = I1 - I2;

				I++;
			}

		TransposeMatrix(A, ptNum, 2, AT);
		MultiMatrix(AT, A, ATA, 2, 2, ptNum);
		MultiMatrix(AT, L, ATL, 2, 1, ptNum);
		InverseMatrix(ATA, 2, invATA);

		MultiMatrix(invATA, ATL, X, 2, 1, 2);

		dx += X[0];
		dy += X[1];

		iter++;
	}

	//free memory
	delete[]A; A = NULL;
	delete[]L; L = NULL;
	delete[]AT; AT = NULL;
	delete[]ATA; ATA = NULL;
	delete[]invATA; invATA = NULL;
	delete[]ATL; ATL = NULL;
	return 1;
}


int CLeastSquareMatching::InverseMatrix(float *pData, int n, float *pinvData)
{
	if ((pData == NULL) || (n < 1) || pinvData == NULL)
		return false;

	if (n == 2)
	{
		pinvData[0] = pData[3] / (pData[0] * pData[3] - pData[1] * pData[2]);
		pinvData[1] = -pData[1] / (pData[0] * pData[3] - pData[1] * pData[2]);
		pinvData[2] = -pData[2] / (pData[0] * pData[3] - pData[1] * pData[2]);
		pinvData[3] = pData[0] / (pData[0] * pData[3] - pData[1] * pData[2]);
	}

	return 1;
}

int CLeastSquareMatching::LeastSquareMatching(float *img1, int w1, int h1, float *img2, int w2, int h2, int WinSize,
	vector<int> feax, vector<int> feay,
	float *&dx, float *&dy)
{
	int ptNum = (int)feax.size();
	dx = new float[ptNum];
	dy = new float[ptNum];

	int i;
	int halfsize = WinSize / 2;
	for (i = 0; i < ptNum;i++)
	{
		int cx, cy;
		cx = feax[i];
		cy = feay[i];

		//the matching window exceed the boundary of images
		if (cx - halfsize<0 || cx + halfsize>w1 - 1 || cy - halfsize<0 || cy + halfsize>h1 - 1)
		{
			dx[i] = InValidValue;
			dy[i] = InValidValue;
		}
		else
		{
			float tdx, tdy;
			LeastSquareMatching_singlePoint(img1, w1, h1, img2, w2, h2, WinSize,
				cx, cy, tdx, tdy);

			if (fabs(tdx)>2 || fabs(tdy)>2)
			{
				dx[i] = InValidValue;
				dy[i] = InValidValue;
			}
			else
			{
				dx[i] = tdx;
				dy[i] = tdy;
			}
		}
	}

	return 1;
}

int CLeastSquareMatching::MultiMatrix(float *A, float *B, float *AB, int Rows, int Cols, int pubs)
{
	if (A == NULL || B == NULL || AB == NULL)
		return false;

	if (Rows < 1 || Cols < 1 || pubs < 1)
		return false;
	float Temp = 0.0;
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

int CLeastSquareMatching::TransposeMatrix(float *pData, int Rows, int Cols, float *pDataT)
{
	if (pData == NULL || pDataT == NULL)
		return false;

	if (Rows < 1 || Cols < 1)
	{
		return false;
	}

	memset(pDataT, 0, sizeof(float)*Rows*Cols);

	int i = 0, j = 0;
	for (i = 0; i < Cols; i++)
		for (j = 0; j < Rows; j++)
			pDataT[i*Rows + j] = pData[j*Cols + i];

	return 1;
}
