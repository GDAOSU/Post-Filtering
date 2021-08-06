#pragma once
#include "math.h"
#include <vector>
using namespace std;

class CLeastSquareMatching
{
public:
	CLeastSquareMatching();
	~CLeastSquareMatching();

public:
	/*
	main function for least squares
	feax1, feay1: image coordinates of the feature points in img1
	dx, dy: system bias, the coordinates of corresponding points can be computed by: feax1 - dx, feay1 - dy;

	GridSize: the size of tile where only one feature point is extracted

	MchSize: the matching window size
	*/
	int ComputeSysBiasBasedonLeastSquareMatching(float *img1, int w1, int h1, float *img2, int w2, int h2,
		int *&feax1, int *&feay1, int &ptNum, float *&dx, float *&dy, int GridSize = 100, int MchSize = 15);

public:
	/*
	Change the intensity of the second image into the scale of the first image
	*/
	int IntensityScaleMatchingBetweenTwoImgs(float *img1, int w1, int h1, float *img2, int w2, int h2);

	/*
	filtering image to get obvious feature points
	*/
	int LaplaceFiltering(float *img, int w, int h, float *mask, int size, float *&img_filter);

	/*
	Get feature points from filtered images
	*/
	int ExtractFeaturePoints(float *img, int w, int h, int GridSize, vector<int> &feax, vector<int> &feay);

	/*
	Least squares matching for img
	*/
	int LeastSquareMatching(float *img1, int w1, int h1, float *img2, int w2, int h2, int WinSize,
		vector <int> feax, vector <int> feay, float *&dx, float *&dy);

	/*
	Least squares for single point
	*/
	int LeastSquareMatching_singlePoint(float *img1, int w1, int h1, float *img2, int w2, int h2, int WinSize,
		int feax, int feay, float &dx, float &dy);

	/*
	compute gradient of x and y direction
	*/
	int ComputeImgGradient(float *img, int w, int h, float x, float y, char direction, float &gradient);

	/*
	Bilinear interpolation
	*/
	int Bilinear_interpolation(float *img, int w, int h, float ix, float iy, float &intensity);

	/*
	matrix computation
	*/
	int TransposeMatrix(float *pData, int Rows, int Cols, float *pDataT);
	int MultiMatrix(float *A, float *B, float *AB, int Rows, int Cols, int pubs);
	int InverseMatrix(float *pData, int n, float *pinvData);
};

