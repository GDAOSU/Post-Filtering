// ParallelProcessing.h: interface for the CParallelProcessing class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PARALLELPROCESSING_H__85102E49_CF71_4399_AD19_8B73629C7796__INCLUDED_)
#define AFX_PARALLELPROCESSING_H__85102E49_CF71_4399_AD19_8B73629C7796__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#include"CVClass.h"

#define GA_PI  3.141592653589793238462643383279502884197169399375105820974944592308
#define GA_INVALID	-9999
#define GA_BASELINE	1000
#define GA_TTT 0.707106781186548
#define GA_QE  0.000000001


////矩阵相乘线程分配结构体////
typedef struct tagMatrixMutiProcess
{
	int nNumOfProcessors;
	int nrow;
	int np;
	int ncol;
	int nThreadID;     					//当前线程号
	double *A;
	double *B;
	double *Result;
	int    *bFinish;
}MatrixMutiProcess;

////SIFT匹配线程分配结构体////
typedef struct tagSIFTMatchProcess
{
	int nNumOfProcessors;
	int lSiftPtsNum;
	int rSiftPtsNum;
	int lWidth;
	int lHeight;
	int rWidth;
	int rHeight;
	float overlapX;
	float overlapY;
	float searchX;
	float searchY;
	int CompareStrategy;

	int nThreadID;     					//当前线程号
	
	SIFTPoint* pLSiftPts;
	SIFTPoint* pRSiftPts;
	int      * rSiftPtMatched;
	int      * pMatchedPtID;
	int      * bFinish;
}SIFTMatchProcess;

////影像旋转重采样线程分配结构体////
typedef struct tagRotateImageProcess
{
	int nNumOfProcessors;
	int row;
	int col;
	float x;
	float y;
	float fRotateAngle;
	int nImageWinWidth;
	int nImageWinLength;

	int nThreadID;     					//当前线程号

	unsigned char* lpImage;
	BYTE *pImageStrip;
	int    *bFinish;
}RotateImageProcess;

////特征点提取线程分配结构体////
typedef struct tagHarrisFeatureProcess
{
	int nNumOfProcessors;
	unsigned char*pImg;
	int Height;
	int Width;
	float *px;
	float *py;
	float *attri;
	int gridRow;
	int gridCol;
	int win;
	int dg;
	bool bExactLocation;

	int nThreadID;     					//当前线程号

	int    *bFinish;
}HarrisFeatureProcess;


#ifndef FPOINT
#define FPOINT
typedef struct tagfPoint//2D point
{
	double x;
	double y;
}fPoint;
#endif

class CParallelProcessing  
{
public:
	CParallelProcessing();
	virtual ~CParallelProcessing();

	//矩阵相乘线程结束标志
	int	 *m_bMatrixMutiFinish; 
	MatrixMutiProcess *m_pMatrixMutiProcess;
	void MatrixMultiply_MultiThread(double *A, double *B, double *Result, int m, int p, int n,int maxThreadNum = 99);

	//SIFT匹配线程结束标志
	int	 *m_bSIFTMatchFinish; 
	SIFTMatchProcess *m_pSIFTMatchProcess;
	void CompareSiftPoints_MultiThread(	SIFTPoint* pLSiftPts,int lSiftPtsNum,SIFTPoint* pRSiftPts,int rSiftPtsNum,
							float* lx,float* ly,float* rx,float* ry,int& matchPtNum,
							int lWidth=0,int lHeight=0,int rWidth=0,int rHeight=0,float overlapX=0,float overlapY=0,
							float searchX=0,float searchY=0,int maxNum=0,int maxThreadNum=99, int CompareStrategy=2);


	//影像旋转重采样线程结束标志
	int	 *m_bRotateImageFinish; 
	RotateImageProcess *m_pRotateImageProcess;
	void RotateImageWinToHorizon_MultiThread(unsigned char* lpImage,int row,int col,float x,float y,float fRotateAngle,
							BYTE *pImageStrip,int nImageWinWidth,int nImageWinLength,int maxThreadNum=99);


	void variance(unsigned char* pg,int col,int length,int width,float n,float* v,float* sg);
	void variance2(unsigned char* pg,int column,int width,int length,float n,int x_range,int y_range,float* v,float* sg);

	void correlationCoefficientMatch(
				unsigned char* pgl,				//左影像数据
				int rowl,int coll,				//左影像行列数
				unsigned char* pgr,				//右影像数据
				int rowr,int colr,				//右影像行列数
				int xl,int yl,					//待匹配点在左影像中的坐标
				int xr0,int yr0,				//对应匹配点在右影像中的初始坐标
				int width,int length,			//目标窗口的大小
				int x_range,int y_range,		//搜索窗口的大小
				int maximum_n,					//需要获取的候选匹配个数
				unsigned char* maxn,			//实际返回的候选匹配点个数
				float* maxd_x,float* maxd_y,	//候选匹配点坐标
				unsigned char* maxw,			//候选匹配点相关系数
				double fRotationAngle = 0.0,	//左右影像间存在的旋转角
				BOOL bEpipolarLineMatch = FALSE,//是否进行核线约束相关匹配
				double epipolarA = 0,double epipolarB = 0,double epipolarC = 0,	//核线方程
				BOOL sameStrip = TRUE);			//是否同一条航带

	void RotateImageWinToHorizon(unsigned char* lpImage,int row,int col,float x,float y,float fRotateAngle,BYTE *pImageStrip,int nImageWinWidth,int nImageWinLength);
	void RotateImageWinToHorizon(float xStart,float yStart,float xEnd,float yEnd,unsigned char* lpImage,int row,int col,unsigned char* pImageStrip,int nImageWinWidth,int nImageWinLength);

	double DistPtToLine(fPoint pt, double a,double b,double c);
//	double CalcInteriorPixelValue(fPoint ptGuest,unsigned char* lpImage,int row,int col);
	void   CalculateEpipolarLine(double x, double y,double *NonMetricReorPara,double &A,double &B,double &C);


	//影像特征点提取线程结束标志
	int	 *m_bHarrisFeatureFinish; 
	HarrisFeatureProcess *m_pHarrisFeatureProcess;
	//harris算子快速算法----->将图像分成格网，每块格网提取一个特征点，调用HarrisFeaturePointExtract()
	int HarrisFeaturePointExtractGrid_MultiThread(
									unsigned char*pImg,			//待提取特征点的影像数据
									int Height,int Width,		//影像高宽
									float *px,float*py,float *attri,//返回特征点坐标，和相应每个点的强度值
									int gridRow = 45,			//格网高度
									int gridCol = 45,           //格网宽度
									char*outputFile = NULL,		//输出特征点，以及格网信息
									int win = 3,				//局部窗口大小，一般取3即可
									int dg = 10,                //一阶差分阈值，若小于阈值则不作进一步处理
									bool bExactLocation=true,
									int maxThreadNum=99);  //是否使用forstner定位算子进行精确定位             

};

#endif // !defined(AFX_PARALLELPROCESSING_H__85102E49_CF71_4399_AD19_8B73629C7796__INCLUDED_)
