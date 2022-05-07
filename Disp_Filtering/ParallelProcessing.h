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


////��������̷߳���ṹ��////
typedef struct tagMatrixMutiProcess
{
	int nNumOfProcessors;
	int nrow;
	int np;
	int ncol;
	int nThreadID;     					//��ǰ�̺߳�
	double *A;
	double *B;
	double *Result;
	int    *bFinish;
}MatrixMutiProcess;

////SIFTƥ���̷߳���ṹ��////
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

	int nThreadID;     					//��ǰ�̺߳�
	
	SIFTPoint* pLSiftPts;
	SIFTPoint* pRSiftPts;
	int      * rSiftPtMatched;
	int      * pMatchedPtID;
	int      * bFinish;
}SIFTMatchProcess;

////Ӱ����ת�ز����̷߳���ṹ��////
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

	int nThreadID;     					//��ǰ�̺߳�

	unsigned char* lpImage;
	BYTE *pImageStrip;
	int    *bFinish;
}RotateImageProcess;

////��������ȡ�̷߳���ṹ��////
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

	int nThreadID;     					//��ǰ�̺߳�

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

	//��������߳̽�����־
	int	 *m_bMatrixMutiFinish; 
	MatrixMutiProcess *m_pMatrixMutiProcess;
	void MatrixMultiply_MultiThread(double *A, double *B, double *Result, int m, int p, int n,int maxThreadNum = 99);

	//SIFTƥ���߳̽�����־
	int	 *m_bSIFTMatchFinish; 
	SIFTMatchProcess *m_pSIFTMatchProcess;
	void CompareSiftPoints_MultiThread(	SIFTPoint* pLSiftPts,int lSiftPtsNum,SIFTPoint* pRSiftPts,int rSiftPtsNum,
							float* lx,float* ly,float* rx,float* ry,int& matchPtNum,
							int lWidth=0,int lHeight=0,int rWidth=0,int rHeight=0,float overlapX=0,float overlapY=0,
							float searchX=0,float searchY=0,int maxNum=0,int maxThreadNum=99, int CompareStrategy=2);


	//Ӱ����ת�ز����߳̽�����־
	int	 *m_bRotateImageFinish; 
	RotateImageProcess *m_pRotateImageProcess;
	void RotateImageWinToHorizon_MultiThread(unsigned char* lpImage,int row,int col,float x,float y,float fRotateAngle,
							BYTE *pImageStrip,int nImageWinWidth,int nImageWinLength,int maxThreadNum=99);


	void variance(unsigned char* pg,int col,int length,int width,float n,float* v,float* sg);
	void variance2(unsigned char* pg,int column,int width,int length,float n,int x_range,int y_range,float* v,float* sg);

	void correlationCoefficientMatch(
				unsigned char* pgl,				//��Ӱ������
				int rowl,int coll,				//��Ӱ��������
				unsigned char* pgr,				//��Ӱ������
				int rowr,int colr,				//��Ӱ��������
				int xl,int yl,					//��ƥ�������Ӱ���е�����
				int xr0,int yr0,				//��Ӧƥ�������Ӱ���еĳ�ʼ����
				int width,int length,			//Ŀ�괰�ڵĴ�С
				int x_range,int y_range,		//�������ڵĴ�С
				int maximum_n,					//��Ҫ��ȡ�ĺ�ѡƥ�����
				unsigned char* maxn,			//ʵ�ʷ��صĺ�ѡƥ������
				float* maxd_x,float* maxd_y,	//��ѡƥ�������
				unsigned char* maxw,			//��ѡƥ������ϵ��
				double fRotationAngle = 0.0,	//����Ӱ�����ڵ���ת��
				BOOL bEpipolarLineMatch = FALSE,//�Ƿ���к���Լ�����ƥ��
				double epipolarA = 0,double epipolarB = 0,double epipolarC = 0,	//���߷���
				BOOL sameStrip = TRUE);			//�Ƿ�ͬһ������

	void RotateImageWinToHorizon(unsigned char* lpImage,int row,int col,float x,float y,float fRotateAngle,BYTE *pImageStrip,int nImageWinWidth,int nImageWinLength);
	void RotateImageWinToHorizon(float xStart,float yStart,float xEnd,float yEnd,unsigned char* lpImage,int row,int col,unsigned char* pImageStrip,int nImageWinWidth,int nImageWinLength);

	double DistPtToLine(fPoint pt, double a,double b,double c);
//	double CalcInteriorPixelValue(fPoint ptGuest,unsigned char* lpImage,int row,int col);
	void   CalculateEpipolarLine(double x, double y,double *NonMetricReorPara,double &A,double &B,double &C);


	//Ӱ����������ȡ�߳̽�����־
	int	 *m_bHarrisFeatureFinish; 
	HarrisFeatureProcess *m_pHarrisFeatureProcess;
	//harris���ӿ����㷨----->��ͼ��ֳɸ�����ÿ�������ȡһ�������㣬����HarrisFeaturePointExtract()
	int HarrisFeaturePointExtractGrid_MultiThread(
									unsigned char*pImg,			//����ȡ�������Ӱ������
									int Height,int Width,		//Ӱ��߿�
									float *px,float*py,float *attri,//�������������꣬����Ӧÿ�����ǿ��ֵ
									int gridRow = 45,			//�����߶�
									int gridCol = 45,           //�������
									char*outputFile = NULL,		//��������㣬�Լ�������Ϣ
									int win = 3,				//�ֲ����ڴ�С��һ��ȡ3����
									int dg = 10,                //һ�ײ����ֵ����С����ֵ������һ������
									bool bExactLocation=true,
									int maxThreadNum=99);  //�Ƿ�ʹ��forstner��λ���ӽ��о�ȷ��λ             

};

#endif // !defined(AFX_PARALLELPROCESSING_H__85102E49_CF71_4399_AD19_8B73629C7796__INCLUDED_)
