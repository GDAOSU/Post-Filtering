// CVClass.h: interface for the CCVClass class.
//
//////////////////////////////////////////////////////////////////////
/************************************************************************/
/*DPClass  ������Ӿ���

  ����				       ����					             �޸����
  ---------------------------------------------------------------------
  2007.09.03			   �� ��							  ����
 
  File Path: F:\Ketao\==���γ��򱸷�==\AAAAAA-�������(���µ�prj)\===��Ƭ��===\===KetaoLib===\
               ======CVClassLib======\CVClassLib\CVClass.h
  
  Email: ice_tao@21cn.com  
  ---------------------------------------------------------------------	
/*************************** "I love this game!" *********************************************/

#if !defined(AFX_CVCLASS_H__3568131F_74FA_42F1_9D55_645BEE9C4D3F__INCLUDED_)
#define AFX_CVCLASS_H__3568131F_74FA_42F1_9D55_645BEE9C4D3F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#ifndef  EULERNUBER //��Ȼ��e
#define EULERNUBER  2.7182818284590452353602874713527
#endif

#ifndef PI
#define PI  3.141592653589793238462643383279502884197169399375105820974944592308
#endif

#ifndef DOUBLE_PI
#define DOUBLE_PI  6.2831853071795862
#endif

#ifndef RADIAN_OF_10DEGREE//10�ȴ���Ļ���
#define RADIAN_OF_10DEGREE  0.17453292519943295
#endif

#ifndef RADIAN_OF_45DEGREE//8�ȴ���Ļ���
#define RADIAN_OF_45DEGREE  0.78539816339744828
#endif

/*typedef unsigned char       BYTE;*/


#include <math.h>
#include <Windows.h>
/*#include <afx.h>*/
#include <atlstr.h>

//���ݽṹ����
//�����ݶ�����
typedef struct tagGradsPoint
{
	float  magnitude;//����
	float  orientation;//����
	tagGradsPoint()
	{
		magnitude = orientation = 0.f;
	}
	
}GradsPoint;

//36ά�ݶ�ֱ��ͼ
typedef struct tagGrads36Histogram//36����
{
	float histogram[36];
	BYTE   mainOriIndex[36];
	BYTE   mainOriNum;
	tagGrads36Histogram()
	{
		mainOriNum = 0;
		memset(histogram,0,sizeof(float)*36);
		memset(mainOriIndex,0,sizeof(BYTE)*36);
	}
}
Grads36Histogram;

//8ά�ݶ�ֱ��ͼ
typedef struct tagGrads8Histogram//8����
{
	float histogram[8];
	tagGrads8Histogram()
	{
		memset(histogram,0,sizeof(float)*8);
	}
}
Grads8Histogram;


//SIFT������
typedef struct tagSIFTPOINT
{
	float imgX,imgY;//�������(���½�ԭ��)
	float mainGradsOri;//���ݶȷ���
	float mainGradsMgnt;//���ݶȷ����ֵ
	float featureVector[128];//128ά����
	//������
	Grads36Histogram histgramOf36;//��ǰ������������������(�������������ĵ�)
	tagSIFTPOINT()
	{
		imgX = imgY  = -1.f;
		mainGradsOri = mainGradsMgnt = 0.f;
		memset(featureVector,0,sizeof(float)*128);
	}

}SIFTPoint;

//����ֱ��ͼ����
typedef struct tagHstGramSort
{
	float hstValue;
	int   index;
	tagHstGramSort()
	{
		hstValue = -1.f;
		index    = -1;
	}

}HstGramSort;

typedef struct tagGridSiftPoints
{
	SIFTPoint* pPts;
	int ptNum;
	int curMemSize;
	tagGridSiftPoints()
	{
		pPts = NULL;
		ptNum=0;
		curMemSize = 0;
	}
	
}GridSiftPoints;






inline double  EcldnDst(SIFTPoint lSiftPt,SIFTPoint rSiftPt)//ŷʽ����
{
	double dff = 0.000, distance =0.000;
	for(int i =0;i<128;i++)
	{
		dff = lSiftPt.featureVector[i] - rSiftPt.featureVector[i];		distance+=dff*dff;
	}
	return distance;
}

inline double  StrtDst(SIFTPoint lSiftPt,SIFTPoint rSiftPt)//ŷʽ����
{
	double distance =0.000;
	for(int i =0;i<128;i++)
	{
		distance+= fabs(lSiftPt.featureVector[i] - rSiftPt.featureVector[i]);
	}
	return distance;
}



class  CCVClass  
{
public:

//	int CompareStrategy, 1==������ƥ�䣬ʹ����Ӱ���Ҳ�60�������������
//						 0==������ƥ�䣬ʹ����Ӱ���²�60�������������
//						 2==��ȷ��ƥ�䣬ʹ����Ӱ�������������

	void CompareSiftPoints(	SIFTPoint* pLSiftPts,int lSiftPtsNum,SIFTPoint* pRSiftPts,int rSiftPtsNum,
							float* lx,float* ly,float* rx,float* ry,int& matchPtNum,
							int lWidth=0,int lHeight=0,int rWidth=0,int rHeight=0,float overlapX=0,float overlapY=0,
							float searchX=0,float searchY=0,int maxNum=0, int CompareStrategy=2);

	void CompareSiftEpipolarPoints(	SIFTPoint* pLSiftPts,int lSiftPtsNum,
									SIFTPoint* pRSiftPts,int rSiftPtsNum,
									int imgWidth,int imgHeight,
									float focus,float x0,float y0,float pixelSize,
									double k1,double k2,double p1,double p2,
									double lXs,double lYs,double lZs,double lPhi,double lOmega,double lKappa,
									double rXs,double rYs,double rZs,double rPhi,double rOmega,double rKappa,
									float* lx,float* ly,float* rx,float* ry,int& matchPtNum);
	
	void Cal8Histogram(GradsPoint* pGradsPts,int ptNum,Grads8Histogram& histogram_8);
	void DescriptSIFTPoint(	GradsPoint *&pGradsPts, int imgWidth, int imgHeight,SIFTPoint& siftPt);
	void NormalizeSiftPoints(SIFTPoint* pSiftPts,int ptNum,float threshold = 0.2);
	BOOL ReadSiftPts(SIFTPoint*& pSiftPt, int& ptNum, CString filePath);
	BOOL OutputSiftPts(SIFTPoint* pSiftPt,int ptNum,CString filePath);
	void CalSiftFeaturePtMainOri(	GradsPoint *&pGradsPts, int imgWidth, int imgHeight,
									SIFTPoint& siftPt);
	void CalImgGrads(BYTE*& pGrey,int imgWidth,int imgHeight,GradsPoint*& pGradsPts);

	CCVClass();
	virtual ~CCVClass();	


	//��ɫת�ڰ�
	/*
	TransClrToGreyImg(	BYTE *pClrImg, //��ɫӰ��
						int width, int height,//Ӱ�����
						BYTE *&pGreyImg)//ת����ĻҶ�Ӱ��;
	*/
	void TransClrToGreyImg(BYTE *pClrImg, int width, int height, BYTE *&pGreyImg);
	
	//Ӱ����Сһ��(ÿ��������ȡһ������)
	/*
	void DownSampleImageBy2(BYTE*pOriImg,//ԭʼӰ��
							int imgWidth,int imgHeight,//ԭʼӰ����
							BYTE* pResmplImg)//��Сһ���Ӱ��
	*/
	void DownSampleImageBy2(BYTE*pOriImg,int imgWidth,int imgHeight,BYTE* pResmplImg);
	
	//�������������
	/*	
	void ZoomImage(	BYTE* pOriImg,//ԭʼӰ��
					int oriImgWidth,int oriImgHeight,//ԭʼӰ����
					BYTE*& pZoomImg,//���ź�Ӱ��
					int zoomImgWidth,int zoomImgHeight)///���ź�Ӱ����
	*/
	void ZoomImage(BYTE* pOriImg,int oriImgWidth,int oriImgHeight,BYTE* pZoomImg,int zoomImgWidth,int zoomImgHeight);

	//���ɸ�˹ģ��
	/*
	GenGaussTemplate(	float*& guassTemplate,//ģ������(�������½���㣬����ɨ��)
						int wndWidth,int wndHeight, //���ڿ��)
						float detta,		  //��˹����dettaֵ
						BOOL bNormalized = TRUE)//�Ƿ��һ��
	*/
	void GenGaussTemplate(float*& guassTemplate,int wndWidth,int wndHeight,float detta,BOOL bNormalized = TRUE);
	//Ӱ����
	/*
	ImageFiltering(	BYTE* pOriImg,//ԭʼӰ��ָ��
					int width,int height,//ԭʼӰ����
					float* filter,//�˲�����ϵ��
					int filterWidth,int filterHeight,//�˲����ڳߴ�
					BYTE*& pFilterImg)//�˲���Ӱ��
	*/
	void ImageFiltering(BYTE* pOriImg,int width,int height,
						float* filter,int filterWidth,int filterHeight,
						BYTE*& pFilterImg);

	//��ȡSIFT����
	/*	
	ExtractSIFTPoints(	BYTE* pGrey,//Ӱ��ָ��
						int imgWidth,int imgHeight,//Ӱ����
						int dffThreshold,//����ֵ��ֵ
						int& ptNum,//����;
						SIFTPoint*& pSiftPts,//���ص�������
						BOOL bZoomOutImage = FALSE,//�Ƿ�Ŵ�ԭʼӰ��
						CString outPutDir = "NULL");
	*/

	void ExtractSIFTPoints(	BYTE*& pGrey,int imgWidth,int imgHeight,int dffThreshold,int& ptNum,SIFTPoint*& pSiftPts,
							CString outPutDir="NULL",BOOL bZoomOutImage=TRUE);

	
	//��ȡSIFT����(���ָ�����ÿ������ȡ���ֵ)
	/*	
	ExtractSIFTPointsByGrid(	BYTE* pGrey,//Ӱ��ָ��
						int imgWidth,int imgHeight,//Ӱ����
						int dffThreshold,//����ֵ��ֵ
						int GridRows,//��������
						int GridCols,//��������
						int& ptNum,//����;
						SIFTPoint*& pSiftPts,//���ص�������
						BOOL bZoomOutImage = FALSE,//�Ƿ�Ŵ�ԭʼӰ��
						CString outPutDir = "NULL");
	*/
	void ExtractSIFTPointsByGrid(	BYTE* pGrey,int imgWidth,int imgHeight,
									int dffThreshold,int GridRows,int GridCols,
									int& ptNum,SIFTPoint*& pSiftPts,
									CString outPutDir="NULL",BOOL bZoomOutImage=TRUE);

	

	//SIFT����ƥ��
	/*
	BOOL SIFTMatch(	BYTE* pLImg,//��Ӱ������ָ��
					int lWidth,int lHeight,//��Ӱ����
					int lClrNum,//��Ӱ��ͨ������(1-�ڰ�;3-��ɫ)
					BYTE* pRImg,//��Ӱ������ָ��
					int rWidth,int rHeight,//��Ӱ����
					int rClrNum,//��Ӱ��ͨ������(1-�ڰ�;3-��ɫ)
					int& matchPtNum,//ƥ�����
					float*& lx,float*& ly,float*& rx,float*& ry,//ƥ������
					CString outPutDir = "NULL",//��ʱ������(���=="NULL",������м���)
					CString lSiftPtFile = "NULL",//��Ƭ�������ļ�(���=="NULL",�����)
					CString rSiftPtFile = "NULL");//��Ƭ�������ļ�(���=="NULL",�����)
	*/
	BOOL SIFTMatch(	BYTE*& pLImg,int lWidth,int lHeight,int lClrNum,
					BYTE*& pRImg,int rWidth,int rHeight,int rClrNum,
					int& matchPtNum,float*& lx,float*& ly,float*& rx,float*& ry,
					CString outPutDir = "NULL",
					CString lSiftPtFile = "NULL",
					CString rSiftPtFile = "NULL",
					float overlapX=0,float overlapY=0,float searchX=0,float searchY=0,int maxNum=0);
	//SIFT����ƥ��(����Ӱ�񻮷�Ϊ������ÿ��������ȡ����ֵ���ĵ����ƥ��)
	/*
	BOOL SIFTMatchByGrid(	BYTE* pLImg,//��Ӱ������ָ��
							int lWidth,int lHeight,//��Ӱ����
							int lClrNum,//��Ӱ��ͨ������(1-�ڰ�;3-��ɫ)
							BYTE* pRImg,//��Ӱ������ָ��
							int rWidth,int rHeight,//��Ӱ����
							int rClrNum,//��Ӱ��ͨ������(1-�ڰ�;3-��ɫ)
							int gridRows,//��������
							int gridCols,//��������
							int& matchPtNum,//ƥ�����
							float*& lx,float*& ly,float*& rx,float*& ry,//ƥ������
							CString outPutDir = "NULL",//��ʱ������(���=="NULL",������м���)
							CString lSiftPtFile = "NULL",//��Ƭ�������ļ�(���=="NULL",�����)
							CString rSiftPtFile = "NULL");//��Ƭ�������ļ�(���=="NULL",�����)
	*/	
	BOOL SIFTMatchByGrid(	BYTE*& pLImg,int lWidth,int lHeight,int lClrNum,
							BYTE*& pRImg,int rWidth,int rHeight,int rClrNum,
							int gridRows,int gridCols,
							int& matchPtNum,float*& lx,float*& ly,float*& rx,float*& ry,
							CString outPutDir = "NULL",
							CString lSiftPtFile = "NULL",
							CString rSiftPtFile = "NULL",
							float overlapX=0,float overlapY=0,float searchX=0,float searchY=0,int maxNum=0,int dffThreshold =3, int CompareStrategy=2);


};

#endif // !defined(AFX_CVCLASS_H__3568131F_74FA_42F1_9D55_645BEE9C4D3F__INCLUDED_)
