// CVClass.h: interface for the CCVClass class.
//
//////////////////////////////////////////////////////////////////////
/************************************************************************/
/*DPClass  计算机视觉类

  日期				       作者					             修改情况
  ---------------------------------------------------------------------
  2007.09.03			   柯 涛							  创建
 
  File Path: F:\Ketao\==柯涛程序备份==\AAAAAA-程序更新(读新的prj)\===刀片机===\===KetaoLib===\
               ======CVClassLib======\CVClassLib\CVClass.h
  
  Email: ice_tao@21cn.com  
  ---------------------------------------------------------------------	
/*************************** "I love this game!" *********************************************/

#if !defined(AFX_CVCLASS_H__3568131F_74FA_42F1_9D55_645BEE9C4D3F__INCLUDED_)
#define AFX_CVCLASS_H__3568131F_74FA_42F1_9D55_645BEE9C4D3F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#ifndef  EULERNUBER //自然数e
#define EULERNUBER  2.7182818284590452353602874713527
#endif

#ifndef PI
#define PI  3.141592653589793238462643383279502884197169399375105820974944592308
#endif

#ifndef DOUBLE_PI
#define DOUBLE_PI  6.2831853071795862
#endif

#ifndef RADIAN_OF_10DEGREE//10度代表的弧度
#define RADIAN_OF_10DEGREE  0.17453292519943295
#endif

#ifndef RADIAN_OF_45DEGREE//8度代表的弧度
#define RADIAN_OF_45DEGREE  0.78539816339744828
#endif

/*typedef unsigned char       BYTE;*/


#include <math.h>
#include <Windows.h>
/*#include <afx.h>*/
#include <atlstr.h>

//数据结构定义
//单点梯度特征
typedef struct tagGradsPoint
{
	float  magnitude;//幅度
	float  orientation;//方向
	tagGradsPoint()
	{
		magnitude = orientation = 0.f;
	}
	
}GradsPoint;

//36维梯度直方图
typedef struct tagGrads36Histogram//36方向
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

//8维梯度直方图
typedef struct tagGrads8Histogram//8方向
{
	float histogram[8];
	tagGrads8Histogram()
	{
		memset(histogram,0,sizeof(float)*8);
	}
}
Grads8Histogram;


//SIFT描述符
typedef struct tagSIFTPOINT
{
	float imgX,imgY;//像点坐标(左下角原点)
	float mainGradsOri;//主梯度方向
	float mainGradsMgnt;//主梯度方向幅值
	float featureVector[128];//128维向量
	//调试用
	Grads36Histogram histgramOf36;//当前特征点主方向描述符(特征描述符中心点)
	tagSIFTPOINT()
	{
		imgX = imgY  = -1.f;
		mainGradsOri = mainGradsMgnt = 0.f;
		memset(featureVector,0,sizeof(float)*128);
	}

}SIFTPoint;

//用于直方图排序
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






inline double  EcldnDst(SIFTPoint lSiftPt,SIFTPoint rSiftPt)//欧式距离
{
	double dff = 0.000, distance =0.000;
	for(int i =0;i<128;i++)
	{
		dff = lSiftPt.featureVector[i] - rSiftPt.featureVector[i];		distance+=dff*dff;
	}
	return distance;
}

inline double  StrtDst(SIFTPoint lSiftPt,SIFTPoint rSiftPt)//欧式距离
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

//	int CompareStrategy, 1==航带内匹配，使用左影像右部60％像幅的特征点
//						 0==航带间匹配，使用左影像下部60％像幅的特征点
//						 2==不确定匹配，使用左影像的所有特征点

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


	//彩色转黑白
	/*
	TransClrToGreyImg(	BYTE *pClrImg, //彩色影像
						int width, int height,//影像宽，高
						BYTE *&pGreyImg)//转换后的灰度影像;
	*/
	void TransClrToGreyImg(BYTE *pClrImg, int width, int height, BYTE *&pGreyImg);
	
	//影像缩小一半(每两个像素取一个像素)
	/*
	void DownSampleImageBy2(BYTE*pOriImg,//原始影像
							int imgWidth,int imgHeight,//原始影像宽高
							BYTE* pResmplImg)//缩小一半后影像
	*/
	void DownSampleImageBy2(BYTE*pOriImg,int imgWidth,int imgHeight,BYTE* pResmplImg);
	
	//任意比例尺缩放
	/*	
	void ZoomImage(	BYTE* pOriImg,//原始影像
					int oriImgWidth,int oriImgHeight,//原始影像宽高
					BYTE*& pZoomImg,//缩放后影像
					int zoomImgWidth,int zoomImgHeight)///缩放后影像宽高
	*/
	void ZoomImage(BYTE* pOriImg,int oriImgWidth,int oriImgHeight,BYTE* pZoomImg,int zoomImgWidth,int zoomImgHeight);

	//生成高斯模板
	/*
	GenGaussTemplate(	float*& guassTemplate,//模板数组(窗口左下角起点，按行扫描)
						int wndWidth,int wndHeight, //窗口宽高)
						float detta,		  //高斯函数detta值
						BOOL bNormalized = TRUE)//是否归一化
	*/
	void GenGaussTemplate(float*& guassTemplate,int wndWidth,int wndHeight,float detta,BOOL bNormalized = TRUE);
	//影像卷积
	/*
	ImageFiltering(	BYTE* pOriImg,//原始影像指针
					int width,int height,//原始影像宽高
					float* filter,//滤波窗口系数
					int filterWidth,int filterHeight,//滤波窗口尺寸
					BYTE*& pFilterImg)//滤波后影像
	*/
	void ImageFiltering(BYTE* pOriImg,int width,int height,
						float* filter,int filterWidth,int filterHeight,
						BYTE*& pFilterImg);

	//提取SIFT算子
	/*	
	ExtractSIFTPoints(	BYTE* pGrey,//影像指针
						int imgWidth,int imgHeight,//影像宽高
						int dffThreshold,//特征值阈值
						int& ptNum,//点数;
						SIFTPoint*& pSiftPts,//返回的特征点
						BOOL bZoomOutImage = FALSE,//是否放大原始影像
						CString outPutDir = "NULL");
	*/

	void ExtractSIFTPoints(	BYTE*& pGrey,int imgWidth,int imgHeight,int dffThreshold,int& ptNum,SIFTPoint*& pSiftPts,
							CString outPutDir="NULL",BOOL bZoomOutImage=TRUE);

	
	//提取SIFT算子(划分格网，每个格网取最大值)
	/*	
	ExtractSIFTPointsByGrid(	BYTE* pGrey,//影像指针
						int imgWidth,int imgHeight,//影像宽高
						int dffThreshold,//特征值阈值
						int GridRows,//格网行数
						int GridCols,//格网列数
						int& ptNum,//点数;
						SIFTPoint*& pSiftPts,//返回的特征点
						BOOL bZoomOutImage = FALSE,//是否放大原始影像
						CString outPutDir = "NULL");
	*/
	void ExtractSIFTPointsByGrid(	BYTE* pGrey,int imgWidth,int imgHeight,
									int dffThreshold,int GridRows,int GridCols,
									int& ptNum,SIFTPoint*& pSiftPts,
									CString outPutDir="NULL",BOOL bZoomOutImage=TRUE);

	

	//SIFT算子匹配
	/*
	BOOL SIFTMatch(	BYTE* pLImg,//左影像数据指针
					int lWidth,int lHeight,//左影像宽高
					int lClrNum,//左影像通道个数(1-黑白;3-彩色)
					BYTE* pRImg,//右影像数据指针
					int rWidth,int rHeight,//右影像宽高
					int rClrNum,//右影像通道个数(1-黑白;3-彩色)
					int& matchPtNum,//匹配点数
					float*& lx,float*& ly,float*& rx,float*& ry,//匹配数组
					CString outPutDir = "NULL",//临时输出结果(如果=="NULL",则不输出中间结果)
					CString lSiftPtFile = "NULL",//左片特征点文件(如果=="NULL",则不输出)
					CString rSiftPtFile = "NULL");//右片特征点文件(如果=="NULL",则不输出)
	*/
	BOOL SIFTMatch(	BYTE*& pLImg,int lWidth,int lHeight,int lClrNum,
					BYTE*& pRImg,int rWidth,int rHeight,int rClrNum,
					int& matchPtNum,float*& lx,float*& ly,float*& rx,float*& ry,
					CString outPutDir = "NULL",
					CString lSiftPtFile = "NULL",
					CString rSiftPtFile = "NULL",
					float overlapX=0,float overlapY=0,float searchX=0,float searchY=0,int maxNum=0);
	//SIFT算子匹配(将左影像划分为格网，每个格网提取特征值最大的点进行匹配)
	/*
	BOOL SIFTMatchByGrid(	BYTE* pLImg,//左影像数据指针
							int lWidth,int lHeight,//左影像宽高
							int lClrNum,//左影像通道个数(1-黑白;3-彩色)
							BYTE* pRImg,//右影像数据指针
							int rWidth,int rHeight,//右影像宽高
							int rClrNum,//右影像通道个数(1-黑白;3-彩色)
							int gridRows,//格网行数
							int gridCols,//格网列数
							int& matchPtNum,//匹配点数
							float*& lx,float*& ly,float*& rx,float*& ry,//匹配数组
							CString outPutDir = "NULL",//临时输出结果(如果=="NULL",则不输出中间结果)
							CString lSiftPtFile = "NULL",//左片特征点文件(如果=="NULL",则不输出)
							CString rSiftPtFile = "NULL");//右片特征点文件(如果=="NULL",则不输出)
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
