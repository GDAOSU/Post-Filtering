// CVClass.cpp: implementation of the CCVClass class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "CVClass.h"
#include "direct.h"
//#include "SpXImage.h"

// #ifdef _DEBUG
// #undef THIS_FILE
// static char THIS_FILE[]=__FILE__;
// #define new DEBUG_NEW
// #endif






int SortHstGram(const void *arg1, const void *arg2)//�ȵ���
{
	HstGramSort* leftPt	   = (HstGramSort*) arg1;
	HstGramSort* rightPt   = (HstGramSort*) arg2;
	//��С����
	if (leftPt->hstValue > rightPt->hstValue) 
		return -1;
	else if (leftPt->hstValue == rightPt->hstValue)
	{
		return 0;
	}
		
	else
		return 1;
}


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCVClass::CCVClass()
{

}

CCVClass::~CCVClass()
{

}

void CCVClass::GenGaussTemplate(float*& guassTemplate,int wndWidth,int wndHeight,float detta,BOOL bNormalized)
{
	int i,j;
	float tempValue,sum;
	sum = 0.f;
	int cntX = wndWidth/2;
	int cntY = wndHeight/2;
	float x,y;
	int count;
	
	count=0;
	sum = 0.f;

	for(i=0;i<wndHeight;i++)
	{
		for(j=0;j<wndWidth;j++)
		{
			x = float(j-cntX);
			y = float(i-cntY);
			tempValue = float( pow(EULERNUBER, -0.5*(x*x + y*y)/(detta*detta)) / ( 2.f*PI*detta*detta ) );
			guassTemplate[count] = tempValue;
			sum+=tempValue;
			count++;
		}
	}
	//��һ��
	if (bNormalized==TRUE)
	{
		count = 0;
		for(i=0;i<wndHeight;i++)
		{
			for(j=0;j<wndWidth;j++)
			{
				guassTemplate[count] = guassTemplate[count]*1.f/sum;
				count++;
			}
		}
		
	}

}
//Ӱ����
void CCVClass::ImageFiltering(BYTE* pOriImg,int width,int height,float* filter,int filterWidth,int filterHeight,BYTE*& pFilterImg)
{
	int i,j,k,l;
	int filterBufLenght = filterWidth*filterHeight;
	int* smplPos = new int[filterBufLenght];//�˲����ڸ�����ָ��λ��(���½�ԭ��)
	int cntPtPos = 0;//�������ĵ�Ӱ��ָ��λ��
	int semiFilterHeight = filterHeight/2;
	int semiFilterWidth  = filterWidth/2;
	int tempCountWnd;
	int tempWndPos = 0;
	double wndSigma = 0.000;
	


	for(i=0;i<height;i++)
	{
		for(j=0;j<width;j++)
		{

			if (i<=semiFilterHeight	||i>=height-1 - semiFilterHeight || 
				j<=semiFilterWidth	||j>=width-1- semiFilterWidth) 
			{
				cntPtPos++;
				continue;
			}
			tempCountWnd = 0;
			tempWndPos   = cntPtPos - width*semiFilterHeight - semiFilterWidth;//�������½ǵ�Ӱ��λ��
			for(k=0;k<filterHeight;k++)
			{				
				for(l=0;l<filterWidth;l++)
				{
					smplPos[tempCountWnd] = tempWndPos;
					tempCountWnd++;
					tempWndPos++;
				}

				tempWndPos=tempWndPos+width-filterWidth;				
			}
			//���
			wndSigma = 0.000;
			for(k=0;k<filterBufLenght;k++)
			{
				wndSigma+= pOriImg[ smplPos[k] ]*filter[k];
			}
			pFilterImg[cntPtPos] = BYTE(wndSigma);

			cntPtPos++;
			
			
		}
	
	}
	//�ͷ��ڴ�
	if (smplPos!=NULL)	{	delete []smplPos;	smplPos = NULL;	}

}


BOOL CCVClass::SIFTMatch(	BYTE*& pLImg,int lWidth,int lHeight,int lClrNum,
							BYTE*& pRImg,int rWidth,int rHeight,int rClrNum,
							int& matchPtNum,float*& lx,float*& ly,float*& rx,float*& ry,
							CString outPutDir,
							CString lSiftPtFile,
							CString rSiftPtFile,
							float seedX,float seedY,float searchX,float searchY,int maxNum)
{
	int lImgBufLength = lWidth*lHeight;
	int rImgBufLength = rWidth*rHeight;
	//��Ƭת�ڰ�Ӱ��
	BYTE* pLGrey = NULL;
	if (lClrNum==3)//��ɫ 
	{
		pLGrey =  new BYTE[lImgBufLength];
		TransClrToGreyImg(pLImg,lWidth,lHeight,pLGrey);	
		delete []pLImg;
		pLImg = NULL;
	}		
	else
	{
		pLGrey = pLImg;
		pLImg  = NULL;
	}


	//��Ƭת�ڰ�Ӱ��
	BYTE* pRGrey = NULL;
	if (rClrNum==3)//��ɫ 
	{
		pRGrey =  new BYTE[rImgBufLength];
		TransClrToGreyImg(pRImg,rWidth,rHeight,pRGrey);	
		delete []pRImg;
		pRImg = NULL;
	}		
	else
	{
		pRGrey = pRImg;
		pRImg  = NULL;
	}

	//��ȡ��ƬSIFT������
	int lSiftPtsNum = 0;
	SIFTPoint* pLSiftPts = NULL;
	
	outPutDir = "NULL";
	CString lImgTempDir;
	lImgTempDir.Format("%s\\LImg",outPutDir);
	if(outPutDir != "NULL")
	_mkdir(lImgTempDir);
	int dffThreshold = 2;		///��С20����0��2��ԭʼӰ��8-10   yjzhang
	ExtractSIFTPoints(pLGrey,lWidth,lHeight,dffThreshold,lSiftPtsNum,pLSiftPts,lImgTempDir,FALSE);
	//����������ļ�
	if (lSiftPtFile!="NULL")
	{
		OutputSiftPts(pLSiftPts,lSiftPtsNum,lSiftPtFile);
	}
	//�ͷ��ڴ�
	if (lClrNum==3&&pLGrey!=NULL)
	{
		delete []pLGrey; pLGrey = NULL;
	}

	//��ȡ��ƬSIFT������
	int rSiftPtsNum = 0;
	SIFTPoint* pRSiftPts = NULL;


	CString rImgTempDir;
	rImgTempDir.Format("%s\\RImg",outPutDir);
	if(outPutDir != "NULL")
	_mkdir(rImgTempDir);

	ExtractSIFTPoints(pRGrey,rWidth,rHeight,dffThreshold,rSiftPtsNum,pRSiftPts,rImgTempDir,FALSE);
	if (rSiftPtFile!="NULL")
	{
		OutputSiftPts(pRSiftPts,rSiftPtsNum,rSiftPtFile);
	}
	if (rClrNum==3&&pRGrey!=NULL)
	{
		delete []pRGrey; pRGrey = NULL;
	}
	
	//////////////////////�������//////////////////////////////////////////////
	matchPtNum = lSiftPtsNum>rSiftPtsNum?lSiftPtsNum:rSiftPtsNum;
	lx = new float[matchPtNum];
	ly = new float[matchPtNum];
	rx = new float[matchPtNum];
	ry = new float[matchPtNum];

//	CompareSiftPoints(pLSiftPts,lSiftPtsNum,pRSiftPts,rSiftPtsNum,lx,ly,rx,ry,matchPtNum);
	CompareSiftPoints(pLSiftPts,lSiftPtsNum,pRSiftPts,rSiftPtsNum,lx,ly,rx,ry,matchPtNum,
		lWidth, lHeight, rWidth, rHeight,seedX, seedY, searchX, searchY,maxNum);
//	matchPtNum = lSiftPtsNum>rSiftPtsNum?lSiftPtsNum:rSiftPtsNum;
//	lx = new float[matchPtNum];
//	ly = new float[matchPtNum];
//	rx = new float[matchPtNum];
//	ry = new float[matchPtNum];
//	int i=0;
//	for(i=0;i<matchPtNum;i++)
//	{
//		if (i>=lSiftPtsNum)
//		{
//			lx[i] = 0.000;
//			ly[i] = 0.000;
//		}
//		else
//		{
//			lx[i] = pLSiftPts[i].imgX;
//			ly[i] = pLSiftPts[i].imgY;
//		}
//		if (i>=rSiftPtsNum)
//		{
//			rx[i] = 0.000;
//			ry[i] = 0.000;
//		}
//		else
//		{
//			rx[i] = pRSiftPts[i].imgX;
//			ry[i] = pRSiftPts[i].imgY;
//		}
//
//	}
	
	//////////////////////////////////////////////////////////////////////////
	





	if (pLSiftPts!=NULL)
	{
		delete []pLSiftPts; pLSiftPts = NULL;
	}
	if (pRSiftPts!=NULL)
	{
		delete []pRSiftPts; pRSiftPts = NULL;
	}

	
	return TRUE;
}
#include"ParallelProcessing.h"

BOOL CCVClass::SIFTMatchByGrid(	BYTE*& pLImg,int lWidth,int lHeight,int lClrNum,
									BYTE*& pRImg,int rWidth,int rHeight,int rClrNum,
									int gridRows,int gridCols,
									int& matchPtNum,float*& lx,float*& ly,float*& rx,float*& ry,
									CString outPutDir,
									CString lSiftPtFile,
									CString rSiftPtFile,
									float seedX,float seedY,float searchX,float searchY,int maxNum,int dffThreshold, int CompareStrategy)
{
	int lImgBufLength = lWidth*lHeight;
	int rImgBufLength = rWidth*rHeight;
	//��Ƭת�ڰ�Ӱ��
	BYTE* pLGrey = NULL;
	if (lClrNum==3)//��ɫ 
	{
		pLGrey =  new BYTE[lImgBufLength];
		TransClrToGreyImg(pLImg,lWidth,lHeight,pLGrey);	
		delete []pLImg;
		pLImg = NULL;
	}		
	else
	{
		pLGrey = pLImg;
		pLImg  = NULL;
	}


	//��Ƭת�ڰ�Ӱ��
	BYTE* pRGrey = NULL;
	if (rClrNum==3)//��ɫ 
	{
		pRGrey =  new BYTE[rImgBufLength];
		TransClrToGreyImg(pRImg,rWidth,rHeight,pRGrey);	
		delete []pRImg;
		pRImg = NULL;
	}		
	else
	{
		pRGrey = pRImg;
		pRImg  = NULL;
	}

	//��ȡ��ƬSIFT������
	int lSiftPtsNum = 0;
	SIFTPoint* pLSiftPts = NULL;
	
	outPutDir = "NULL";
	CString lImgTempDir;
	lImgTempDir.Format("%s\\LImg",outPutDir);
	_mkdir(lImgTempDir);
//	int dffThreshold = 3;
	ExtractSIFTPointsByGrid(pLGrey,lWidth,lHeight,dffThreshold,gridRows,gridCols,lSiftPtsNum,pLSiftPts,lImgTempDir,FALSE);
	//����������ļ�
	if (lSiftPtFile!="NULL")
	{
		OutputSiftPts(pLSiftPts,lSiftPtsNum,lSiftPtFile);
	}
	//�ͷ��ڴ�
	if (lClrNum==3&&pLGrey!=NULL)
	{
		delete []pLGrey; pLGrey = NULL;
	}

	//��ȡ��ƬSIFT������
	int rSiftPtsNum = 0;
	SIFTPoint* pRSiftPts = NULL;

	CString rImgTempDir;
	rImgTempDir.Format("%s\\RImg",outPutDir);
	_mkdir(rImgTempDir);

	ExtractSIFTPoints(pRGrey,rWidth,rHeight,dffThreshold,rSiftPtsNum,pRSiftPts,rImgTempDir,FALSE);
	if (rSiftPtFile!="NULL")
	{
		OutputSiftPts(pRSiftPts,rSiftPtsNum,rSiftPtFile);
	}
	if (rClrNum==3&&pRGrey!=NULL)
	{
		delete []pRGrey; pRGrey = NULL;
	}
	
	//////////////////////�������//////////////////////////////////////////////
	matchPtNum = lSiftPtsNum>rSiftPtsNum?lSiftPtsNum:rSiftPtsNum;
	lx = new float[matchPtNum];
	ly = new float[matchPtNum];
	rx = new float[matchPtNum];
	ry = new float[matchPtNum];
	
	CParallelProcessing pParallelProcess;
	pParallelProcess.CompareSiftPoints_MultiThread(pLSiftPts,lSiftPtsNum,pRSiftPts,rSiftPtsNum,lx,ly,rx,ry,matchPtNum,
		lWidth, lHeight, rWidth, rHeight,seedX, seedY, searchX, searchY,maxNum,CompareStrategy);

//	CompareSiftPoints(pLSiftPts,lSiftPtsNum,pRSiftPts,rSiftPtsNum,lx,ly,rx,ry,matchPtNum,
//		lWidth, lHeight, rWidth, rHeight,seedX, seedY, searchX, searchY,maxNum, CompareStrategy);

	if (pLSiftPts!=NULL)
	{
		delete []pLSiftPts; pLSiftPts = NULL;
	}
	if (pRSiftPts!=NULL)
	{
		delete []pRSiftPts; pRSiftPts = NULL;
	}

	
	return TRUE;
}

void CCVClass::TransClrToGreyImg(BYTE *pClrImg, int width, int height, BYTE *&pGreyImg)
{
	//��ɫӰ��ָ��λ��
	int clrPos=0;
	//�ڰ�Ӱ��ָ��λ��
	int grayPos=0;	
	int i,j;
	for(i=0;i<height;i++)
	{
		for(j=0;j<width;j++)
		{
			pGreyImg[grayPos]  = BYTE ( (pClrImg[clrPos] + pClrImg[clrPos+1] + pClrImg[clrPos+2])/3.00 );
			grayPos+=1;
			clrPos +=3;
		}
	}
}


void CCVClass::ExtractSIFTPoints(	BYTE*& pGrey,int imgWidth,int imgHeight,int dffThreshold,int& ptNum,SIFTPoint*& pSiftPts,	
									CString outPutDir,BOOL bZoomOutImage)
{
//	bZoomOutImage = TRUE;//�Ŵ�Ӱ��
	BOOL bDeleteEdge = FALSE;
//	bDeleteEdge= TRUE;//ȥ��ԵЧӦ
	
	#define outPutTempImage

	outPutDir = "NULL";
	if (outPutDir!="NULL")
	{
		_mkdir(outPutDir);
	}
	
	CString svFilePath;

	//��ԭʼӰ��Ŵ�һ��!!!
	BYTE* pZoomImg = NULL;
	int zoomImgWidth,zoomImgHeight;

	if (bZoomOutImage==TRUE) 
	{
		zoomImgWidth	= imgWidth*2;
		zoomImgHeight	= imgHeight*2;
		pZoomImg = new BYTE[zoomImgWidth*zoomImgHeight];
		memset(pZoomImg,0,sizeof(BYTE)*zoomImgWidth*zoomImgHeight);
		ZoomImage(pGrey,imgWidth,imgHeight,pZoomImg,zoomImgWidth,zoomImgHeight);
		delete []pGrey;
		pGrey = NULL;
	}
	else
	{
		zoomImgWidth	= imgWidth;
		zoomImgHeight	= imgHeight;
		pZoomImg		= pGrey;
		pGrey           = NULL;
	}




/*	
	xImgAcs.m_pBuf	= pZoomImg;
	xImgAcs.m_Width	= zoomImgWidth;
	xImgAcs.m_Height= zoomImgHeight;
	xImgAcs.Save2File("c:\\test.tif");
	xImgAcs.m_pBuf = NULL;
*/

//	int s = 3;//DOG�߶ȿռ���(����)
//	int scaleTime = s+3;//�����߶�(��˹Ӱ�����)

	int scaleNum = 2;//�����߿ռ����(ֻ�����ξ��!!!)
	//Paper Test===========================================================================
//	int scaleNum = 6;//�����߿ռ����(ֻ�����ξ��!!!)

	BYTE** ppScaleImg = new BYTE*[scaleNum];//��˹�����Ӱ��
	int i,j,k,l,m,n;
	int imgBufLength = imgWidth*imgHeight;
	float guassDetta,curGuassDetta;

	guassDetta =1.6f;// pow(2.00,1.000/s);//1.6f;//pow(2.00,1.000/s);
	//Paper Test===========================================================================
//	guassDetta =3.6f;// pow(2.00,1.000/s);//1.6f;//pow(2.00,1.000/s);

	float constanK;//����k
	constanK = float( pow(2.00,1.000/scaleNum) );//float( pow(2.00,1.000/s) );
	int guassWndSize = 3;//��˹���ڳߴ�

	//Paper Test===========================================================================
//	int guassWndSize = 5;//��˹���ڳߴ�

	float *gaussTemp = new float[guassWndSize*guassWndSize];//��˹ģ��
	int tempPos = 0;
	int pryLevel = 3;//���ý������ļ���(2�ı����ݼ�)
	int minSize  = 80;//������Ӱ����С�ߴ�
	int imgMinSize = zoomImgWidth<zoomImgHeight?zoomImgWidth:zoomImgHeight;
	double pryZoomRatio = imgMinSize*1.000/minSize;
	pryLevel = int( log(pryZoomRatio)/log(2.00) );
	pryLevel+=1;//�ײ�ΪԭʼӰ��(���Ŵ�������Ӱ��)

//	int doubleSigmaIndex = 3;//��˹���sigmaֵΪ2��sigma�Ĳ�����

	//��ǰ�����߿ռ����
	BYTE* pCurBottomImg = NULL;
	pCurBottomImg		= pZoomImg;
	pZoomImg            = NULL;
	int curImgWidth		= zoomImgWidth;
	int curImgHeight	= zoomImgHeight;
	int curImgBufLength = curImgWidth*curImgHeight;
	
	BYTE* tempSwap;	
	int curSiftPtMemSize = 8000;//��ǰ�������ڴ���
	int memAddStep       = 1000;//�ڴ����Ӳ���

	
	//ȥ��ԵЧӦ����
	double rConstant = 10.000;
	double rThreshold = (rConstant+1)*(rConstant+1)/rConstant;
	double    traceH;
	double    detH;
	double    ratioTraceToDet;

	//DOG�ݶ�
	double Dxx,Dyy,Dxy,doubleD_x_y;

	//�����ֵ
	//Paper Test===========================================================================
   // dffThreshold = 4.00;

	
	if (pSiftPts!=NULL)
	{
		delete []pSiftPts;	pSiftPts = NULL;
	}
	ptNum				 = 0;
	pSiftPts	= new SIFTPoint[curSiftPtMemSize];
	double  curImgResolve = 1.000;//��ǰӰ��ֱ���;
	//PaperTest
	pryLevel = 1;
//	pryLevel = 1;
	for(i=0;i<pryLevel;i++)
	{	
		if (bZoomOutImage==TRUE)
		{
			curImgResolve = powl(2,i-1);
		}

		else
		{
			curImgResolve = powl(2,i) ;
		}
		
		if (i>0)
		{
			//�ز����ϲ��������ײ�����߿ռ��Ӱ����Ϊ��ǰ�������ĵײ�Ӱ��
			tempSwap = new BYTE[int(curImgWidth/2)*int(curImgHeight/2)];
			DownSampleImageBy2(pCurBottomImg,curImgWidth,curImgHeight,tempSwap);
			//DownSampleImageBy2(pCurBottomImg,curImgWidth,curImgHeight,tempSwap);
			//ZoomImage(pCurBottomImg,curImgWidth,curImgHeight,tempSwap,int(curImgWidth/2),int(curImgHeight/2) );
			delete []pCurBottomImg;
			pCurBottomImg = tempSwap;
			tempSwap      = NULL;
			curImgWidth  = int(curImgWidth/2);
			curImgHeight = int(curImgHeight/2);

		}
		curImgBufLength = curImgWidth*curImgHeight;

		for(j=0;j<scaleNum;j++)
		{
			ppScaleImg[j] = new BYTE[curImgBufLength];
			memcpy(ppScaleImg[j],pCurBottomImg,sizeof(BYTE)*curImgBufLength);
		}
		
		
		for(j=0;j<scaleNum;j++)
		{
			curGuassDetta = float( guassDetta*powl(2,j) );
			GenGaussTemplate(gaussTemp,guassWndSize,guassWndSize,curGuassDetta,TRUE);
			if (j==0)
			{
				ImageFiltering(pCurBottomImg,curImgWidth,curImgHeight,gaussTemp,guassWndSize,guassWndSize,ppScaleImg[j]);
			}
			else
			{
				ImageFiltering(ppScaleImg[j-1],curImgWidth,curImgHeight,gaussTemp,guassWndSize,guassWndSize,ppScaleImg[j]);
			}
			//������pCurBottomImg�����ͷ�
/*
			if (pCurBottomImg!=NULL)
			{
				delete []pCurBottomImg ;	pCurBottomImg	=	NULL;
			}*/

			
		}

		//����ÿ����ݶ�
		GradsPoint* pGradsPts  = new GradsPoint[curImgBufLength];//ͼ����ÿ���ݶ�
		CalImgGrads(pCurBottomImg,curImgWidth,curImgHeight,pGradsPts);
		
		//���ɲ��Ӱ��
#ifdef outPutTempImage

		BYTE** ppDffImg = new BYTE*[scaleNum-1];
		for(j=0;j<scaleNum-1;j++)
		{
			ppDffImg[j] = new BYTE[curImgBufLength];
			memset(ppDffImg[j],0,sizeof(BYTE)*curImgBufLength);
			
		}

		//���
		for(j=0;j<scaleNum-1;j++)
		{
			tempPos = 0;
			BYTE maxGrey,minGrey;
			maxGrey = 0;
			minGrey = 255;
			for(k=0;k<curImgBufLength;k++)
			{
				ppDffImg[j][tempPos] = BYTE( abs( ppScaleImg[j][tempPos] - ppScaleImg[j+1][tempPos]) );
				maxGrey = maxGrey>ppDffImg[j][tempPos]?maxGrey:ppDffImg[j][tempPos];
				minGrey = minGrey<ppDffImg[j][tempPos]?minGrey:ppDffImg[j][tempPos];
				tempPos++;
			}
			//��һ��
			double nrlK,nrlD;
			nrlD = minGrey;
			nrlK = 255.000/(maxGrey - minGrey);

			tempPos = 0;
			for(k=0;k<curImgBufLength;k++)
			{
				tempPos++;
			}		
			
			
		}
		//�ͷ��ڴ�
		for(j=0;j<scaleNum-1;j++)
		{
			if (ppDffImg[j]!=NULL)		{	delete []ppDffImg[j];		ppDffImg[j]		=	NULL;	}	
		}
		if (ppDffImg!=NULL)				{	delete []ppDffImg;			ppDffImg		=	NULL;	}

#endif

		//Ӱ����ֵ
		int** pdff		= new int*[scaleNum-1];
		for(j=0;j<scaleNum-1;j++)
		{
			pdff[j] = new int[curImgBufLength];
			memset(pdff[j],0,sizeof(int)*curImgBufLength);
			
		}
		//���
		for(j=0;j<scaleNum-1;j++)
		{
			tempPos = 0;

			int maxGrey,minGrey;
			maxGrey = -255;
			minGrey = 255;
			
			for(k=0;k<curImgBufLength;k++)
			{
				//��ֺ��Ƿ�Ҫȡ����ֵ?????
			//	pdff[j][tempPos] =  abs(ppScaleImg[j][tempPos] - ppScaleImg[j+1][tempPos]) ;
				pdff[j][tempPos] =    ppScaleImg[j][tempPos] - ppScaleImg[j+1][tempPos];
				maxGrey = maxGrey>pdff[j][tempPos]?maxGrey:pdff[j][tempPos];
				minGrey = minGrey<pdff[j][tempPos]?minGrey:pdff[j][tempPos];
				tempPos++;
			}

			
		}

		//�ͷ��ڴ�ײ�Ӱ��,�ϲ�Ӱ����Ϊ�¼��������ĳ�ʼӰ��
		for(j=0;j<scaleNum;j++)
		{
			if (ppScaleImg[j]!=NULL)	{	delete []ppScaleImg[j];		ppScaleImg[j]	=	NULL;	}
		}
//		pCurBottomImg = ppScaleImg[scaleNum-1]; 
		
		//��ֵ̽��
		for(j=0;j<scaleNum-1;j++)
		{
			int imgBufPos = 0;//�����߿ռ�Ӱ��ָ��λ��
			int wndBufPos = 0;//�����򴰿�Ӱ��ָ��λ��
		//	int maxValue,minValue;
			int cntValue;
			int neighbourValue[9];//��ǰ�������ڳ߶ȿռ�(������ǰ�ռ�)����������ֵ
			int wndPos = 0;
			BOOL bMin = FALSE;
			BOOL bMax = FALSE;
			for(k=0;k<curImgHeight;k++)
			{
				for(l=0;l<curImgWidth;l++)
				{
					if (k<16||k>=curImgHeight-16||l<16||l>=curImgWidth-16)//16Ϊ�ֲ����������ڴ�С
					{
						imgBufPos++;
						continue;
					}
					cntValue	= pdff[j][imgBufPos];

					wndPos   = 0;
					wndBufPos= imgBufPos-curImgWidth-1;//�������½���DOGӰ���е�λ��
					for(m=0;m<3;m++)
					{
						for(n=0;n<3;n++)
						{
							neighbourValue[wndPos]   = pdff[j][wndBufPos];	
							wndPos+=1;
							wndBufPos+=1;
						}
						wndBufPos = wndBufPos+curImgWidth-3;
					}
					//�Ƚ�
					bMin     = bMax     = TRUE;
					for(m=0;m<9;m++)
					{
						//�������ز��Ƚ�
						if (m==4)
						{
							continue;
						}
						if (cntValue<=neighbourValue[m])
						{
							bMax = FALSE;
						}
						if (cntValue>=neighbourValue[m]) 
						{
							bMin = FALSE;
						}
					}
					if ( (bMax==TRUE||bMin==TRUE) && abs(cntValue)>=dffThreshold)//��ֵ��
					{
						
						//ȥ��ԵЧӦ(Hessian����)
						/*
						Dxx = D[x+1,y] + D[x-1,y] - 2*D[x,y];
						Dyy = D[x,y+1] + D[x,y-1] - 2*D[x,y];
						Dxy = 0.25*( (D[x+1,y+1] - D[x+1,y-1]) - (D[x-1,y+1] - D[x-1,y-1]) );
						
						traceH  = Dxx + Dyy;
						detH    = Dxx*Dyy - Dxy*Dxy;
						*/

						if (bDeleteEdge==TRUE)
						{
							doubleD_x_y = 2.0*pdff[j][imgBufPos];

							Dxx			= pdff[j][imgBufPos+1]				+ pdff[j][imgBufPos-1]			- doubleD_x_y;
							Dyy			= pdff[j][imgBufPos+curImgWidth]	    + pdff[j][imgBufPos-curImgWidth]  - doubleD_x_y;
								
							Dxy = 0.25*( (	pdff[j][imgBufPos+1+curImgWidth] - pdff[j][imgBufPos+1-curImgWidth]) 
										-(	pdff[j][imgBufPos-1+curImgWidth] - pdff[j][imgBufPos-1-curImgWidth]) 
										);
							traceH  = Dxx + Dyy;
							detH    = Dxx*Dyy - Dxy*Dxy;
							ratioTraceToDet = traceH*traceH/detH;
						}


						if (ratioTraceToDet<rThreshold||bDeleteEdge==FALSE)
						{
						//	int mainOriNum = 0;
						//	float mainOri[36];

							SIFTPoint tempSiftPt;
							tempSiftPt.imgX = float(l);
							tempSiftPt.imgY = float(k);
							
							//����sift������
							CalSiftFeaturePtMainOri(pGradsPts,curImgWidth,curImgHeight,tempSiftPt);
							for(m=0;m<tempSiftPt.histgramOf36.mainOriNum;m++)
							{
								pSiftPts[ptNum] = tempSiftPt;
								pSiftPts[ptNum].mainGradsMgnt = tempSiftPt.histgramOf36.histogram[ tempSiftPt.histgramOf36.mainOriIndex[m] ];
								pSiftPts[ptNum].mainGradsOri  = float(tempSiftPt.histgramOf36.mainOriIndex[m]*RADIAN_OF_10DEGREE);
								//����������������!!!
								DescriptSIFTPoint(pGradsPts,curImgWidth,curImgHeight,pSiftPts[ptNum]);
								pSiftPts[ptNum].imgX = float(curImgResolve*l);
								pSiftPts[ptNum].imgY = float(curImgResolve*k);

								ptNum++;
								//��̬�����ڴ�
								if (ptNum==curSiftPtMemSize)
								{
									curSiftPtMemSize+=memAddStep;
									SIFTPoint* pSwap = new SIFTPoint[curSiftPtMemSize];
									memcpy(pSwap,pSiftPts,sizeof(SIFTPoint)*ptNum);
									delete []pSiftPts;
									pSiftPts = pSwap;
									pSwap    = NULL;
								}
							}


			
						}
						

					}

					imgBufPos++;
				}
			}
		}
		


		

		//�ͷ��ڴ�
		for(j=0;j<scaleNum-1;j++)
		{
			if (pdff[j]!=NULL)			{	delete []pdff[j];		pdff[j]			=	NULL;	}	
		}
		if (pdff!=NULL)					{	delete []pdff;			pdff			=	NULL;	}
		if (pGradsPts!=NULL)			{   delete []pGradsPts;		pGradsPts		=	NULL;   }
//		if (pCurBottomImg!=NULL)        {	delete []pCurBottomImg; pCurBottomImg	=	NULL;	}

	
		
	}

	//�ͷŵײ������
	if (pCurBottomImg!=NULL)			{	delete[]pCurBottomImg;		pCurBottomImg = NULL;		}
	if (ppScaleImg!=NULL)				{	delete []ppScaleImg;		ppScaleImg		=	NULL;	}
	if (gaussTemp!=NULL)				{	delete []gaussTemp;			gaussTemp		=	NULL;	}
	//�������ӱ����߹�һ��
	NormalizeSiftPoints(pSiftPts,ptNum);



}



void CCVClass::ExtractSIFTPointsByGrid(	BYTE* pGrey,int imgWidth,int imgHeight,
										int dffThreshold,int GridRows,int GridCols,
										int& ptNum,SIFTPoint*& pSiftPts,	
										CString outPutDir,BOOL bZoomOutImage)
{
//	bZoomOutImage = TRUE;//�Ŵ�Ӱ��
	BOOL bDeleteEdge = FALSE;
	bDeleteEdge= TRUE;//ȥ��ԵЧӦ
	
#define outPutTempImage

	outPutDir = "NULL";
	if (outPutDir!="NULL")
	{
		_mkdir(outPutDir);
	}
	CString svFilePath;

	//��ԭʼӰ��Ŵ�һ��!!!
	BYTE* pZoomImg = NULL;
	int zoomImgWidth,zoomImgHeight;

	if (bZoomOutImage==TRUE) 
	{
		zoomImgWidth	= imgWidth*2;
		zoomImgHeight	= imgHeight*2;
		pZoomImg = new BYTE[zoomImgWidth*zoomImgHeight];
		memset(pZoomImg,0,sizeof(BYTE)*zoomImgWidth*zoomImgHeight);
		ZoomImage(pGrey,imgWidth,imgHeight,pZoomImg,zoomImgWidth,zoomImgHeight);
// 		delete []pGrey;
// 		pGrey = NULL;
	}
	else
	{
		zoomImgWidth	= imgWidth;
		zoomImgHeight	= imgHeight;
		pZoomImg		= pGrey;
/*		pGrey           = NULL;*/
	}




/*	
	xImgAcs.m_pBuf	= pZoomImg;
	xImgAcs.m_Width	= zoomImgWidth;
	xImgAcs.m_Height= zoomImgHeight;
	xImgAcs.Save2File("c:\\test.tif");
	xImgAcs.m_pBuf = NULL;
*/

//	int s = 3;//DOG�߶ȿռ���(����)
//	int scaleTime = s+3;//�����߶�(��˹Ӱ�����)
	int scaleNum = 2;//�����߿ռ����(ֻ�����ξ��!!!)
	BYTE** ppScaleImg = new BYTE*[scaleNum];//��˹�����Ӱ��
	int i,j,k,l,m,n;
	int imgBufLength = imgWidth*imgHeight;
	float guassDetta,curGuassDetta;
	guassDetta =1.6f;// pow(2.00,1.000/s);//1.6f;//pow(2.00,1.000/s);
	float constanK;//����k
	constanK = float( pow(2.00,1.000/scaleNum) );//float( pow(2.00,1.000/s) );
	int guassWndSize = 3;//��˹���ڳߴ�
	float *gaussTemp = new float[guassWndSize*guassWndSize];//��˹ģ��
	int tempPos = 0;
	int pryLevel = 1;//���ý������ļ���(2�ı����ݼ�)
	int minSize  = 80;//������Ӱ����С�ߴ�
	int imgMinSize = zoomImgWidth<zoomImgHeight?zoomImgWidth:zoomImgHeight;
	double pryZoomRatio = imgMinSize*1.000/minSize;
	pryLevel = int( log(pryZoomRatio)/log(2.00) );
	pryLevel+=1;//�ײ�ΪԭʼӰ��(���Ŵ�������Ӱ��)

//	int doubleSigmaIndex = 3;//��˹���sigmaֵΪ2��sigma�Ĳ�����

	//��ǰ�����߿ռ����
	BYTE* pCurBottomImg = NULL;
	pCurBottomImg		= pZoomImg;
	pZoomImg            = NULL;
	int curImgWidth		= zoomImgWidth;
	int curImgHeight	= zoomImgHeight;
	int curImgBufLength = curImgWidth*curImgHeight;
	
	BYTE* tempSwap;	
	int curSiftPtMemSize = 8000;//��ǰ�������ڴ���
	int memAddStep       = 1000;//�ڴ����Ӳ���

	
	//ȥ��ԵЧӦ����
	double rConstant = 10.000;
	double rThreshold = (rConstant+1)*(rConstant+1)/rConstant;
	double    traceH;
	double    detH;
	double    ratioTraceToDet;

	//DOG�ݶ�
	double Dxx,Dyy,Dxy,doubleD_x_y;

	//�����ֵ
//	double dffThreshold = 1.00;

	
	if (pSiftPts!=NULL)
	{
		delete []pSiftPts;	pSiftPts = NULL;
	}
	ptNum				 = 0;
	pSiftPts	= new SIFTPoint[curSiftPtMemSize];
	double  curImgResolve = 1.000;//��ǰӰ��ֱ���;
//	pryLevel = 2;
	pryLevel = 1;
	for(i=0;i<pryLevel;i++)
	{	
		if (bZoomOutImage==TRUE)
		{
			curImgResolve = powl(2,i-1);
		}

		else
		{
			curImgResolve = powl(2,i) ;
		}
		
		if (i>0)
		{
			//�ز����ϲ��������ײ�����߿ռ��Ӱ����Ϊ��ǰ�������ĵײ�Ӱ��
			tempSwap = new BYTE[int(curImgWidth/2)*int(curImgHeight/2)];
			DownSampleImageBy2(pCurBottomImg,curImgWidth,curImgHeight,tempSwap);
			//DownSampleImageBy2(pCurBottomImg,curImgWidth,curImgHeight,tempSwap);
			//ZoomImage(pCurBottomImg,curImgWidth,curImgHeight,tempSwap,int(curImgWidth/2),int(curImgHeight/2) );
			delete []pCurBottomImg;
			pCurBottomImg = tempSwap;
			tempSwap      = NULL;
			curImgWidth  = int(curImgWidth/2);
			curImgHeight = int(curImgHeight/2);

		}
		curImgBufLength = curImgWidth*curImgHeight;

		for(j=0;j<scaleNum;j++)
		{
			ppScaleImg[j] = new BYTE[curImgBufLength];
			memcpy(ppScaleImg[j],pCurBottomImg,sizeof(BYTE)*curImgBufLength);
		}
		
		
		for(j=0;j<scaleNum;j++)
		{
			curGuassDetta = float( guassDetta*powl(2,j) );
			GenGaussTemplate(gaussTemp,guassWndSize,guassWndSize,curGuassDetta,TRUE);
			if (j==0)
			{
				ImageFiltering(pCurBottomImg,curImgWidth,curImgHeight,gaussTemp,guassWndSize,guassWndSize,ppScaleImg[j]);
			}
			else
			{
				ImageFiltering(ppScaleImg[j-1],curImgWidth,curImgHeight,gaussTemp,guassWndSize,guassWndSize,ppScaleImg[j]);
			}
			//������pCurBottomImg�����ͷ�
/*
			if (pCurBottomImg!=NULL)
			{
				delete []pCurBottomImg ;	pCurBottomImg	=	NULL;
			}*/

			
		}

		//����ÿ����ݶ�
		GradsPoint* pGradsPts  = new GradsPoint[curImgBufLength];//ͼ����ÿ���ݶ�
		CalImgGrads(pCurBottomImg,curImgWidth,curImgHeight,pGradsPts);
		
		//���ɲ��Ӱ��
#ifdef outPutTempImage

		BYTE** ppDffImg = new BYTE*[scaleNum-1];
		for(j=0;j<scaleNum-1;j++)
		{
			ppDffImg[j] = new BYTE[curImgBufLength];
			memset(ppDffImg[j],0,sizeof(BYTE)*curImgBufLength);
			
		}

		//���
		for(j=0;j<scaleNum-1;j++)
		{
			tempPos = 0;
			BYTE maxGrey,minGrey;
			maxGrey = 0;
			minGrey = 255;
			for(k=0;k<curImgBufLength;k++)
			{
				ppDffImg[j][tempPos] = BYTE( abs( ppScaleImg[j][tempPos] - ppScaleImg[j+1][tempPos]) );
				maxGrey = maxGrey>ppDffImg[j][tempPos]?maxGrey:ppDffImg[j][tempPos];
				minGrey = minGrey<ppDffImg[j][tempPos]?minGrey:ppDffImg[j][tempPos];
				tempPos++;
			}
			//��һ��
			double nrlK,nrlD;
			nrlD = minGrey;
			nrlK = 255.000/(maxGrey - minGrey);

			tempPos = 0;
			for(k=0;k<curImgBufLength;k++)
			{
				tempPos++;
			}		
			
			
		}
		//�ͷ��ڴ�
		for(j=0;j<scaleNum-1;j++)
		{
			if (ppDffImg[j]!=NULL)		{	delete []ppDffImg[j];		ppDffImg[j]		=	NULL;	}	
		}
		if (ppDffImg!=NULL)				{	delete []ppDffImg;			ppDffImg		=	NULL;	}

#endif

		//Ӱ����ֵ
		int** pdff		= new int*[scaleNum-1];
		for(j=0;j<scaleNum-1;j++)
		{
			pdff[j] = new int[curImgBufLength];
			memset(pdff[j],0,sizeof(int)*curImgBufLength);
			
		}
		//���
		for(j=0;j<scaleNum-1;j++)
		{
			tempPos = 0;

			int maxGrey,minGrey;
			maxGrey = -255;
			minGrey = 255;
			
			for(k=0;k<curImgBufLength;k++)
			{
				//��ֺ��Ƿ�Ҫȡ����ֵ?????
			//	pdff[j][tempPos] =  abs(ppScaleImg[j][tempPos] - ppScaleImg[j+1][tempPos]) ;
				pdff[j][tempPos] =    ppScaleImg[j][tempPos] - ppScaleImg[j+1][tempPos];
				maxGrey = maxGrey>pdff[j][tempPos]?maxGrey:pdff[j][tempPos];
				minGrey = minGrey<pdff[j][tempPos]?minGrey:pdff[j][tempPos];
				tempPos++;
			}

			
		}

		//�ͷ��ڴ�ײ�Ӱ��,�ϲ�Ӱ����Ϊ�¼��������ĳ�ʼӰ��
		for(j=0;j<scaleNum;j++)
		{
			if (ppScaleImg[j]!=NULL)	{	delete []ppScaleImg[j];		ppScaleImg[j]	=	NULL;	}
		}
//		pCurBottomImg = ppScaleImg[scaleNum-1]; 
		
		//��ֵ̽��
		for(j=0;j<scaleNum-1;j++)
		{
			int imgBufPos = 0;//�����߿ռ�Ӱ��ָ��λ��
			int wndBufPos = 0;//�����򴰿�Ӱ��ָ��λ��
		//	int maxValue,minValue;
			int cntValue;
			int neighbourValue[9];//��ǰ�������ڳ߶ȿռ�(������ǰ�ռ�)����������ֵ
			int wndPos = 0;
			BOOL bMin = FALSE;
			BOOL bMax = FALSE;
			for(k=0;k<curImgHeight;k++)
			{
				for(l=0;l<curImgWidth;l++)
				{
					if (k<16||k>=curImgHeight-16||l<16||l>=curImgWidth-16)//16Ϊ�ֲ����������ڴ�С
					{
						imgBufPos++;
						continue;
					}
					cntValue	= pdff[j][imgBufPos];

					wndPos   = 0;
					wndBufPos= imgBufPos-curImgWidth-1;//�������½���DOGӰ���е�λ��
					for(m=0;m<3;m++)
					{
						for(n=0;n<3;n++)
						{
							neighbourValue[wndPos]   = pdff[j][wndBufPos];	
							wndPos+=1;
							wndBufPos+=1;
						}
						wndBufPos = wndBufPos+curImgWidth-3;
					}
					//�Ƚ�
					bMin     = bMax     = TRUE;
					for(m=0;m<9;m++)
					{
						//�������ز��Ƚ�
						if (m==4)
						{
							continue;
						}
						if (cntValue<=neighbourValue[m])
						{
							bMax = FALSE;
						}
						if (cntValue>=neighbourValue[m]) 
						{
							bMin = FALSE;
						}
					}
					if ( (bMax==TRUE||bMin==TRUE) && abs(cntValue)>=dffThreshold)//��ֵ��
					{
						
						//ȥ��ԵЧӦ(Hessian����)
						/*
						Dxx = D[x+1,y] + D[x-1,y] - 2*D[x,y];
						Dyy = D[x,y+1] + D[x,y-1] - 2*D[x,y];
						Dxy = 0.25*( (D[x+1,y+1] - D[x+1,y-1]) - (D[x-1,y+1] - D[x-1,y-1]) );
						
						traceH  = Dxx + Dyy;
						detH    = Dxx*Dyy - Dxy*Dxy;
						*/

						if (bDeleteEdge==TRUE)
						{
							doubleD_x_y = 2.0*pdff[j][imgBufPos];

							Dxx			= pdff[j][imgBufPos+1]				+ pdff[j][imgBufPos-1]			- doubleD_x_y;
							Dyy			= pdff[j][imgBufPos+curImgWidth]	    + pdff[j][imgBufPos-curImgWidth]  - doubleD_x_y;
								
							Dxy = 0.25*( (	pdff[j][imgBufPos+1+curImgWidth] - pdff[j][imgBufPos+1-curImgWidth]) 
										-(	pdff[j][imgBufPos-1+curImgWidth] - pdff[j][imgBufPos-1-curImgWidth]) 
										);
							traceH  = Dxx + Dyy;
							detH    = Dxx*Dyy - Dxy*Dxy;
							ratioTraceToDet = traceH*traceH/detH;
						}


						if (ratioTraceToDet<rThreshold||bDeleteEdge==FALSE)
						{
						//	int mainOriNum = 0;
						//	float mainOri[36];

							SIFTPoint tempSiftPt;
							tempSiftPt.imgX = float(l);
							tempSiftPt.imgY = float(k);
							
							//����sift������
							CalSiftFeaturePtMainOri(pGradsPts,curImgWidth,curImgHeight,tempSiftPt);
							for(m=0;m<tempSiftPt.histgramOf36.mainOriNum;m++)
							{
								pSiftPts[ptNum] = tempSiftPt;
								pSiftPts[ptNum].mainGradsMgnt = tempSiftPt.histgramOf36.histogram[ tempSiftPt.histgramOf36.mainOriIndex[m] ];
								pSiftPts[ptNum].mainGradsOri  = float(tempSiftPt.histgramOf36.mainOriIndex[m]*RADIAN_OF_10DEGREE);
								//����������������!!!
								DescriptSIFTPoint(pGradsPts,curImgWidth,curImgHeight,pSiftPts[ptNum]);
								pSiftPts[ptNum].imgX = float(curImgResolve*l);
								pSiftPts[ptNum].imgY = float(curImgResolve*k);

								ptNum++;
								//��̬�����ڴ�
								if (ptNum==curSiftPtMemSize)
								{
									curSiftPtMemSize+=memAddStep;
									SIFTPoint* pSwap = new SIFTPoint[curSiftPtMemSize];
									memcpy(pSwap,pSiftPts,sizeof(SIFTPoint)*ptNum);
									delete []pSiftPts;
									pSiftPts = pSwap;
									pSwap    = NULL;
								}
							}


			
						}
						

					}

					imgBufPos++;
				}
			}
		}
		


		

		//�ͷ��ڴ�
		for(j=0;j<scaleNum-1;j++)
		{
			if (pdff[j]!=NULL)			{	delete []pdff[j];		pdff[j]			=	NULL;	}	
		}
		if (pdff!=NULL)					{	delete []pdff;			pdff			=	NULL;	}
		if (pGradsPts!=NULL)			{   delete []pGradsPts;		pGradsPts		=	NULL;   }
//		if (pCurBottomImg!=NULL)        {	delete []pCurBottomImg; pCurBottomImg	=	NULL;	}

	
		
	}

	//�ͷŵײ������
	if (pCurBottomImg!=NULL)			{	delete[]pCurBottomImg;		pCurBottomImg = NULL;		}
	if (ppScaleImg!=NULL)				{	delete []ppScaleImg;		ppScaleImg		=	NULL;	}
	if (gaussTemp!=NULL)				{	delete []gaussTemp;			gaussTemp		=	NULL;	}
	//�������ӱ����߹�һ��
	NormalizeSiftPoints(pSiftPts,ptNum);

	//��������������
	int gridWidth  = imgWidth/GridCols +1;
	int gridHeight = imgHeight/GridRows+1;
	int   gridNumber = GridCols*GridRows;
	SIFTPoint* tempGridSiftPts = new SIFTPoint[gridNumber];
	for(i=0;i<gridNumber;i++)
	{
		tempGridSiftPts[i].mainGradsMgnt = -1.f;
	}
	int rowIndex,colIndex,gridIndex;
	for(i=0;i<ptNum;i++)
	{
		rowIndex = int(pSiftPts[i].imgY/gridHeight);
		colIndex = int(pSiftPts[i].imgX/gridWidth);
		gridIndex= rowIndex*GridCols + colIndex;
		if (pSiftPts[i].mainGradsMgnt>tempGridSiftPts[gridIndex].mainGradsMgnt)
		{
			tempGridSiftPts[gridIndex] = pSiftPts[i];
		}
	}
	int tempValidGridPtNum = 0;
	for(i=0;i<gridNumber;i++)
	{
		if (tempGridSiftPts[i].mainGradsMgnt>0.f)
		{
			tempValidGridPtNum+=1;
		}
	}


	SIFTPoint* tempValidGridSiftPts = new SIFTPoint[tempValidGridPtNum];
	tempValidGridPtNum = 0;
	for(i=0;i<gridNumber;i++)
	{
		if (tempGridSiftPts[i].mainGradsMgnt>0.f)
		{
			tempValidGridSiftPts[tempValidGridPtNum]=tempGridSiftPts[i];
			tempValidGridPtNum+=1;
		}
	}
	delete []tempGridSiftPts;
	
	//ָ��ת��
	SIFTPoint* tempSwapSiftPts;
	tempSwapSiftPts      = pSiftPts;
	pSiftPts             = tempValidGridSiftPts;
	tempValidGridSiftPts = tempSwapSiftPts;

	int        tempSwapPtNum;
	tempSwapPtNum		= ptNum;
	ptNum				= tempValidGridPtNum;
	tempValidGridPtNum  = tempSwapPtNum;

	delete[] tempValidGridSiftPts;
}

void CCVClass::ZoomImage(BYTE *pOriImg, int oriImgWidth, int oriImgHeight, BYTE *pZoomImg, int zoomImgWidth, int zoomImgHeight)
{
	//˫�����ڲ�
/*

	/ *******    bilinear interpolation of gray level  ******** /
	//	z = grid[0]*(1-dx)*(1-dy) + grid[1]*(dx)*(1-dy) + 
	//		grid[2]*(1-dx)*(dy)   + grid[3]*(dx)*(dy);     =====>
	
	//	z = (grid[0]-grid[1]-grid[2]+grid[3])*dx*dy + grid[0]*(1-dx-dy) + grid[1]*dx + grid[2]*dy;	
	
	z = ( *(grid) - *(grid+1) - *(grid+2) + *(grid+3) )*dx*dy + *(grid)*(1-dx-dy) + *(grid+1)*dx + *(grid+2)*dy;
*/
	double ratioX = oriImgWidth*1.000/zoomImgWidth;
	double ratioY = oriImgHeight*1.000/zoomImgHeight;
	double dxStep     = ratioX;
	double dyStep     = ratioY;
	double dx,dy;
	double g[4];
	

	int i,j;
	int		iOriImgPosX  = 0;
	int		iOriImgPosY  = 0;
	double  dOriImgPosX  = 0.000;
	double  dOriImgPosY  = 0.000;
	int     iOriImgPos   = 0;

	int zoomImgPos = 0;

	dOriImgPosY = 0.000;
	for(i=0;i<zoomImgHeight;i++)
	{
		dOriImgPosX = 0.0000;
		for(j=0;j<zoomImgWidth;j++)
		{
			iOriImgPosX = int(dOriImgPosX);
			iOriImgPosY = int(dOriImgPosY);
			iOriImgPos  = iOriImgPosY*oriImgWidth + iOriImgPosX;//���½�ԭ��
			
			g[0]		= pOriImg[iOriImgPos];//���½�
			g[1]		= pOriImg[iOriImgPos+1];//���½�
			g[2]		= pOriImg[iOriImgPos+oriImgWidth];//���Ͻ�
			g[3]		= pOriImg[iOriImgPos+oriImgWidth+1];//���Ͻ�

			dx          = dOriImgPosX - iOriImgPosX;
			dy          = dOriImgPosY - iOriImgPosY;

			pZoomImg[zoomImgPos] = BYTE( (g[0]-g[1]-g[2]+g[3])*dx*dy + g[0]*(1-dx-dy) + g[1]*dx + g[2]*dy );
			zoomImgPos++;
			dOriImgPosX+=dxStep;
			if (dOriImgPosX>=oriImgWidth-1)
			{
				dOriImgPosX = oriImgWidth-2;
			}
		}
		dOriImgPosY+=dyStep;
		if (dOriImgPosY>=oriImgHeight-1)
		{
			dOriImgPosY = oriImgHeight-2;
		}
	}
}

void CCVClass::DownSampleImageBy2(BYTE *pOriImg, int imgWidth, int imgHeight, BYTE *pResmplImg)
{
	int i,j;
	int oriImgPos,resmplImgPos;
	oriImgPos = resmplImgPos = 0;
	//���Ӱ������ż���Ļ�,�����һ�����ز���ȡ
	for (i=0;i<imgHeight;i++)
	{
		if (i%2!=1)
		{
			oriImgPos+=imgWidth;
			continue;
		}
		for(j=0;j<imgWidth;j++)
		{
			if (j%2!=1) 
			{
				oriImgPos+=1;
				continue;
			}
			pResmplImg[resmplImgPos] = pOriImg[oriImgPos];
			oriImgPos+=1;
			resmplImgPos+=1;
			
		}
	}
}





void CCVClass::CalImgGrads(BYTE *&pGrey, int imgWidth, int imgHeight, GradsPoint *&pGradsPts)
{
	int i,j;
	int bufPos = 0;
	int dxx,dyy;
	for(i=0;i<imgHeight;i++)
	{
		for(j=0;j<imgWidth;j++)
		{
			if (i<1||i>=imgHeight-1||j<1||j>=imgWidth-1)
			{
				bufPos+=1;
				continue;
			}
			dxx = pGrey[bufPos+1] - pGrey[bufPos-1];
			dyy = pGrey[bufPos+imgWidth] - pGrey[bufPos-imgWidth];
			pGradsPts[bufPos].magnitude   = float( sqrt((double)dxx*dxx + dyy*dyy) );
			if (dxx==0.000)
			{
				if (dyy>0.000)
				{
					pGradsPts[bufPos].orientation = float(PI/2.000);
				}
				else if (dyy<0.000)
				{
					pGradsPts[bufPos].orientation = float(-PI/2.000);
				}
				else
				{
					pGradsPts[bufPos].orientation = 0.f;			
				}
			}
			else
			{				
				pGradsPts[bufPos].orientation = float(atan2((double)dyy, dxx));
			}
			if (pGradsPts[bufPos].orientation <0.000)
			{
				pGradsPts[bufPos].orientation +=float(PI*2.000);
			}

			bufPos+=1;
			



		}
	}
}

void CCVClass::CalSiftFeaturePtMainOri(	 GradsPoint *&pGradsPts, int imgWidth, int imgHeight,
									     SIFTPoint& siftPt)
{
	//////////////////////////Step 1 ����������������////////////////////////////////////////////////
	
	//ͳ����������ΧftrPtWndSizexftrPtWndSize�������ݶ�ֱ��ͼ(����0,10,20,....350);ÿ10��һ������,��36������
	int ftrPtWndSize = 4;
	int wndBufLength = ftrPtWndSize*ftrPtWndSize;
	int x = int(siftPt.imgX+0.5);
	int y = int(siftPt.imgY+0.5);

	int imgBufPos = (y-ftrPtWndSize/2)*imgWidth + x-ftrPtWndSize/2;//���½ǿ�ʼ
	int i,j;
	
	//ȡ��16x16���ݶȵ㴰��
	GradsPoint* wndGradsPts			= new GradsPoint[wndBufLength];//ԭʼ����ֵ
	GradsPoint* filteringGradsPts	= new GradsPoint[wndBufLength];//�˲��󴰿�ֵ
	int wndPos = 0;
	for(i=0;i<ftrPtWndSize;i++)
	{
		for(j=0;j<ftrPtWndSize;j++)
		{
			wndGradsPts[wndPos] = pGradsPts[imgBufPos];
			wndPos+=1;
			imgBufPos+=1;
		}		
		imgBufPos = imgBufPos+imgWidth-ftrPtWndSize;			
	}
	float* filter = new float[wndBufLength];
	float  detta  = float(1.6*1.5);//1.5���ڻ�׼��˹���������detta
	GenGaussTemplate(filter,ftrPtWndSize,ftrPtWndSize,detta);
	//�Դ����ڵ���ݶȽ��и�˹��Ȩ
	wndPos = 0;
	for(i=0;i<ftrPtWndSize;i++)
	{
		for(j=0;j<ftrPtWndSize;j++)
		{
			filteringGradsPts[wndPos].magnitude   = float( wndGradsPts[wndPos].magnitude*filter[wndPos] );
			filteringGradsPts[wndPos].orientation = wndGradsPts[wndPos].orientation;
			wndPos+=1;
		}
	}
    //����ֱ��ͼ
	int   leftIntPos,rightIntPos;//�ٽ�ֱ��ͼλ��
	float fltHstgrmAnglePos;
	float dstOfIntplHstGrm;//ֱ��ͼ�ڲ����

	for(i=0;i<wndBufLength;i++)
	{
		fltHstgrmAnglePos    = float( filteringGradsPts[i].orientation/RADIAN_OF_10DEGREE);
		leftIntPos           = int (fltHstgrmAnglePos);
		rightIntPos          = leftIntPos+1;
		dstOfIntplHstGrm     = fltHstgrmAnglePos - leftIntPos;

		if (rightIntPos>=36)
		{
			rightIntPos = 0;
		}
		//siftPt.histgramOf36[leftIntPos].histogram[leftIntPos] = 
		siftPt.histgramOf36.histogram[leftIntPos]+= float( (1.000-dstOfIntplHstGrm)*filteringGradsPts[i].magnitude);
		siftPt.histgramOf36.histogram[rightIntPos]+= float(dstOfIntplHstGrm*filteringGradsPts[i].magnitude);		
	}
	//ͳ��ֱ��ͼ����
	//����
	HstGramSort srtHis[36];
	for(i=0;i<36;i++)
	{
		srtHis[i].hstValue = siftPt.histgramOf36.histogram[i];
		srtHis[i].index    = i;
	}	
	qsort(srtHis,36,sizeof(HstGramSort),SortHstGram);
	//����ֱ��ͼ������(��������ļ�ֵ�����0.8���ֵ��,Ҳ������Ϊ������-����������)
	int mainOriNum = siftPt.histgramOf36.mainOriNum = 1;
	float maxValue = srtHis[0].hstValue;
	siftPt.histgramOf36.mainOriIndex[0] = srtHis[0].index;
	for(i=1;i<36;i++)
	{
		if (srtHis[i].hstValue>0.8*maxValue)
		{
			siftPt.histgramOf36.mainOriIndex[mainOriNum] = srtHis[i].index;
			mainOriNum+=1;
			siftPt.histgramOf36.mainOriNum				= mainOriNum;
			
		}
		
	}
	//�ͷ��ڴ�
	delete []wndGradsPts;
	delete []filteringGradsPts;
	delete []filter;


	
}
void CCVClass::DescriptSIFTPoint(GradsPoint *&pGradsPts, int imgWidth, int imgHeight, SIFTPoint &siftPt)
{
	//////////////////////////////////////////////////////////////////////////
	int x = int(siftPt.imgX+0.5);
	int y = int(siftPt.imgY+0.5);
	int i,j,k,l;

	int lclImgWndSize = 16;//�ֲ����ڴ�С(256���ȶ�)
	int lclImgWndBufLength = lclImgWndSize*lclImgWndSize;
	int discriptPtNum = (lclImgWndSize/4)*(lclImgWndSize/4);//16�����ӵ�
	int discripWndSize= lclImgWndSize/4;//�������������ڴ�С(4x4)
	//ȡ�ֲ�����Ӱ��(Ҫ��תӰ������ϵ�����ݶȷ���!!!!)
	int lclWndBufLength = lclImgWndSize*lclImgWndSize;	
	int lclImgBufPos = (y-lclImgWndSize/2)*imgWidth + x-lclImgWndSize/2;//���½ǿ�ʼ
	
	//ȡ��16x16���ݶȵ㴰��
	GradsPoint* lclWndGradsPts			    = new GradsPoint[lclWndBufLength];//ԭʼ����ֵ
	GradsPoint* lclFilteringGradsPts		= new GradsPoint[lclWndBufLength];//�˲��󴰿�ֵ
	int lclWndPos = 0;
	for(i=0;i<lclImgWndSize;i++)
	{
		for(j=0;j<lclImgWndSize;j++)
		{
			lclWndGradsPts[lclWndPos] = pGradsPts[lclImgBufPos];
			lclWndPos+=1;
			lclImgBufPos+=1;
		}		
		lclImgBufPos = lclImgBufPos+imgWidth-lclImgWndSize;			
	}



	//���ô�����ʱ����ת�����ݶȷ���
	float mainGradsOri = siftPt.mainGradsOri;
	//��ת��
	float rtAngle  = mainGradsOri;//Ϊ�˱�֤���Ĵ��ڲ���,Ӧ�ƴ���������ת
//	rtAngle = 0.000;
	//��ת�ݶ�
	lclWndPos = 0;
	for(i=0;i<lclImgWndBufLength;i++)
	{
		lclWndGradsPts[lclWndPos].orientation-= rtAngle;//��֤��ת��Ƕ���0 - 360��
		if(lclWndGradsPts[lclWndPos].orientation<0.f)
		{
			lclWndGradsPts[lclWndPos].orientation+=float(DOUBLE_PI);
		}

		lclWndPos+=1;
		
	}
	
	//��ת����
	lclWndPos = 0;
	double lcx,lcy,oriX,oriY;
	int oriI,oriJ;
	GradsPoint* rtWndGradsPts = new GradsPoint[lclWndBufLength];
	for(i=0;i<lclImgWndSize;i++)
	{
		for(j=0;j<lclImgWndSize;j++)
		{
			lcx = j - 8.f;
			lcy = i - 8.f;
			oriX = cos(rtAngle)*lcx - sin(rtAngle)*lcy;
			oriY = sin(rtAngle)*lcx + cos(rtAngle)*lcy;
			oriJ = int(oriX + 8.f + 0.5);
			oriI = int(oriY + 8.f + 0.5);
			if (oriJ<0)		{		oriJ = 0;		}
			if (oriJ>15)	{		oriJ = 15;		}
			if (oriI<0)		{		oriI = 0;		}
			if (oriI>15)	{		oriI = 15;		}
			rtWndGradsPts [lclWndPos] = lclWndGradsPts[oriI*16+oriJ];
			lclWndPos++;
		}
	}
	


	float* lclFilter = new float[lclWndBufLength];
	float  lclDetta  = 8;//discripWndSize/2.f;//0.5�������������ڿ��
	GenGaussTemplate(lclFilter,lclImgWndSize,lclImgWndSize,lclDetta);
	//�Դ����ڵ���ݶȽ��и�˹��Ȩ
	lclWndPos = 0;
	for(i=0;i<lclImgWndBufLength;i++)
	{
		lclFilteringGradsPts[lclWndPos].magnitude   = float( rtWndGradsPts[lclWndPos].magnitude*lclFilter[lclWndPos] );
		lclFilteringGradsPts[lclWndPos].orientation = rtWndGradsPts[lclWndPos].orientation;
		lclWndPos+=1;

	}
	
	////////////////////////////////////////////////////////////
	//����ֱ��ͼ(�����ӵ�ͳ��4x4���ӵ��ֱ��ͼ)
	Grads8Histogram curSeedPt;
	GradsPoint curWndGradPt[16] ;
	int smpBufPos = 0;
	int curWndBufPos = 0;
	int descriptVectPos = 0;
	for(i=0;i<discripWndSize;i++)
	{
		for(j=0;j<discripWndSize;j++)
		{
			//ȡ����ǰ���ӵ�����16���ݶ�
			smpBufPos = i*64 + j*4;// i*4*16 + j*4*16//��ǰ���ӵ㴰�����½ǵ���256�����µ�ָ��λ��
			curWndBufPos = 0;
			for(k=0;k<4;k++)
			{
				for(l=0;l<4;l++)
				{		
					curWndGradPt[curWndBufPos] = lclFilteringGradsPts[smpBufPos];
					curWndBufPos+=1;
					smpBufPos+=1;
				}
				smpBufPos = smpBufPos+16-4;
			}
			//ͳ��ֱ��ͼ
			
			Cal8Histogram(curWndGradPt,16,curSeedPt);
			for(k=0;k<8;k++)
			{
				siftPt.featureVector[descriptVectPos] = curSeedPt.histogram[k];
				descriptVectPos+=1;
			}
		}
	}
//	//�ͷ��ڴ�
	delete []lclWndGradsPts;
	delete []lclFilteringGradsPts;
	delete []lclFilter;
	delete []rtWndGradsPts;



}

BOOL CCVClass::OutputSiftPts(SIFTPoint *pSiftPt, int ptNum, CString filePath)
{
	FILE* fp = fopen(filePath,"w");
	if (fp==NULL)
	{
		CString errInfo;
		errInfo.Format("Write SiftPoints File Fail!\n\n%s",filePath);
		//AfxMessageBox(errInfo);
		return FALSE;
	}
	fprintf(fp,"%10d %10d\n",ptNum,128);
	int i,j;
	for(i=0;i<ptNum;i++)
	{
		fprintf(fp,"%10.3f %10.3f %10.3f %10.3f\n", pSiftPt[i].imgX,
													pSiftPt[i].imgY,
													pSiftPt[i].mainGradsMgnt,
													pSiftPt[i].mainGradsOri);
	//	fprintf(fp,"%10d\n",128);
		for(j=0;j<128;j++)
		{
			fprintf(fp,"%10.3f ",pSiftPt[i].featureVector[j]);
			if (j%10==9)
			{
				fprintf(fp,"\n");
			}
		}
		fprintf(fp,"\n");
	}
	fclose(fp);
	return TRUE;
}


BOOL CCVClass::ReadSiftPts(SIFTPoint*& pSiftPt, int& ptNum, CString filePath)
{
	
	FILE* fp = fopen(filePath,"r");
	if (fp==NULL)
	{
		CString errInfo;
		errInfo.Format("Read SiftPoints File Fail!\n\n%s",filePath);
		//AfxMessageBox(errInfo);
		return FALSE;
	}

	if (pSiftPt!=NULL)
	{
		delete []pSiftPt; pSiftPt = NULL;
	}
	
	int tempInt;
	fscanf(fp,"%d%d",&ptNum,&tempInt);
	if (ptNum<=0)
	{
		return TRUE;
	}
	pSiftPt = new SIFTPoint[ptNum];
	int i,j;

	for(i=0;i<ptNum;i++)
	{
		fscanf(fp,"%f%f%f%f",  &pSiftPt[i].imgX,
							   &pSiftPt[i].imgY,
							   &pSiftPt[i].mainGradsMgnt,
							   &pSiftPt[i].mainGradsOri);
//		fscanf(fp,"%d\n",&tempInt);
//		pSiftPt[i].imgY = 640-pSiftPt[i].imgY;


/*
		//��ʾ��������
		float tempSwap = pSiftPt[i].imgX;
		pSiftPt[i].imgX= pSiftPt[i].imgY;
		pSiftPt[i].imgY= 640-tempSwap;
		*/


		for(j=0;j<tempInt;j++)
		{
			fscanf(fp,"%f",&pSiftPt[i].featureVector[j]);
		}
	}
	fclose(fp);
	return TRUE;
}


void CCVClass::NormalizeSiftPoints(SIFTPoint *pSiftPts, int ptNum, float threshold)
{
	threshold = 0.2f;
	int i,j;
	double sum = 0.0;
	BOOL bRNrm = FALSE;
	for(i=0;i<ptNum;i++)
	{
		sum = 0.f;

		for(j=0;j<128;j++)
		{
			sum+=pSiftPts[i].featureVector[j]*pSiftPts[i].featureVector[j];
		}
		sum =sqrt(sum);
		if (sum==0.000)
		{
			sum = 0.0001;
		}
		bRNrm = FALSE;
		for(j=0;j<128;j++)
		{
			pSiftPts[i].featureVector[j] = float(pSiftPts[i].featureVector[j]/sum);
			if (pSiftPts[i].featureVector[j]>threshold) 
			{
				pSiftPts[i].featureVector[j] = threshold;
				bRNrm = TRUE;
			}
		}
		//���¹�һ��
		if (bRNrm==TRUE)
		{
			sum = 0.f;
			for(j=0;j<128;j++)
			{
				sum+=pSiftPts[i].featureVector[j]*pSiftPts[i].featureVector[j];
			}
			sum =sqrt(sum);
			if (sum==0.000)
			{
				sum = 0.0001;
			}
			
			for(j=0;j<128;j++)
			{
				pSiftPts[i].featureVector[j] = float(pSiftPts[i].featureVector[j]/sum);
				
			}
		}
		
	}


}



void CCVClass::Cal8Histogram(GradsPoint *pGradsPts, int ptNum, Grads8Histogram& histogram_8)
{
	
	int   leftIntPos,rightIntPos;//�ٽ�ֱ��ͼλ��
	float fltHstgrmAnglePos;
	float dstOfIntplHstGrm;//ֱ��ͼ�ڲ����
	int i;
	memset(histogram_8.histogram,0,sizeof(float)*8);
	for(i=0;i<ptNum;i++)
	{
		fltHstgrmAnglePos    = float( pGradsPts[i].orientation/RADIAN_OF_45DEGREE);
		leftIntPos           = int (fltHstgrmAnglePos);
		rightIntPos          = leftIntPos+1;
		dstOfIntplHstGrm     = float(fltHstgrmAnglePos - leftIntPos);

		if (rightIntPos>=7)
		{
			rightIntPos = 0;
		}

		histogram_8.histogram[leftIntPos]+= float( (1.000-dstOfIntplHstGrm)*pGradsPts[i].magnitude);
		histogram_8.histogram[rightIntPos]+= float(dstOfIntplHstGrm*pGradsPts[i].magnitude);		
	}

}

void CCVClass::CompareSiftPoints(SIFTPoint *pLSiftPts, int lSiftPtsNum, SIFTPoint *pRSiftPts, int rSiftPtsNum, 
								 float *lx, float *ly, float *rx, float *ry, int& matchPtNum,
								 int lWidth,int lHeight,int rWidth,int rHeight,float seedX,float seedY,
								 float searchX,float searchY,int maxNum,int CompareStrategy)
{
//	return ;
    double dsq, distsq1 = 100000000.000, distsq2 = 100000000.000;
     /* Find the two closest matches, and put their squared distances in
       distsq1 and distsq2.
    */
	
	//==================Test For Paper==================//
/*
	SIFTPoint* tempLSiftPts = new SIFTPoint[lSiftPtsNum];
	SIFTPoint* tempRSiftPts = new SIFTPoint[rSiftPtsNum];
	int        tempPtNum    = 0;
*/
	//==================Test For Paper==================//

	int i,j;
	matchPtNum = 0;
	int keyPt = 0;
	int step = 1;
	if(maxNum==1000)	step = 5;

	BOOL *rSiftPtMatched = new BOOL [rSiftPtsNum];
	for(i=0;i<rSiftPtsNum;i++)	rSiftPtMatched[i] = FALSE;
	
	for(i=0;i<lSiftPtsNum;i+= step)
	{
		distsq1 = 100000000, distsq2 = 100000000;
		keyPt   = 0;
		for(j=0;j<rSiftPtsNum;j++)
		{
			if(rSiftPtMatched[j] == TRUE)	continue;

			if(lWidth>0)	//���������С��������Χ
			{
				float x1,y1,x2,y2;
				x1 = pLSiftPts[i].imgX;
				y1 = pLSiftPts[i].imgY;
		//		x2 = (x1-(1.0-seedX )*lWidth );
		//		y2 = (y1+(1.0- seedY)*lHeight);
				x2 = x1 + seedX;
				y2 = y1 + seedY;

				x1 = pRSiftPts[j].imgX;
				y1 = pRSiftPts[j].imgY;

				if(CompareStrategy == 1)		//	1==������ƥ�䣬ʹ����Ӱ���Ҳ�60�������������
				{
					if(x1 < 0.45*lWidth) continue;
				}
				else if(CompareStrategy == 0)	//	0==������ƥ�䣬ʹ����Ӱ���²�60�������������
				{
					if(y1 > 0.55*lHeight) continue;
				}

				if(fabs(x2-x1)>searchX*rWidth || fabs(y2-y1)>searchY*rHeight)
					continue;

			}
		//	dsq = EcldnDst(pLSiftPts[i],pRSiftPts[j]);
			dsq = StrtDst(pLSiftPts[i],pRSiftPts[j]);
			if (dsq < distsq1) 
			{
				distsq2 = distsq1;
				distsq1 = dsq;
				keyPt   = j;
			 } 
			else if (dsq < distsq2) 
			{
				distsq2 = dsq;
			}

			
		}

		if (distsq1 /distsq2 < 0.80 )
		 {
			lx[matchPtNum]	= pLSiftPts[i].imgX;
			ly[matchPtNum]  = pLSiftPts[i].imgY;

			rx[matchPtNum]  = pRSiftPts[keyPt].imgX;
			ry[matchPtNum]  = pRSiftPts[keyPt].imgY;
			
			rSiftPtMatched[keyPt] = TRUE;

			matchPtNum+=1;
	      //==================Test For Paper==================//
/*
			tempLSiftPts[tempPtNum]	= pLSiftPts[i];
			tempRSiftPts[tempPtNum]	= pRSiftPts[keyPt];
	
			tempPtNum+=1;
*/
	     //==================Test For Paper==================//
		 }
				
	}
		//==================Test For Paper==================//
/*
	    OutputSiftPts(tempLSiftPts,tempPtNum,"c:\\L.sift");
		OutputSiftPts(tempRSiftPts,tempPtNum,"c:\\R.sift");
		delete[] tempLSiftPts;
		delete[] tempLSiftPts;
*/
		//==================Test For Paper==================//

	delete rSiftPtMatched;
}
