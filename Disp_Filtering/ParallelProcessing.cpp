// ParallelProcessing.cpp: implementation of the CParallelProcessing class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ParallelProcessing.h"

// #ifdef _DEBUG
// #undef THIS_FILE
// static char THIS_FILE[]=__FILE__;
// #define new DEBUG_NEW
// #endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
//高斯模板，系数0.7
double harris_gaussTemplateParallel07[9];
double harris_gaussTemplateParallel05[9];
double harris_gaussTemplateParallel09[9];
//x方向的梯度：
double harris_xGradientTempalteParallel[9];
//y方向的梯度：
double harris_yGradientTempalteParallel[9];

CParallelProcessing::CParallelProcessing()
{
	m_bMatrixMutiFinish = NULL;   //线程结束标志
	m_pMatrixMutiProcess= NULL;

	m_bSIFTMatchFinish  = NULL;   //线程结束标志
	m_pSIFTMatchProcess = NULL;

	m_bRotateImageFinish = NULL;  //线程结束标志
	m_pRotateImageProcess= NULL;

	m_bHarrisFeatureFinish = NULL;//线程结束标志
	m_pHarrisFeatureProcess= NULL;

	//高斯模板，系数0.7

	double temp_harris_gaussTemplateParallel07[9]={	double(0.04387081413862) , 	double(0.12171198028232) , 	double(0.04387081413862),
											double(0.12171198028232),  	double(0.33766882231624),  	double(0.12171198028232),
											double(0.04387081413862),  	double(0.12171198028232),  	double(0.04387081413862)};
	memcpy(harris_gaussTemplateParallel07,temp_harris_gaussTemplateParallel07,sizeof(double)*9);

	
	double temp_harris_gaussTemplateParallel05[9]={	double(0.0113),double(0.0838),double(0.0113),
											double(0.0838),double(0.6193),double(0.0838),
											double(0.0113),double(0.0838),double(0.0113)}; 
	memcpy(harris_gaussTemplateParallel05,temp_harris_gaussTemplateParallel05,sizeof(double)*9);

	
	double temp_harris_gaussTemplateParallel09[9]={	double(0.0673),double(0.1248),double(0.0673),
											double(0.1248),double(0.2314),double(0.1248),
											double(0.0673),double(0.1248),double(0.0673)};
	memcpy(harris_gaussTemplateParallel09,temp_harris_gaussTemplateParallel09,sizeof(double)*9);

	//x方向的梯度：
	double temp_harris_xGradientTempalteParallel[9]={	-1 , 0 , 1 ,
												-1 , 0 , 1 ,
												-1 , 0 , 1 };
	memcpy(harris_xGradientTempalteParallel,temp_harris_xGradientTempalteParallel,sizeof(double)*9);

	//y方向的梯度：
	double temp_harris_yGradientTempalteParallel[9]={	-1 , -1 , -1 ,
												0 ,  0 ,  0 ,
												1 ,  1 ,  1 };
	memcpy(harris_yGradientTempalteParallel,temp_harris_yGradientTempalteParallel,sizeof(double)*9);


}

CParallelProcessing::~CParallelProcessing()
{

}

//*/////////////////////////////////////////////////////////////
//双线性内插
double CalcInteriorPixelValue(fPoint ptGuest,unsigned char* lpImage,int row,int col)
{
	double deltaX, deltaY;
	double fGray;
	fPoint ptNeighbor[4];
	ptNeighbor[0].x= floor(ptGuest.x);
	ptNeighbor[0].y= floor(ptGuest.y);
	if( ptNeighbor[0].x <0)
		ptNeighbor[0].x = 0;
	if( ptNeighbor[0].x > (col-1) )
		 ptNeighbor[0].x= col-1;

	if( ptNeighbor[0].y <0)
		ptNeighbor[0].y = 0;
	if( ptNeighbor[0].y > (row-1) )
		 ptNeighbor[0].y = row-1;

	ptNeighbor[1].x= ptNeighbor[0].x+1;
	ptNeighbor[1].y= ptNeighbor[0].y;
	if( ptNeighbor[1].x > (col-1) )
		 ptNeighbor[1].x = col-1;

	ptNeighbor[2].x= ptNeighbor[0].x+1;
	ptNeighbor[2].y= ptNeighbor[0].y+1;
	if( ptNeighbor[2].x > (col-1) )
		 ptNeighbor[2].x= col-1;
	if( ptNeighbor[2].y > (row-1) )
		 ptNeighbor[2].y = row-1;

	ptNeighbor[3].x= ptNeighbor[0].x;
	ptNeighbor[3].y= ptNeighbor[0].y+1;
	if( ptNeighbor[3].y > (row-1) )
		 ptNeighbor[3].y = row-1;
	
	deltaX=ptGuest.x-floor(ptGuest.x);
	deltaY=ptGuest.y-floor(ptGuest.y);

	fGray=(1-deltaX)*(1-deltaY)*
		lpImage[(int)(ptNeighbor[0].x)+(int)(ptNeighbor[0].y)*col]+
		(1-deltaY)*deltaX*
		lpImage[(int)(ptNeighbor[1].x)+(int)(ptNeighbor[1].y)*col]+		
		deltaX*deltaY*
		lpImage[(int)(ptNeighbor[2].x)+(int)(ptNeighbor[2].y)*col]+		
		(1-deltaX)*deltaY*
		lpImage[(int)(ptNeighbor[3].x)+(int)(ptNeighbor[3].y)*col];
		
	return fGray;
}
void f_oper4_Parallel(unsigned char* gray,int  row,int  col,double* nor,double* b, int flag)
	//unsigned char 	*gray;
	//int 		row, col, flag;
	//double 		*nor,*b;
{
	int   i,j;
	double         gu,gv,guv;
	double         r,c,u,v;

	for(i=0;i<3;i++) *(nor+i)=0;
	for(i=0;i<2;i++) *(b+i)=0;
	for (i=0; i<row-1; i++)
	{
		for (j=0; j<col-1; j++)
		{
			gu    =(double) *(gray+col+j) - *(gray+j+1);
			gv    =(double) *(gray+col+j+1) - *(gray+j);
			guv   = gu*gv;
			gu   *= gu;
			gv   *= gv;
			r = (double)(i+1.5);
			c = (double)(j+1.5);
			u = GA_TTT* (r-c);
			v = GA_TTT* (r+c);
			if (flag==1)
			{
				*nor     += gu;
				*(nor+1) += guv;
				*(nor+2) += gv;
				*b       += gu*u + guv*v;
				*(b+1)   += guv*u + gv*v;
			}
			else
			{
				*nor     += gv;
				*(nor+1) -= guv;
				*(nor+2) += gu;
				*b       += gv*u - guv*v;
				*(b+1)   += gu*v - guv*u;
			}
		}
		gray += col;
	}
}

void f_prew_Parallel(unsigned char* gray,int  row,int  col,double*  nor,double*  b,int  flag,int  flagg)
//	unsigned char 	*gray;
//	int 		row, col, flag, flagg;
//double 		*nor, *b;
{
	int     i,j;
	double  gx,gy,gxy;
	double  r,c;
	
	for(i=0;i<3;i++) *(nor+i)=0;
	for(i=0;i<2;i++) *(b+i)=0;
	for (i=0; i<row-2; i++)
	{
		for (j=0; j<col-2; j++)
		{
			if(flagg==2)
			{
				gx =  *(gray+2*col+j+2)+ *(gray+col+j+2)
				    + *(gray+j+2)      - *(gray+2*col+j)
				    - *(gray+col+j)    - *(gray+j);
				gy =  *(gray+2*col+j+2)+ *(gray+2*col+j+1)
				    + *(gray+2*col+j)  - *(gray+j+2)
				    - *(gray+j+1)      - *(gray+j);
			}
			if(flagg==3)
			{
				gx =  *(gray+2*col+j+2)+ *(gray+col+j+2)*2
				    + *(gray+j+2)      - *(gray+2*col+j)
				    - *(gray+col+j)*2  - *(gray+j);
				gy =  *(gray+2*col+j+2)+ *(gray+2*col+j+1)*2
				    + *(gray+2*col+j)  - *(gray+j+2)
				    - *(gray+j+1)*2    - *(gray+j);
			}
			gxy   =gx*gy;
			gx   *= gx;
			gy   *= gy;
			r = (double)i+2;
			c = (double)j+2;
			if (flag==1)
			{
				*nor     += gy;
				*(nor+1) += gxy;
				*(nor+2) += gx;
				*b       += gy*r + gxy*c;
				*(b+1)   += gxy*r + gx*c;
			}
			else
			{
				*nor     += gx;
				*(nor+1) -= gxy;
				*(nor+2) += gy;
				*b       += gx*r- gxy*c;
				*(b+1)   += gy*c - gxy*r;
			}
		}
		gray += col;
	}
}
void f_weight_Parallel(unsigned char* gray,int  row, int col,double* x,int  flag,int  flagg)
//	unsigned char 	*gray;
	//int 	 row, col, flag, flagg;
//	double 	 *x;
{
	double *nor,*b,detnor;

	nor  = (double *)malloc(4*sizeof(double));
	b    = (double *)malloc(3*sizeof(double));

	/////////////////////////////////////////
	if(flagg == 1)
		f_oper4_Parallel (gray,row,col,nor,b,flag);
	else
		f_prew_Parallel(gray,row,col,nor,b,flag,flagg);
    /////////////////////////////////////////    
        /*	
        round = direction(nor);
        */
	
	detnor= *nor* *(nor+2)- *(nor+1)* *(nor+1);
	if (detnor == 0.0)
	{
		*x     = (double)(row/2+1);
		*(x+1) = (double)(col/2+1);
	}
	else
	{
		detnor=1.0/detnor;
		*x=    (*b * *(nor+2) - *(b+1) * *(nor+1)) *detnor;
		*(x+1)=(*nor * *(b+1) - *(nor+1) * *b) *detnor;
		if(flagg==1)
		{
			*b     = GA_TTT * ( *x + *(x+1));
			*(b+1) = GA_TTT * ( *(x+1) - *x);
			*x     = *b;
			*(x+1) = *(b+1);
		}
	}
	*x 	-= 1;
	*(x+1)  -= 1;
	free(nor);
	free(  b);
}

//采用harris算子提取n个特征点，返回实际特征点数
int HarrisFeaturePointExtract_Parallel(unsigned char*pImg,int Height,int Width,//图像数据及其高宽
							  //返回特征点坐标，和相应每个点的强度值
							  float *px,float*py,float*attri,
							  int n,int win,int dg,bool bExactLocation)
{
	int i=0,j=0,k=0,l=0;

	long size = Height*Width;
	double *Ix2 = new double[size];
	double *Iy2 = new double[size];
	double *Ixy = new double[size];

	unsigned char *deleteIndex = new unsigned char[size];

	memset(Ix2,0,sizeof(double)*size);
	memset(Iy2,0,sizeof(double)*size);
	memset(Ixy,0,sizeof(double)*size);
	memset(deleteIndex,0,sizeof(unsigned char)*size);
    //////////////////////////////////////////////////////////////////
	int totalNum = 0;
	int aa = 0;
	//一阶差分：X  Y方向
	//一阶差分的平方
	double value1 = 0,value2 = 0,value3 = 0;
    for(i =1;i<Height-1;i++)
	{
		int row = i*Width;
		for(j=1;j<Width-1;j++)
		{

			//计算当前点在米字形四个方向上的一阶差分，
			//并判断其是否大于阈值，必须保证在2个以上的方向上大于阈值的点才进行高斯滤波等处理，
			//否则仅仅计算差分
			int flag = 0;
			//左右方向
			int dg1 = pImg[  row +(j+1)] - pImg[  row +(j-1)];
			if(abs(dg1) >= dg) flag ++;
            //上下方向
			int dg2 = pImg[row + Width+  j  ] - pImg[row - Width+  j  ];
			if(abs(dg2) >= dg) flag ++;
			//左上右下方向、左下右上方向	
			if(flag < 2)
			{
					//左上右下方向
					int dg3 = pImg[row + Width+(j+1)] - pImg[row - Width+(j-1)];
					if(abs(dg3) >= dg)
						flag ++;
					if(flag < 2)
					{
						//左下右上方向
						int dg4 = pImg[row + Width+(j-1)] - pImg[row - Width+(j+1)];
						if(abs(dg4) >= dg)
							flag ++;
					}
			}
			if(flag < 2)
			{
				deleteIndex[row +j] = 1;
				//continue;
				aa ++;
			}		
			/////////////////////////////////////////////////////////////////
			//一阶差分
			value1 = (	pImg[row - Width+(j+1)] - pImg[row - Width+(j-1)] +
							//pImg[ row +(j+1)      ] - pImg[  row +(j-1)     ] +
							dg1 +
							pImg[row + Width+(j+1)] - pImg[row + Width+(j-1)]);

			value2 = (	pImg[row + Width+(j-1)] - pImg[row - Width+(j-1)] +
							//pImg[row + Width+  j  ] - pImg[row - Width+  j  ] +
							dg2 +
							pImg[row + Width+(j+1)] - pImg[row - Width+(j+1)]);
		    //////////////////////////////////////////////////
    		Ixy[row+j] = value1 * value2;
			Ix2[row+j] = value1 * value1;
			Iy2[row+j] = value2 * value2;
			////////////////////
		}
	}
	//////////////////////////
	//对“差分平方”及“x y差分之积”进行高斯滤波
	double * Intensity = new double [size];
 	memset(Intensity,9999,sizeof(double)*size);

	for(i =1;i<Height-1;i++)
	{	
		int row = i*Width;
		for(j=1;j<Width-1;j++)
		{
			int center = row +j;
			/////////////////				
			if(deleteIndex[center] == 1)
			{
				Intensity[center] = GA_INVALID;
				continue;
			}
			/////////////////
			value1 = Ix2[center-Width-1] * harris_gaussTemplateParallel07[0] + 
					 Ix2[center-Width]   * harris_gaussTemplateParallel07[1] +
					 Ix2[center-Width+1] * harris_gaussTemplateParallel07[2] +
					 Ix2[center-1]       * harris_gaussTemplateParallel07[3] + 
					 Ix2[center]         * harris_gaussTemplateParallel07[4] +
					 Ix2[center+1]       * harris_gaussTemplateParallel07[5] +
					 Ix2[center+Width-1] * harris_gaussTemplateParallel07[6] + 
					 Ix2[center+Width]   * harris_gaussTemplateParallel07[7] +
					 Ix2[center+Width+1] * harris_gaussTemplateParallel07[8];

			value2 = Iy2[center-Width-1] * harris_gaussTemplateParallel07[0] + 
					 Iy2[center-Width]   * harris_gaussTemplateParallel07[1] +
					 Iy2[center-Width+1] * harris_gaussTemplateParallel07[2] +
					 Iy2[center-1]       * harris_gaussTemplateParallel07[3] + 
					 Iy2[center]         * harris_gaussTemplateParallel07[4] +
					 Iy2[center+1]       * harris_gaussTemplateParallel07[5] +
					 Iy2[center+Width-1] * harris_gaussTemplateParallel07[6] + 
					 Iy2[center+Width]   * harris_gaussTemplateParallel07[7] +
					 Iy2[center+Width+1] * harris_gaussTemplateParallel07[8];

			value3 = Ixy[center-Width-1] * harris_gaussTemplateParallel07[0] + 
		 			 Ixy[center-Width]   * harris_gaussTemplateParallel07[1] +
					 Ixy[center-Width+1] * harris_gaussTemplateParallel07[2] +
					 Ixy[center-1]       * harris_gaussTemplateParallel07[3] + 
					 Ixy[center]         * harris_gaussTemplateParallel07[4] +
					 Ixy[center+1]       * harris_gaussTemplateParallel07[5] +
					 Ixy[center+Width-1] * harris_gaussTemplateParallel07[6] + 
					 Ixy[center+Width]   * harris_gaussTemplateParallel07[7] +
					 Ixy[center+Width+1] * harris_gaussTemplateParallel07[8];

			//////////////////
            //求每个象素的Harris强度值
			double value = value1 * value2 - value3 * value3;
			if(value == 0)	   
			{
				Intensity[center] = GA_INVALID;
				continue;
			}
			else
				Intensity[center] = (value1 + value2) / ( value);
			/////////////////
			totalNum ++;
		}
	}
	delete[]Ix2; Ix2 = NULL;
	delete[]Iy2; Iy2 = NULL;
	delete[]Ixy; Ixy = NULL;
	delete[]deleteIndex;	deleteIndex = NULL;

	//寻找局部最优------>Intensity值最小:根据计算harris强度值的公式不同，可能是以局部最大为准
	//能否将局部最优策略改成：将整块数据分成若干小块，
	//而采用冒泡法取小块区域内Intensity值最小的点为特征点，其它点不作比较？

	int *ptx = new int[totalNum];
	int *pty = new int[totalNum];
	double *pLocalOptimal = new double[totalNum];	

 	memset(ptx,0,sizeof(int)*totalNum);
 	memset(pty,0,sizeof(int)*totalNum);
 	memset(pLocalOptimal,0,sizeof(double)*totalNum);

    int sum = 0;
    bool bLocalOptimal = true;

	int winRadius = win/2;
	for(i=(winRadius+1);i<Height-(winRadius+1);i++)
	{
		for(j=(winRadius+1);j<Width-(winRadius+1);j++)
		{
			int index1 = i*Width+j;
			if(Intensity[index1] == GA_INVALID ) 
				continue;
			//////////////////
			bLocalOptimal = true;
			for(k=0;k<win;k++)
			{
				for(l=0;l<win;l++)
				{
					int index2 = (i+k-winRadius)*Width+(j+l-winRadius);

					if(Intensity[index2] == GA_INVALID) continue;
					///////////////
                    if(Intensity[index1] > Intensity[index2])
					{
						bLocalOptimal = false;		k = win;	l = win;
						break;
					}
				}				
			}
			///////////////
			if(!bLocalOptimal)
			{
				Intensity[index1] = GA_INVALID;
				continue;
			}
			/////////////						
			pLocalOptimal[sum] = Intensity[index1];
			ptx[sum] = j;
			pty[sum] = i;
			sum ++;
		}
	}
	delete[]Intensity; Intensity = NULL;

	//按Harris强度，从小到大进行冒泡排序，取出足够特征点之后退出排序
	int tempP;
	double m_temp = 0;
    //冒泡法，逐个取出强度最小的特征点
    for(i=0;i<sum-1;i++)
	{
		for(j = i+1;j<sum;j++)
		{
			if(pLocalOptimal[i] > pLocalOptimal[j])
			{
				//交换点坐标
				tempP = ptx[i];			ptx[i] = ptx[j];	ptx[j] = tempP;
				tempP = pty[i];			pty[i] = pty[j];	pty[j] = tempP;
                //交换强度值
				m_temp = pLocalOptimal[i];
				pLocalOptimal[i] = pLocalOptimal[j];
				pLocalOptimal[j] = m_temp;
				//若已经达到要求的点数，则退出排序
				if(i == n)
				{
					i = sum;
					j = sum;
				}

			}

		}
	}

	//对提取的特征点进行精确定位
	unsigned char*gray = new unsigned char[7*7];
	memset(gray,0,sizeof(unsigned char)*7*7);
	double *x = new double[2];
	memset(x,0,sizeof(double)*2);

	if(sum > n) sum = n;
    for(i =0;i<sum;i++)
	{
		int x0 = ptx[i];
		int y0 = pty[i];
		/////////////////////////////////////////////
		if(!bExactLocation)//不进行精确定位
		{
			py[i] = float(y0);			px[i] = float(x0);
			attri[i] = (float)pLocalOptimal[i];//返回强度值
			continue;
		}
		/////////////////////////////////////////////
		///////////
		if(x0 <= 2 || y0 <= 2 || x0 >= Width-3 || y0 >= Height-3)
		{
			py[i] = float(y0);			px[i] = float(x0);
			attri[i] = (float)pLocalOptimal[i];//返回强度值
			continue;
		}
		//以pInterestP[i]为中心取出小块数据
		for(int k=0;k<7;k++)
		{
			for(int l =0;l<7;l++)
			{
				gray[k*7+l] = pImg[(y0+k-3)*Width + (x0 + l -3)];
			}
		}
		//注意：x返回的坐标是以gray左上角为原点的坐标，
		//      因此，必须将pInterestP[i]首先移到gray的左上角位置，即x,y均减去3个像素
		f_weight_Parallel(gray,7,7,x,1,1);
//		pInterestP[i].y =float(y0 + (*x) -3);
//		pInterestP[i].x =float(x0 + ( *(x+1)) - 3);
//		pInterestP[i].attr = pLocalOptimal[i];//返回强度值
		py[i] =float(y0 + (*x) -3);
		px[i] =float(x0 + ( *(x+1)) - 3);
		attri[i] = (float)pLocalOptimal[i];//返回强度值
	}
	delete[]pLocalOptimal; pLocalOptimal = NULL;
	delete[]gray; gray = NULL;
	delete[]x; x = NULL;

	////////////////////////////////////////
	delete[]ptx; ptx = NULL;
	delete[]pty; pty = NULL;

    return sum;
}

////矩阵相乘线程函数////
DWORD WINAPI ProcThread_MatrixMultiply(LPVOID lpParameter)
{
	MatrixMutiProcess *lp = (MatrixMutiProcess *) lpParameter;
	int Total = lp->nNumOfProcessors;
	int Count = lp->nThreadID;
	double *A = lp->A;
	double *B = lp->B;
	//A矩阵的行数与列数
	int m = lp->nrow;
	int p = lp->np;
	int n = lp->ncol;
	int nnrow = m/Total;
	int res = m%Total;
	//起始行与终止行
	int row0 = Count*nnrow; 
	int row1 = (Count+1)*nnrow;
	if( Count == Total-1)
		row1 =  (Count+1)*nnrow  + res;
	
	int i,j,k;
	for(i=row0; i<row1; i++)
		for(j=0; j<n; j++)
		{
			lp->Result[i*n+j] = 0.0;
			for(k=0; k<p; k++)
				lp->Result[i*n+j] += A[i*p+k]*B[k*n+j];
		}
		
		lp->bFinish[Count] = 1;
		return 0;
}

////SIFT匹配线程函数////
DWORD WINAPI ProcThread_SIFTMatch(LPVOID lpParameter)
{
	SIFTMatchProcess *lp = (SIFTMatchProcess *) lpParameter;

	int nNumOfProcessors = lp->nNumOfProcessors;
	int nCurThreadID     = lp->nThreadID;
	SIFTPoint* pLSiftPts = lp->pLSiftPts;
	SIFTPoint* pRSiftPts = lp->pRSiftPts;
	int *rSiftPtMatched  = lp->rSiftPtMatched;
	int *pMatchedPtID    = lp->pMatchedPtID;

	int lSiftPtsNum	 = lp->lSiftPtsNum;
	int rSiftPtsNum	 = lp->rSiftPtsNum;
	int lWidth     	 = lp->lWidth     ;
	int lHeight    	 = lp->lHeight    ;
	int rWidth     	 = lp->rWidth     ;
	int rHeight    	 = lp->rHeight    ;
	
	int CompareStrategy = lp->CompareStrategy;

	float seedX      = lp->overlapX   ;
	float seedY      = lp->overlapY   ;
	float searchX    = lp->searchX    ;
	float searchY    = lp->searchY    ;

	int nPts = lSiftPtsNum/nNumOfProcessors;
	int res  = lSiftPtsNum%nNumOfProcessors;

	//起始行与终止行
	int startID = nCurThreadID*nPts; 
	int endID   = (nCurThreadID+1)*nPts;
	if( nCurThreadID == nNumOfProcessors-1)
		endID =  (nCurThreadID+1)*nPts + res;
	
	int i=0,j=0,k=0,keyPt;
    double dsq, distsq1 = 100000000.000, distsq2 = 100000000.000;
	
	for(i=startID;i<endID;i++)
	{
		distsq1 = 100000000, distsq2 = 100000000;
		keyPt   = 0;
		for(j=0;j<rSiftPtsNum;j++)
		{
			if(rSiftPtMatched[j] == TRUE)	continue;

			if(lWidth>0)	//输入像幅大小和搜索范围
			{
				float x1,y1,x2,y2;
				x1 = pLSiftPts[i].imgX;
				y1 = pLSiftPts[i].imgY;

				if(CompareStrategy == 1)		//	1==航带内匹配，使用左影像右部60％像幅的特征点
				{
					if(x1 < 0.45*lWidth) continue;
				}
				else if(CompareStrategy == 0)	//	0==航带间匹配，使用左影像下部60％像幅的特征点
				{
					if(y1 > 0.55*lHeight) continue;
				}

				x2 = x1 + seedX;
				y2 = y1 + seedY;

				x1 = pRSiftPts[j].imgX;
				y1 = pRSiftPts[j].imgY;

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
			pMatchedPtID[i] = keyPt;
			rSiftPtMatched[keyPt] = TRUE;
		}			
	}
	
	lp->bFinish[nCurThreadID] = 1;
	return 0;
}
////影像旋转重采样线程函数////
DWORD WINAPI ProcThread_RotateImageWinToHorizon(LPVOID lpParameter)
{

	RotateImageProcess *lp = (RotateImageProcess *) lpParameter;

	int nNumOfProcessors = lp->nNumOfProcessors;
	int nCurThreadID     = lp->nThreadID;

	int row = lp->row;
	int col = lp->col;
	float x = lp->x;
	float y = lp->y;
	float fRotateAngle = lp->fRotateAngle;
	int nImageWinWidth = lp->nImageWinWidth;
	int nImageWinLength= lp->nImageWinLength;
	unsigned char* lpImage= lp->lpImage;
	BYTE *pImageStrip     = lp->pImageStrip;


	//起始行与终止行
	int nLines = nImageWinWidth/nNumOfProcessors;
	int res  = nImageWinWidth%nNumOfProcessors;

	//起始行与终止行
	int startID = nCurThreadID*nLines; 
	int endID   = (nCurThreadID+1)*nLines;
	if( nCurThreadID == nNumOfProcessors-1)
		endID =  (nCurThreadID+1)*nLines + res;

	int i,j;
	double fGrayLevel;
	fPoint ptInOriImage, ptInHorImage,  ptDisplace;//
  
	ptDisplace.x=x;
	ptDisplace.y=y;

	double cosRotateAngle=cos(fRotateAngle);
	double sinRotateAngle=sin(fRotateAngle);

	for(i= -nImageWinWidth/2+startID; i< -nImageWinWidth/2+endID; i++)
	{
		for(j=-nImageWinLength/2; j<= nImageWinLength/2; j++)
		{//从左下角算起
			ptInHorImage.x=j;
			ptInHorImage.y=i;

			ptInOriImage.x=ptInHorImage.x*cosRotateAngle - ptInHorImage.y*sinRotateAngle + ptDisplace.x;
			ptInOriImage.y=ptInHorImage.x*sinRotateAngle + ptInHorImage.y*cosRotateAngle + ptDisplace.y;

			fGrayLevel=CalcInteriorPixelValue(ptInOriImage,lpImage,row,col);
    		pImageStrip[(i+nImageWinWidth/2)*nImageWinLength+(j+nImageWinLength/2)]=(BYTE)(fGrayLevel+0.5); //[0][0] stores gray value of lower left point
		}
	}
	lp->bFinish[nCurThreadID] = 1;
	return 0;
}

///特征点提取线程函数////
DWORD WINAPI ProcThread_HarrisFeaturePoint(LPVOID lpParameter)
{

	HarrisFeatureProcess *lp = (HarrisFeatureProcess *) lpParameter;

	int nNumOfProcessors = lp->nNumOfProcessors;
	int nCurThreadID     = lp->nThreadID;

	unsigned char*pImg = lp->pImg;
	int Height = lp->Height;
	int Width  = lp->Width;
	float *px  = lp->px;
	float *py  = lp->py;
	float *attri=lp->attri;
	int gridRow= lp->gridRow;
	int gridCol= lp->gridCol;
	int win    = lp->win;
	int dg     = lp->dg;
	bool bExactLocation=lp->bExactLocation;
	int numX = Width/gridCol;
	int numY = Height/gridRow;	

	//起始行与终止行
	int nRows = numY/nNumOfProcessors;
	int res   = numY%nNumOfProcessors;

	//起始行与终止行
	int startID = nCurThreadID*nRows; 
	int endID   = (nCurThreadID+1)*nRows;
	if( nCurThreadID == nNumOfProcessors-1)
		endID =  (nCurThreadID+1)*nRows + res;

	int i=0,j=0;
	float x0=0,y0=0;
	float attri0 = 0;
	unsigned char*gray = new unsigned char[gridCol*gridRow];

	//////////////////
	for(i=startID*gridRow;i<endID*gridRow;i+=gridRow)
	{
		for(int j=0;j<Width-gridCol+1;j+=gridCol)
		{
			//取出格网数据
			for(int k=0;k<gridRow;k++)
			{
				for(int l =0;l<gridCol;l++)
				{
					gray[k*gridCol+l] = pImg[(i+k)*Width+(j+l)];
				}
			}
			//在格网中提取一个特征点
			int n = HarrisFeaturePointExtract_Parallel(gray,gridRow,gridCol,&x0,&y0,&attri0,1,win,dg,bExactLocation);
			if(n == 1)
			{
				px   [i/gridRow*numX + j/gridCol] = x0+j;
				py   [i/gridRow*numX + j/gridCol] = y0+i;
				attri[i/gridRow*numX + j/gridCol] = attri0;
			}
		}
	}

	lp->bFinish[nCurThreadID] = 1;
	delete gray; gray = NULL;

	return 0;
}

////多cpu乘法：A阵按行分块，B阵不变////
///////////////////////////////////////////////
//设A为m x p阶矩阵，B为p x n阶矩阵，Result为m x n阶矩阵，计算 Result=A x B的子程序为：
void CParallelProcessing::MatrixMultiply_MultiThread(double *A, double *B, double *Result, int m, int p, int n,int maxThreadNum ) 
{
	if(A==NULL||B==NULL||Result==NULL) return;
	if(m<=0 || p<=0 || n<=0) return;

	for(int i=0; i<m*n; i++)	Result[i] = 0;


	//////////////声明硬件信息结构变量////////////////////
	SYSTEM_INFO siSysInfo; 
	//复制硬件信息到SYSTEM_INFO结构变量
	GetSystemInfo(&siSysInfo); 
	int nNumOfProcessors = siSysInfo.dwNumberOfProcessors;
	//////////////////////////////////////////////////////
	
	nNumOfProcessors = min(nNumOfProcessors,maxThreadNum);	//线程个数取CPU核数和限定的最大线程数的小值，避免计算资源全部占用

	m_bMatrixMutiFinish = new int[nNumOfProcessors];
	memset(m_bMatrixMutiFinish,0,nNumOfProcessors*sizeof(int));
	
	if( m_pMatrixMutiProcess != NULL )
	{
		delete[] m_pMatrixMutiProcess;
		m_pMatrixMutiProcess = NULL;
	}

	m_pMatrixMutiProcess = new MatrixMutiProcess[ nNumOfProcessors ];
	memset(m_pMatrixMutiProcess,0,nNumOfProcessors*sizeof(MatrixMutiProcess));
	
	for(int i=0; i<nNumOfProcessors; i++)
	{	
		m_pMatrixMutiProcess[i].A = A;
		m_pMatrixMutiProcess[i].B = B;
		m_pMatrixMutiProcess[i].nThreadID = i;
		m_pMatrixMutiProcess[i].nrow = m;
		m_pMatrixMutiProcess[i].ncol = n;
		m_pMatrixMutiProcess[i].np = p;
		m_pMatrixMutiProcess[i].nNumOfProcessors = nNumOfProcessors;	
		m_pMatrixMutiProcess[i].bFinish = m_bMatrixMutiFinish;
		m_pMatrixMutiProcess[i].Result = Result;
	}
	
	HANDLE *pHandle;
	pHandle = new HANDLE[nNumOfProcessors];
	
	for(int i = 0 ; i < nNumOfProcessors; i++)
	{
		pHandle[i] = ::CreateThread(NULL,0,ProcThread_MatrixMultiply,(void*)&m_pMatrixMutiProcess[i],0,NULL);
//		CloseHandle(pHandle[i]);
	}
		
/*
	while(true)
	{
		int flgover = 1;
		for(i=0; i<nNumOfProcessors; i++)
		{
			flgover *= m_bFinish[i];
		}
		if(flgover) 
		{
			break;
		}
		else
		{
			Sleep(1);
			MSG msg;
			if( ::PeekMessage( &msg, NULL, 0, 0, PM_REMOVE) )
			{
				::TranslateMessage( &msg );
				::DispatchMessage( &msg );
			}
		}
	}
*/
	DWORD res = WaitForMultipleObjects(nNumOfProcessors,pHandle,TRUE,INFINITE);

	if(res == WAIT_OBJECT_0)
	{
		for(int i=0; i<nNumOfProcessors; i++)
			CloseHandle(pHandle[i]);

		delete m_bMatrixMutiFinish;	m_bMatrixMutiFinish =NULL;
		delete m_pMatrixMutiProcess;	m_pMatrixMutiProcess=NULL;
	}
}

//*////////////////////////////////////////////////////////////
void CParallelProcessing::CompareSiftPoints_MultiThread(SIFTPoint *pLSiftPts, int lSiftPtsNum, SIFTPoint *pRSiftPts, int rSiftPtsNum, 
								 float *lx, float *ly, float *rx, float *ry, int& matchPtNum,
								 int lWidth,int lHeight,int rWidth,int rHeight,float seedX,float seedY,
								 float searchX,float searchY,int maxNum,int maxThreadNum, int CompareStrategy)
{
    double  distsq1 = 100000000.000, distsq2 = 100000000.000;

	int i=0,j=0;
	matchPtNum = 0;
	int keyPt = 0;
	int step = 1;
	if(maxNum==1000)	step = 5;
	if(lSiftPtsNum<=0 || rSiftPtsNum<=0) return;

	BOOL *rSiftPtMatched = new BOOL [rSiftPtsNum];
	for(i=0;i<rSiftPtsNum;i++)	rSiftPtMatched[i] = FALSE;

	int *pMatchedPtID = new int[lSiftPtsNum];
	for(i=0;i<lSiftPtsNum;i++)	pMatchedPtID[i] = -1;

	//////////////声明硬件信息结构变量////////////////////
	SYSTEM_INFO siSysInfo; 
	//复制硬件信息到SYSTEM_INFO结构变量
	GetSystemInfo(&siSysInfo); 
	int nNumOfProcessors = siSysInfo.dwNumberOfProcessors;
	//////////////////////////////////////////////////////
	
	nNumOfProcessors = min(nNumOfProcessors,maxThreadNum);	//线程个数取CPU核数和限定的最大线程数的小值，避免计算资源全部占用

	m_bSIFTMatchFinish = new int[nNumOfProcessors];
	memset(m_bSIFTMatchFinish,0,nNumOfProcessors*sizeof(int));
	
	if( m_pSIFTMatchProcess != NULL )
	{
		delete[] m_pSIFTMatchProcess;
		m_pSIFTMatchProcess= NULL;
	}

	m_pSIFTMatchProcess = new SIFTMatchProcess[ nNumOfProcessors ];
	memset(m_pSIFTMatchProcess,0,nNumOfProcessors*sizeof(SIFTMatchProcess));

	for( i=0; i<nNumOfProcessors; i++)
	{	
		m_pSIFTMatchProcess[i].lSiftPtsNum = lSiftPtsNum;
		m_pSIFTMatchProcess[i].rSiftPtsNum = rSiftPtsNum;
		m_pSIFTMatchProcess[i].lWidth      = lWidth;
		m_pSIFTMatchProcess[i].lHeight     = lHeight;
		m_pSIFTMatchProcess[i].rWidth      = rWidth;
		m_pSIFTMatchProcess[i].rHeight     = rHeight;
		m_pSIFTMatchProcess[i].overlapX    = seedX;
		m_pSIFTMatchProcess[i].overlapY    = seedY;
		m_pSIFTMatchProcess[i].searchX     = searchX ;
		m_pSIFTMatchProcess[i].searchY     = searchY ;

		m_pSIFTMatchProcess[i].CompareStrategy        = CompareStrategy;

		m_pSIFTMatchProcess[i].pLSiftPts        = pLSiftPts;
		m_pSIFTMatchProcess[i].pRSiftPts        = pRSiftPts;
		m_pSIFTMatchProcess[i].rSiftPtMatched   = rSiftPtMatched;
		m_pSIFTMatchProcess[i].nThreadID        = i;
		m_pSIFTMatchProcess[i].nNumOfProcessors = nNumOfProcessors;	
		m_pSIFTMatchProcess[i].bFinish          = m_bSIFTMatchFinish;
		m_pSIFTMatchProcess[i].pMatchedPtID     = pMatchedPtID;
	}
	
	HANDLE *pHandle;
	pHandle = new HANDLE[nNumOfProcessors];
	
	for(i = 0 ; i < nNumOfProcessors; i++)
	{
		pHandle[i] = ::CreateThread(NULL,0,ProcThread_SIFTMatch,(void*)&m_pSIFTMatchProcess[i],0,NULL);
	}
		
	DWORD res = WaitForMultipleObjects(nNumOfProcessors,pHandle,TRUE,INFINITE);

	if(res == WAIT_OBJECT_0)
	{
		for(i=0; i<nNumOfProcessors; i++)
			CloseHandle(pHandle[i]);

		delete m_bSIFTMatchFinish;	m_bSIFTMatchFinish =NULL;
		delete m_pSIFTMatchProcess;	m_pSIFTMatchProcess=NULL;
	}
	
	for(i=0;i<lSiftPtsNum;i+= step)
	{

		if (pMatchedPtID[i] >= 0)
		 {
			lx[matchPtNum]	= pLSiftPts[i].imgX;
			ly[matchPtNum]  = pLSiftPts[i].imgY;

			rx[matchPtNum]  = pRSiftPts[pMatchedPtID[i]].imgX;
			ry[matchPtNum]  = pRSiftPts[pMatchedPtID[i]].imgY;
			
			matchPtNum+=1;
		 }			
	}

	delete rSiftPtMatched;
	delete pMatchedPtID;
	delete pHandle;
}

double CParallelProcessing::DistPtToLine(fPoint pt, double a,double b,double c)
{
	double d;
	if(fabs(a)<1e-3 && fabs(b)<1e-3)
		return (double)0;

	d=fabs(a*pt.x+b*pt.y+c)/sqrt(a*a+b*b);
	return d;
}

void CParallelProcessing::RotateImageWinToHorizon(unsigned char* lpImage,int row,int col,float x,float y,float fRotateAngle,BYTE *pImageStrip,int nImageWinWidth,int nImageWinLength)
{
	int i,j;
	double fGrayLevel;
	fPoint ptInOriImage, ptInHorImage,  ptDisplace;//
  
	ptDisplace.x=x;
	ptDisplace.y=y;

	double cosRotateAngle=cos(fRotateAngle);
	double sinRotateAngle=sin(fRotateAngle);

	for(i= -nImageWinWidth/2; i<= nImageWinWidth/2; i++)
		for(j=-nImageWinLength/2; j<= nImageWinLength/2; j++)
		{//从左下角算起
			ptInHorImage.x=j;
			ptInHorImage.y=i;

			ptInOriImage.x=ptInHorImage.x*cosRotateAngle - ptInHorImage.y*sinRotateAngle + ptDisplace.x;
			ptInOriImage.y=ptInHorImage.x*sinRotateAngle + ptInHorImage.y*cosRotateAngle + ptDisplace.y;

			fGrayLevel=CalcInteriorPixelValue(ptInOriImage,lpImage,row,col);
    		pImageStrip[(i+nImageWinWidth/2)*nImageWinLength+(j+nImageWinLength/2)]=(BYTE)(fGrayLevel+0.5); //[0][0] stores gray value of lower left point
		}
}
void CParallelProcessing::RotateImageWinToHorizon(float xStart,float yStart,float xEnd,float yEnd,unsigned char* lpImage,int row,int col,unsigned char* pImageStrip,int nImageWinWidth,int nImageWinLength)
{
	double fGrayLevel;

	fPoint ptInOriImage, ptInHorImage,  ptDisplace;//
  
	ptDisplace.x=xStart,			ptDisplace.y=yStart;

	double dx=xEnd-xStart;
	double dy=yEnd-yStart;
	double r=sqrt(dx*dx + dy*dy);
	double cosRotateAngle=dx/r;
	double sinRotateAngle=dy/r;
//    TRACE("Angle of processed line: %f\n", 180*acos(cosRotateAngle)/PI);

	int nHalfImageWinWidth=nImageWinWidth/2;
	for(int i=-nHalfImageWinWidth; i<=nHalfImageWinWidth; i++)
		for(int j=0; j<nImageWinLength; j++)
		{//从左下角算起
			ptInHorImage.x=j;
			ptInHorImage.y=i;

			ptInOriImage.x=ptInHorImage.x*cosRotateAngle - ptInHorImage.y*sinRotateAngle + ptDisplace.x;
			ptInOriImage.y=ptInHorImage.x*sinRotateAngle + ptInHorImage.y*cosRotateAngle + ptDisplace.y;

			fGrayLevel=CalcInteriorPixelValue(ptInOriImage,lpImage,row,col);
    		pImageStrip[(i+nHalfImageWinWidth)*nImageWinLength+j]=(BYTE)(fGrayLevel+0.5); //[0][0] stores gray value of lower left point
		}
}

void CParallelProcessing::RotateImageWinToHorizon_MultiThread(unsigned char* lpImage,int row,int col,float x,float y,float fRotateAngle,
															  BYTE *pImageStrip,int nImageWinWidth,int nImageWinLength,int maxThreadNum)
{
	int i=0,j=0;

	if(nImageWinWidth<=0 || nImageWinLength<=0) return;

	//////////////声明硬件信息结构变量////////////////////
	SYSTEM_INFO siSysInfo; 
	//复制硬件信息到SYSTEM_INFO结构变量
	GetSystemInfo(&siSysInfo); 
	int nNumOfProcessors = siSysInfo.dwNumberOfProcessors;
	//////////////////////////////////////////////////////
	
	nNumOfProcessors = min(nNumOfProcessors,maxThreadNum);	//线程个数取CPU核数和限定的最大线程数的小值，避免计算资源全部占用

	m_bRotateImageFinish = new int[nNumOfProcessors];
	memset(m_bRotateImageFinish,0,nNumOfProcessors*sizeof(int));
	
	if( m_pRotateImageProcess != NULL )
	{
		delete[] m_pRotateImageProcess;
		m_pRotateImageProcess = NULL;
	}

	m_pRotateImageProcess = new RotateImageProcess[ nNumOfProcessors ];
	memset(m_pRotateImageProcess,0,nNumOfProcessors*sizeof(RotateImageProcess));

	for( i=0; i<nNumOfProcessors; i++)
	{	
		m_pRotateImageProcess[i].row              = row;
		m_pRotateImageProcess[i].col              = col;
		m_pRotateImageProcess[i].x                = x;
		m_pRotateImageProcess[i].y                = y;
		m_pRotateImageProcess[i].fRotateAngle     = fRotateAngle;
		m_pRotateImageProcess[i].nImageWinWidth   = nImageWinWidth;
		m_pRotateImageProcess[i].nImageWinLength  = nImageWinLength;

		m_pRotateImageProcess[i].lpImage          = lpImage;
		m_pRotateImageProcess[i].pImageStrip      = pImageStrip;

		m_pRotateImageProcess[i].nThreadID        = i;
		m_pRotateImageProcess[i].nNumOfProcessors = nNumOfProcessors;	
		m_pRotateImageProcess[i].bFinish          = m_bRotateImageFinish;
	}
	
	HANDLE *pHandle;
	pHandle = new HANDLE[nNumOfProcessors];
	
	for(i = 0 ; i < nNumOfProcessors; i++)
	{
		pHandle[i] = ::CreateThread(NULL,0,ProcThread_RotateImageWinToHorizon,(void*)&m_pRotateImageProcess[i],0,NULL);
	}
		
	DWORD res = WaitForMultipleObjects(nNumOfProcessors,pHandle,TRUE,INFINITE);

	if(res == WAIT_OBJECT_0)
	{
		for(i=0; i<nNumOfProcessors; i++)
			CloseHandle(pHandle[i]);

		delete m_bRotateImageFinish;	m_bRotateImageFinish =NULL;
		delete m_pRotateImageProcess;	m_pRotateImageProcess=NULL;
	}
	
	delete pHandle;
}



void CParallelProcessing::variance(unsigned char* pg,int col,int length,int width,float n,float* v,float* sg)
//int		col,length,width;
//unsigned char	*pg;
//float		n,*v,*sg;
{	
	register int     i,j;//k
	float		 sgg;
	unsigned char    *pg1,*pg0;
	
	*sg=0.0;   sgg=0.0;
	
	pg0=pg-col;
	for (i=0; i<width; i++) 
	{
		pg0 +=col;  pg1=pg0;
		for (j=0; j<length; j++)
		{ 
			*sg += *pg1; 
			sgg+= *pg1* *pg1++; 
		}
	}
	
	*v=( sgg-(*sg* *sg)/n)/n;
//	*v=( sgg-(*sg* *sg)/n/n);	//yjzhang
}
/****************************************************************************************
input     :  *pg		pointer to image matrix
	     col        	columns number of image
	     width,length  	the window size of matching
	     x_range,y_range 	the searching range
output	     *sg		sum of grey values of the image window
	     *v			variance

****************************************************************************************/
void CParallelProcessing::variance2(unsigned char* pg,int column,int width,int length,float n,int x_range,int y_range,float* v,float* sg)
//	int		column,width,length,x_range,y_range;
//	unsigned char	*pg;
//	float		n,*v,*sg;
{
	register int	i0,i,j;
	int		col,row,dy;
	unsigned char	*pg0,*pg1,*pg2;
	float		*v0,*sg0;
	float		*sgg0,*sgg;
	float		*syc0,*syc1,*syc2,*syc;
	float		*syyc0,*syyc1,*syyc2,*syyc;
	float		*yy0,*yy,*yy1,*yy2;
	float		sy,syy;

	dy=y_range;
	v0=v;
	col= length+x_range-1;		/*** columns of searching window     ***/
	row= width+y_range-1;		/*** rows of searching window        ***/

	yy0=yy=new float[(width+dy-1)*col]; /*** to store pg*pg                  ***/
	sgg0=sgg=new float[x_range*y_range];		/*** sum of pg*pg for window	     ***/
	syc0=syc=new float[col];		/*** sum of grey levels along column ***/
	syyc0=syyc=new float[col];		/*** sum of pg*pg  along column      ***/

	pg0=pg1=pg;
	v0=v;
	sg0=sg;

/*************   calculating   pg*pg   ****************/
	pg0=pg-column;
	for (i=0; i<row; i++)
	{
		pg0 +=column; pg1=pg0;
	   for (j=0; j<col; j++)
			{  *yy++ = ((float) *pg1) * *pg1++;  }
	}

/************** calculating the sum (sy and syy) along the column for first window line ****/
	for (i=0; i<col; i++)
	{
		pg0=pg+i;  yy=yy0+i;
		sy=syy=0.0;
		for (j=0; j<width; j++)
		{
			sy += *pg0;    pg0+=column;
			syy+= *yy;     yy +=col;
		}
		*syc ++ =sy;
		*syyc++ =syy;
	}

/**********************   *v and *sg calculation     ****************************/
    for (i0=0; i0<y_range; i0++)
	{
		syc =  syc0;	syyc= syyc0;
		
		if (i0!=0)   /**** calculating the sum (sy and syy) for other window lines  ***/
		{
			pg1= pg + (i0-1)*column;
			pg2= pg1+ width*column;
			yy1= yy0 +(i0-1)*col;
			yy2= yy1+ width*col;
			
			for (i=0; i<col; i++)
			{
				*syc += ((float) *pg2 - *pg1);   syc++; pg1++; pg2++;
				*syyc+= (*yy2  - *yy1); syyc++;   yy1++; yy2++;
			}
			
		} /**********  if (i0!=0)  *******/
		
		syc1= syc0;
		syc2  =syc1+length;
		syyc1= syyc0;
		syyc2 =syyc1+length;
		
		for (i=0; i<x_range; i++)
		{
			if (i==0)
			{
				sy=syy=0.0;   syc=syc0;   syyc=syyc0;
				for (j=0; j<length; j++)
				{
					sy += *syc++;
					syy+= *syyc++;
				}
				*sg=sy; *sgg=syy;
			}
			else
			{
				*sg = *(sg-1) - *syc1++ + *syc2++;  
				*sgg= *(sgg-1)- *syyc1++ + *syyc2++;
			}
			*v++=( *sgg++ -(*sg* *sg++)/n  )/n;
//			*v++=( *sgg++ -(*sg* *sg++)/n/n  );	//yjzhang
		}
    }   /*  i0  */

	sg=sg0;  sgg=sgg0; v=v0;

	delete sgg0;
	delete syyc0;
	delete syc0;
	delete yy0;
}


/*
 *	单点相关系数匹配（多个候选匹配点）
 */	
	// 搜索范围为正负x_range/2 和 y_range/2，目标窗口大小不含在内，
	// 即若目标窗口大小为5*5，预测的点为 100，100，搜索范围25，11，
	// 则X实际从  100 － 5/2 - 25/2 到 100 + 5/2 + 25/2，Y实际从  100 － 5/2 - 11/2 到 100 + 5/2 + 11/2进行搜索
void CParallelProcessing::correlationCoefficientMatch(unsigned char* pgl,int rowl,int coll,unsigned char* pgr,int rowr,
			 int colr,int xl,int yl,int xr0,int yr0,int width,int length,int x_range,
			 int y_range,int maximum_n,unsigned char* maxn,
			 float* maxd_x,float* maxd_y,unsigned char* maxw,double fRotationAngle,BOOL bEpipolarLineMatch,
			 double epipolarA,double epipolarB,double epipolarC,BOOL sameStrip)
	//signed char	*pgl,*pgr,*maxw,*maxn;
	//int		rowl,coll,rowr,colr,xl,yl,xr0,yr0;
	//int		width,length,x_range,y_range,maximum_n;
	//float		*maxd_x,*maxd_y;
{
	register int	i,j,k,l;
	int		r1,c1,r2,r20,c2,c20,maxp2,maxp,dy,length2,width2;
	int		tpi;
	float		sx1,sx2,sx3,sy1,sy2,sy3;
	float		n,v1,ssg1,*vv2,*vv20,*ssg2,*ssg20;
	float		cv,sxy;
	unsigned char	*ppc,pppc,*ppc0,*ppc1;
	unsigned char	*pgl_low_left,*pgl1,*pgl2,*pgr_low_left,*pgr1,*pgr2,*pgr3;
//	int		local_max,local_max_PC[200],local_max_I[200],local_max_J[200];

	unsigned char *pImageStrip = new unsigned char [width*length];
	n = (float)(width*length);
	length2=length/2;
 	width2=width/2;
	maxp= x_range;
	dy  = y_range;
	maxp2=maxp/2;
	r1= yl-width2; c1 = xl - length2;	/* the left_low corner of the left window */
	if(r1<0) r1 = 0 ;
	if(c1<0)  c1 = 0 ; 
	pgl_low_left= pgl+ r1*coll+c1;

	r20=r2= yr0-width2-dy/2;
	c20=c2= xr0 - length2-maxp2;		/* the left_low corner of the right window */
	if(r2<0)  r2 = 0 ;
	if(c2<0)  c2 = 0 ; 

	ppc0 = new unsigned char[maxp*dy];
	vv2  = new float[maxp*dy];
	ssg2 = new float[maxp*dy];

	ppc   = ppc0;
	vv20  = vv2;
	ssg20 = ssg2;

	pgr_low_left= pgr+ r2*colr+c2;

	if(fabs(fRotationAngle) > 0.10 )	//两张影像间的旋转角大于0.15弧度，即9度以上，需要考虑左影像的重采样
	{
		RotateImageWinToHorizon(pgl,rowl,coll,(float) xl,(float) yl,-(float)fRotationAngle,pImageStrip, length,width);
		pgl_low_left = pImageStrip;
		coll = length;
	}

	fPoint pt;
	double dist=0;
	
	double maxDistance = 5.0;	//核线匹配的约束条件，正负3个像素！！！！！！

	variance (pgl_low_left,coll,length,width,n,&v1,&ssg1);
	variance2(pgr_low_left,colr,width,length,n,x_range,y_range,vv2,ssg2);

	pgr1 = pgr_low_left - colr;
	for (i=0; i<y_range; i++)
	{
		pgr1 +=colr;

		for (j=0; j<x_range; j++)
		{
			cv = 0.0;
			pt.x = c2 + j;
			pt.y = r2 + i;

			if(bEpipolarLineMatch)
				dist = DistPtToLine(pt,epipolarA,epipolarB,epipolarC);

			if(!bEpipolarLineMatch || bEpipolarLineMatch && dist <= maxDistance)	//使用核线约束
			{
				sxy=0;
				pgl1=pgl2=pgl_low_left;
				pgr3=pgr2=pgr1+j;

				for (k=0; k<width; k++)
				{
					for (l=0; l<length; l++)
						sxy+= *pgl2++ * *pgr3++;

					pgl1+=coll;  pgl2=pgl1;
					pgr2+=colr;  pgr3=pgr2;
				}
				cv=(sxy-(ssg1* *ssg2++)/n)/n;
			}
			else 
				ssg2 ++;

			if (v1<1 || *vv2<1 || cv<=0.0)  { pppc = 0;tpi= 0; }
			else
			{
				if(bEpipolarLineMatch)
				{
					if(sameStrip)
					{
						if(dist<=10.0)	//航带内使用核线约束
						{
							tpi=(int)(100.0*cv/sqrt(v1* *vv2));
							pppc=(unsigned char)tpi; 
						}
						else
						{
							tpi=0;
							pppc=0; 
						}
					}
					else
					{
						if(dist<=15.0)	//航带间使用核线约束
						{
							tpi=(int)(100.0*cv/sqrt(v1* *vv2));
							pppc=(unsigned char)tpi; 
						}
						else
						{
							tpi=0;
							pppc=0; 
						}

					}
				}
				else
				{
					tpi=(int)(100.0*cv/sqrt(v1* *vv2));
					pppc=(unsigned char)tpi; 
				}
			}
			*ppc++   = pppc;    vv2++;	
		}

	}
	ppc=ppc0;

	/*******************	searching local maximum of pc  ********************/
	int	 local_max=0;
	int *local_max_PC= new int[maxp*dy];
	int *local_max_I = new int[maxp*dy];
	int *local_max_J = new int[maxp*dy];

	local_max=0;
	ppc = ppc0;
	for (i=1; i<y_range-1; i++)
	{
		ppc+=x_range; ppc1=ppc;
		for (j=1; j<x_range-1; j++)
		{
			ppc1++;
			if 	((*ppc1>20) &&
			 	(*ppc1>  *(ppc1-x_range+1)) &&
			 	(*ppc1>  *(ppc1-x_range))   &&
			 	(*ppc1>  *(ppc1-x_range-1)) &&
			 	(*ppc1>  *(ppc1-1))         &&
			 	(*ppc1>= *(ppc1+x_range-1)) &&
			 	(*ppc1>= *(ppc1+x_range))   &&
			 	(*ppc1>= *(ppc1+x_range+1)) &&
			 	(*ppc1>= *(ppc1+1))){
				local_max_PC[local_max]= *ppc1;
				local_max_I[local_max]=i;
				local_max_J[local_max]=j;
				local_max++;
			}
		}
	}
	/***************   ordering   *******************/
	for (i=0; i<local_max-1; i++){
		for (j=i+1; j<local_max; j++){
			if (local_max_PC[j]> local_max_PC[i])	{
				pppc=(unsigned char)local_max_PC[i]; local_max_PC[i]=local_max_PC[j]; local_max_PC[j]=pppc;
				k=local_max_I[i];   local_max_I[i]=local_max_I[j];    local_max_I[j]=k;
				k=local_max_J[i];   local_max_J[i]=local_max_J[j];    local_max_J[j]=k;
			}
		
		}
	}

	if (local_max>maximum_n)
		*maxn=(unsigned char)maximum_n; 
	else
		*maxn=(unsigned char)local_max;

	/******************   interpolation *************************/
	for (k=0; k< *maxn; k++){
		i= local_max_I[k];
		j= local_max_J[k];
		ppc= ppc0+ i*x_range + j;


		sy3= *(ppc+ x_range);
		sy2= *(ppc);
		sy1= *(ppc- x_range);
		sy2= sy1 + sy3 - 2 * sy2;
	
		sx1= *(ppc-1);
		sx2= *(ppc);
		sx3= *(ppc+1);
		sx2=sx1 + sx3 - 2 * sx2;

		if (sx2!=0.0)
			*(maxd_x+k)= (float)((xr0+j - x_range/2)+ (sx1 - sx3)/sx2/2.0);
		else
			*(maxd_x+k)= (float)(xr0+j-x_range/2);
		if (sy2!=0.0)
			*(maxd_y+k)= (float)((yr0+i-y_range/2)+ (sy1 - sy3)/sy2/2.0);
		else
			*(maxd_y+k)= (float)(yr0+i-y_range/2);
		*(maxw+k)= *ppc;
	}

	delete pImageStrip;
	delete []vv20;
	delete []ssg20;
	delete []ppc0;
	delete local_max_PC;
	delete local_max_I ;
	delete local_max_J ;
}


int CParallelProcessing::HarrisFeaturePointExtractGrid_MultiThread(unsigned char*pImg,		//待提取特征点的影像数据
						int Height,int Width,		//影像高宽
						float *px,float*py,float*attri,//返回特征点坐标，和相应每个点的强度值
						int gridRow,				//格网高度
						int gridCol,				//格网宽度
						char*outputFile,			//输出特征点，及其所在格网信息
						int win,					//局部窗口大小，一般取3即可
						int dg, 					//一阶差分阈值，若小于阈值则不作进一步处理
						bool bExactLocation,       //是否使用forstner定位算子进行精确定位             
						int maxThreadNum)
{
	int num = 0;
	int i=0;
	float x0=0,y0=0;
	float attri0 = 0;
	int numX = Width/gridCol;
	int numY = Height/gridRow;	

	float *px_m  = new float [numX*numY];
	float *py_m  = new float [numX*numY];
	float *at_m  = new float [numX*numY];
	memset(px_m,0,sizeof(float)*numX*numY);
	memset(px_m,0,sizeof(float)*numX*numY);
	memset(at_m,0,sizeof(float)*numX*numY);

	//////////////////
	//////////////声明硬件信息结构变量////////////////////
	SYSTEM_INFO siSysInfo; 
	//复制硬件信息到SYSTEM_INFO结构变量
	GetSystemInfo(&siSysInfo); 
	int nNumOfProcessors = siSysInfo.dwNumberOfProcessors;
	//////////////////////////////////////////////////////
	
	nNumOfProcessors = min(nNumOfProcessors,maxThreadNum);	//线程个数取CPU核数和限定的最大线程数的小值，避免计算资源全部占用

	m_bHarrisFeatureFinish = new int[nNumOfProcessors];
	memset(m_bHarrisFeatureFinish,0,nNumOfProcessors*sizeof(int));
	
	if( m_pHarrisFeatureProcess != NULL )
	{
		delete[] m_pHarrisFeatureProcess;
		m_pHarrisFeatureProcess= NULL;
	}

	m_pHarrisFeatureProcess = new HarrisFeatureProcess[ nNumOfProcessors ];
	memset(m_pHarrisFeatureProcess,0,nNumOfProcessors*sizeof(HarrisFeatureProcess));

	for( i=0; i<nNumOfProcessors; i++)
	{	
		m_pHarrisFeatureProcess[i].pImg        = pImg;
		m_pHarrisFeatureProcess[i].Height      = Height;
		m_pHarrisFeatureProcess[i].Width       = Width;
		m_pHarrisFeatureProcess[i].px          = px_m;
		m_pHarrisFeatureProcess[i].py          = py_m;
		m_pHarrisFeatureProcess[i].attri       = at_m;
		m_pHarrisFeatureProcess[i].gridRow     = gridRow ;
		m_pHarrisFeatureProcess[i].gridCol     = gridCol ;

		m_pHarrisFeatureProcess[i].win         = win;
		m_pHarrisFeatureProcess[i].dg          = dg;
		m_pHarrisFeatureProcess[i].bExactLocation   = bExactLocation;
		m_pHarrisFeatureProcess[i].nThreadID        = i;
		m_pHarrisFeatureProcess[i].nNumOfProcessors = nNumOfProcessors;	
		m_pHarrisFeatureProcess[i].bFinish          = m_bHarrisFeatureFinish;
	}
	
	HANDLE *pHandle;
	pHandle = new HANDLE[nNumOfProcessors];
	
	for(i = 0 ; i < nNumOfProcessors; i++)
	{
		pHandle[i] = ::CreateThread(NULL,0,ProcThread_HarrisFeaturePoint,(void*)&m_pHarrisFeatureProcess[i],0,NULL);
	}
		
	DWORD res = WaitForMultipleObjects(nNumOfProcessors,pHandle,TRUE,INFINITE);

	if(res == WAIT_OBJECT_0)
	{
		for(i=0; i<nNumOfProcessors; i++)
			CloseHandle(pHandle[i]);

		delete m_bHarrisFeatureFinish;	m_bHarrisFeatureFinish =NULL;
		delete m_pHarrisFeatureProcess;	m_pHarrisFeatureProcess=NULL;
	}
	
	for(i=0;i<numX*numY;i++)
	{

		if (at_m[i] > 0)
		 {
			px[num]	= px_m[i];
			py[num] = py_m[i];
			attri[num]  = at_m[i];
			num ++;
		 }			
	}
	delete px_m;
	delete py_m;
	delete at_m;

	return num;
}