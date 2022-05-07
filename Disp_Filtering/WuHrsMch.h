// WuHrsMch.h : main header file for the WUHRSMCH DLL
/*----------------------------------------------------------------------+
|       WuHrsMch                                                        |
|       Author:     DuanYanSong  2015/09/29                             |
|            Ver 1.0                                                    |
|       Copyright (c)2015, WHU RSGIS DPGrid Group                       |
|            All rights reserved.                                       |
|       ysduan@whu.edu.cn; ysduan@sohu.com                              |
+----------------------------------------------------------------------*/
#ifndef WUHRSMCH_H_DUANYANSONG_2015_09_29_17_14_24906
#define WUHRSMCH_H_DUANYANSONG_2015_09_29_17_14_24906

#ifndef WUHRSMCH_LIB

#define WUHRSMCH_LIB  __declspec(dllimport)

/*#ifdef _X64*/
#ifdef _DEBUG_WUHRSMCH
#pragma comment(lib,"WuHrsMch64D.lib") 
#pragma message("Automatically linking with WuHrsMch64D.lib") 
#else
#pragma comment(lib,"WuHrsMch64.lib") 
#pragma message("Automatically linking with WuHrsMch64.lib") 
#endif
// #else
// #ifdef _DEBUG_WUHRSMCH
// #pragma comment(lib,"WuHrsMchD.lib") 
// #pragma message("Automatically linking with WuHrsMchD.lib") 
// #else
// #pragma comment(lib,"WuHrsMch.lib") 
// #pragma message("Automatically linking with WuHrsMch.lib") 
// #endif
// #endif

#endif


#ifndef _SEDP
#define _SEDP
typedef struct tagSEDP
{
    short c,r;    
}SEDP;
#else
#pragma message("WuHrsMch.h, Warning: SEDP alread define, be sure it was define as: struct tagSEDP{ short c,r; }. ") 
#endif

#ifndef _FPT4D
#define _FPT4D
typedef struct tagFPT4D
{
    float xl,yl,xr,yr;
}FPT4D;
#else 
#pragma message("WuHrsMch.h, Warning: FPT4D alread define, be sure it was define as: struct tagFPT4D{ float xl,yl,xr,yr; }.") 
#endif

#ifndef _PYMIMG
#define _PYMIMG
typedef struct tagPYMIMG
{
    BYTE* pImg[8];
    short cols[8],rows[8];
    short pymSum;
}PYMIMG;
#else
#pragma message("WuHrsMch.h, Warning: PYMIMG alread define, be sure it was define as: struct tagPYMIMG{ BYTE *pImg[8];short cols[8],rows[8];short pymSum; }. ") 
#endif

class WUHRSMCH_LIB CWuHrsMch
{
public:
    CWuHrsMch();
    virtual ~CWuHrsMch();
    void    FreePtsMem();

public:
    virtual bool   OnFreeImgMem(BYTE *pImg){ return false; };
    virtual void   OnEnhanceProc( BYTE *pImg,int colsL,int rows ){};
    virtual void   OnSeedPts(const FPT4D* pPts,int ptSum){};
    virtual PYMIMG Pym_Create( BYTE *pImg,int cols,int rows,int miSz=51 );
    virtual void   Pym_Free(PYMIMG pymImg);
    virtual void   print2log(const char *fmt, ...);

public:
    const FPT4D* Hrs_ExtrFeat(  long *ptSum, BYTE *pImg, int cols, int rows,
                                int window = 31, int interval = /*31*/51,
                                int sC = 0, int eC = -1, int sR = 0, int eR = -1);
    // Get Correlation coefficient for two points. 
    int          Hrs_CorrCoef(  BYTE *pImgL, int colsL, int rowsL,
                                BYTE *pImgR, int colsR, int rowsR,
                                int xl, int yl, int xr, int yr, int window);
    // Single point match,Return Correlation coefficient 
    int          Hrs_SPtMatch(  BYTE *pImgL, int colsL, int rowsL,
                                BYTE *pImgR, int colsR, int rowsR,
                                float xl, float yl, float *xr, float *yr);
    // Refine match
    bool         Hrs_RefMatch(  FPT4D* pList,BYTE*pCors, long listSz,
								double r2lx, double r2ly,
                                BYTE *pImgL, int colsL, int rowsL,
                                BYTE *pImgR, int colsR, int rowsR, 
                                int searchRgn = 64, int corf=75, int isFrmCam = 1, int idL = 0, int idR = 0);

    // Expend match
    const FPT4D* Hrs_ExpMatch( BYTE *pImgL,int colsL,int rowsL,
                               BYTE *pImgR,int colsR,int rowsR,
                               BYTE **pCors, long *ptSum, const FPT4D* pSeedPts,
                               int window = 11, int interval = 21, 
                               int searchRgn = 64, int corf = 75, int isFrmCam = 1, int idL = 0, int idR = 0);
    // Dense match
    bool         Den_RefMatch( FPT4D* pList, long listSz,
                               BYTE *pImgL, int colsL, int rowsL,
                               BYTE *pImgR, int colsR, int rowsR, int interval, 
                               int searchRgn = 64, int corf = 75, int isFrmCam = 1, int idL = 0, int idR = 0);
    const FPT4D* Den_HrsMatch( BYTE *pImgL, int colsL, int rowsL,
                               BYTE *pImgR, int colsR, int rowsR,
                               long *ptSum, int interval = 5, int corf = 75, int isFrmCam = 1,
                               float *kap=NULL,float *dx=NULL,float *dy=NULL );
    const FPT4D* Den_EpiMatch( BYTE *pEpiL, int colsL, int rowsL,
                               BYTE *pEpiR, int colsR, int rowsR,
                               long *ptSum, int interval = 5, int corf = 75, float* par6 = NULL);
protected:
    virtual bool        Exp_MatchZ(PYMIMG pymL, PYMIMG pymR, int gc, int gr, FPT4D *pFts, BYTE *pTm, SEDP *pSd, int sdSum, int searchRgn = 64, int micr=74);
    virtual bool        Exp_MatchD(PYMIMG pymL, PYMIMG pymR, int gc, int gr, FPT4D *pFts, BYTE *pTm, SEDP *pSd, int sdSum, int searchRgn = 64, int micr=74);

protected:
    FPT4D*  m_pListPts;
    BYTE*   m_pListCrs;
    long    m_listSizePts;
    UINT    m_extPar[16];    
};

#endif
