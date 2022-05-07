#pragma once
#ifndef WALIS_HTYY_H
#define WALIS_HTYY_H

#include <Windows.h>
#include "stdio.h"
#include "math.h"

#ifndef _BND2RGB
#define _BND2RGB

static inline void Bnd2RGB(unsigned short int  *pBuf, int cols, int band)
{
	if (cols <= 0 || band <= 3) return;
/*#ifdef _X64*/
	unsigned short int  *pRGB = pBuf; int i;
	for (i = 0; i<cols; i++, pRGB += 3, pBuf += band){ *((unsigned short int *)pRGB) = *((unsigned short int *)pBuf); *(pRGB + 2) = *(pBuf + 2); }
// #else
// 	_asm{
// 		mov edi, pBuf
// 			mov esi, pBuf
// 			mov edx, cols
// 		_loop :
// 		dec edx
// 			mov eax, dword ptr[esi]
// 			mov byte ptr[edi], al
// 			shr eax, 08h
// 			inc edi
// 			mov word ptr[edi], ax
// 			add esi, band
// 			inc edi
// 			inc edi
// 			cmp edx, 0h
// 			jne _loop
// 	}
// #endif
}
#endif


class CSpWlsImage
{
public:
    virtual int  GetRows()=0;
    virtual int  GetCols()=0;
    virtual int  GetPixelBytes()=0;
	virtual BOOL Read(unsigned short int  *pBuf, int rowIdx) = 0;
    
    virtual void SetRows(int nRows){};
    virtual void SetCols(int nCols){};
    virtual void SetPixelBytes( int nPxlBytes ){};
	virtual BOOL Write(unsigned short int  *pBuf, int rowIdx){ return FALSE; };
};

class CSpWlsFile 
{ 
public:
    CSpWlsFile(){
        m_pGridR0 = NULL; m_pGridR1 = NULL;  
        m_gridRow = m_gridCol = 0;
        
        char iniFN[256]; GetModuleFileName( NULL,iniFN,sizeof(iniFN)); strcpy( strrchr(iniFN,'.'),".ini" );
        m_meanValue   = 300/*float( atof( GetPrivateProfileVal("Wallis","meanV" ,"32767" ,iniFN) ) )*/;
        m_sigmaValue  = 300/*float( atof( GetPrivateProfileVal("Wallis","sigmaV","1000"  ,iniFN) ) )*/;
        m_CValue      = 0.95f/*float( atof( GetPrivateProfileVal("Wallis","C_Val" ,"0.8" ,iniFN) ) )*/;
        m_BValue      = 0.9f/*float( atof( GetPrivateProfileVal("Wallis","B_Val" ,"0.9",iniFN) ) )*/;
        m_gridWZ      = 16/*atoi( GetPrivateProfileVal("Wallis","Grid"  ,"16",iniFN ) )*/;
        m_filterWZ    = 31/*atoi( GetPrivateProfileVal("Wallis","Filter","31",iniFN ) )*/;
    };
    virtual ~CSpWlsFile(){ Reset(); };
    virtual void    Reset(){
        if (m_pGridR0) delete m_pGridR0; m_pGridR0 = NULL;
        if (m_pGridR1) delete m_pGridR1; m_pGridR1 = NULL;
        m_gridRow = m_gridCol = 0;
        
        char iniFN[256]; GetModuleFileName( NULL,iniFN,sizeof(iniFN)); strcpy( strrchr(iniFN,'.'),".ini" );
        m_meanValue   = float( atof( GetPrivateProfileVal("Wallis","meanV" ,"137" ,iniFN) ) );
        m_sigmaValue  = float( atof( GetPrivateProfileVal("Wallis","sigmaV","190"  ,iniFN) ) );
        m_CValue      = float( atof( GetPrivateProfileVal("Wallis","C_Val" ,"0.8" ,iniFN) ) );
        m_BValue      = float( atof( GetPrivateProfileVal("Wallis","B_Val" ,"0.9",iniFN) ) );
        m_gridWZ      = atoi( GetPrivateProfileVal("Wallis","Grid"  ,"16",iniFN ) );
        m_filterWZ    = atoi( GetPrivateProfileVal("Wallis","Filter","31",iniFN ) );        
    };

    BOOL CalWlsPar( CSpWlsImage *pImgFile ){
        
        int colsS=pImgFile->GetCols(),rowsS=pImgFile->GetRows(),pxlBytes=pImgFile->GetPixelBytes();
        int lineSize = colsS*pxlBytes; int cw,rw,br,er,bc,ec,sum;
		lineSize = colsS;

		//compute mean value
		unsigned short int maxvalue = 0;
		double meanvalue = 0;
		unsigned short int *buf = new unsigned short int[colsS];
		int validNum = 0;
		for (int i = 0; i < rowsS;i++)
		{
			pImgFile->Read(buf, i);
			for (int j = 0; j < colsS;j++)
			{
				if (maxvalue<buf[j])
				{
					maxvalue = buf[j];
				}
				if (buf[j] != 0)
				{
					meanvalue += buf[j];
					validNum++;
				}
			}
		}
		delete[]buf; buf = NULL;

		meanvalue /= validNum;
		m_meanValue = 1000/*maxvalue / 2*/;
		m_sigmaValue = m_meanValue;
        
        m_gridRow = rowsS/m_gridWZ +1;
        m_gridCol = colsS/m_gridWZ +1;                        
        m_pGridR0 = new float[ (m_gridRow+1)*(m_gridCol+1) ];
        m_pGridR1 = new float[ (m_gridRow+1)*(m_gridCol+1) ];
        
        //ProgBegin( m_gridRow );int cancel=0; PrintMsg( "Calculate WallisFilter Parameter ..." );

        float meanV=m_meanValue,sigmaV=m_sigmaValue,cV=m_CValue,bV=m_BValue;
        float r0,r1,mean0,sigma0,mean,sigma;
		unsigned short int  *imgBuf = new unsigned short int [lineSize*(1 + m_filterWZ) + 128];
		unsigned short int  *pRow = new unsigned short int [lineSize];
		unsigned short int  *pR, *pC, *pD, *pS;
        for(int vv,i=0; i<m_gridRow; i++){
            br = i*m_gridWZ; er = br+m_filterWZ;
            if ( er>rowsS ){ er = rowsS; br = er-m_filterWZ; }
            for ( int r=br;r<er;r++ ){
                pImgFile->Read( pRow,r );
				//////////////////////////////////////////////////////////////////////////The following code is useless
//                 if ( pxlBytes>1 ){
//                     Bnd2RGB( pRow,colsS,pxlBytes );
// 					for (pD = pRow, pS = pRow, vv = 0; vv<colsS; vv++, pD++, pS += pxlBytes){ *pD = unsigned short int ((UINT(*pS) + *(pS + 1) + *(pS + 2)) / 3); }
//                 }
				//////////////////////////////////////////////////////////////////////////
				memcpy(imgBuf + (r - br)*colsS, pRow, colsS*sizeof(unsigned short int ));
            }
            for (int j=0; j<m_gridCol; j++){
                bc = j*m_gridWZ; ec = bc+m_filterWZ;
                if ( ec>colsS ){ ec = colsS; bc = ec-m_filterWZ;  }
                
                mean0=sigma0=0; sum = m_filterWZ*m_filterWZ;
                for ( rw=br,pR=imgBuf;rw<er;rw++,pR+=colsS ){
                    for( cw=bc,pC=pR+bc;cw<ec;cw++,pC++ ){
                        mean0  += *pC;
                        sigma0 += *pC * *pC;
                    }
                }
                mean = mean0/sum;
                if ( sigma0/sum<=mean*mean ){
                    r1 = 1.0;
                    r0 = bV*meanV+(1.0f-bV-r1)*mean;
                }else{
                    sigma = (float)sqrt( sigma0/sum - mean*mean  );    
                    r1 = cV*sigmaV/(cV*sigma+(1.0f-cV)*sigmaV);
                    r0 = bV*meanV+(1.0f-bV-r1)*mean;
                }
                
                *(m_pGridR0+i*m_gridCol+j) = r0;
                *(m_pGridR1+i*m_gridCol+j) = r1;                                
            }
        }
        delete pRow;         
        delete imgBuf;
        
        return TRUE;
    };
    
	inline void WlsFlt(unsigned short int  *pS, int r, int cols){
        float val,r0,r1;
        for ( int c=0;c<cols;c++,pS++ ){
            InterplotWallisParameter( m_pGridR0,m_pGridR1,m_gridRow,m_gridCol,m_gridWZ,&r0,&r1,c-m_filterWZ/2,r-m_filterWZ/2 );
            
            val = *pS * r1 + r0;
            if (val>32768) val = 65535.f;
            else{ if (val<0) val = 0.f; }
            
			*pS = unsigned short int (val);
        }
    };
    
    float   m_meanValue,m_sigmaValue,m_CValue,m_BValue;
    float   m_rmean,m_rsigma;
    int     m_filterWZ,m_gridWZ;
    int     m_gridRow,m_gridCol;
    float   *m_pGridR0,*m_pGridR1;

private:
    LPCSTR GetPrivateProfileVal(LPCSTR lpAppName,LPCSTR lpKeyName,LPCSTR lpDefault,LPCSTR lpFileName){
        static char strVal[64]; strVal[0]=0;
        ::GetPrivateProfileString( lpAppName,lpKeyName,lpDefault,strVal,64,lpFileName );
        ::WritePrivateProfileString( lpAppName,lpKeyName,strVal,lpFileName );
        return strVal;
    };
    void   InterplotWallisParameter( float *pR0,float *pR1, 
                                       int gridRow, int gridCol,int grdSz,
                                       float *r0,float *r1, int x, int y ){
        if ( x<0 ) x = 0; if ( y<0 ) y = 0;    
        float Z00,Z10,Z01,Z11,*pC;
        int   grid_r = y/grdSz,grid_c = x/grdSz;
        float dx = float( x-grid_c*grdSz )/grdSz;
        float dy = float( y-grid_r*grdSz )/grdSz;
        if (grid_r>=gridRow-1){ grid_r = gridRow-2; dy=0.999f; }
        if (grid_c>=gridCol-1){ grid_c = gridCol-2; dx=0.999f; }
        
        pC  = pR0+grid_r*gridCol+grid_c;
        Z00 = *(pC+0);         Z10 = *(pC+1);
        Z01 = *(pC+gridCol); Z11 = *(pC+gridCol+1);
        *r0 = (1-dx)*(1-dy)*Z00+dx*(1-dy)*Z10+(1-dx)*dy*Z01+dx*dy*Z11;    

        pC  = pR1+grid_r*gridCol+grid_c;
        Z00 = *(pC+0);         Z10 = *(pC+1);
        Z01 = *(pC+gridCol); Z11 = *(pC+gridCol+1);
        *r1 = (1-dx)*(1-dy)*Z00+dx*(1-dy)*Z10+(1-dx)*dy*Z01+dx*dy*Z11;    
    };

public:
    enum OUTMSG{
         PROG_MSG   =   10,
         PROG_START =   11,
         PROG_STEP  =   12,
         PROG_OVER  =   13,
    };
    void SetRevMsgWnd( HWND hWnd,UINT msgID ){   m_hWndRec=hWnd; m_msgID=msgID; };
protected:
    //virtual void ProgBegin(int range)       {if ( ::IsWindow(m_hWndRec) )::SendMessage( m_hWndRec,m_msgID,PROG_START,range );          };
    //virtual void ProgStep(int& cancel)      {if ( ::IsWindow(m_hWndRec) )::SendMessage( m_hWndRec,m_msgID,PROG_STEP ,LONG(&cancel) );  };
    //virtual void ProgEnd()                  {if ( ::IsWindow(m_hWndRec) )::SendMessage( m_hWndRec,m_msgID,PROG_OVER ,0 );              };
    //virtual void PrintMsg(LPCSTR lpstrMsg ) {if ( ::IsWindow(m_hWndRec) )::SendMessage( m_hWndRec,m_msgID,PROG_MSG  ,UINT(lpstrMsg) ); };
private:
    HWND            m_hWndRec;
    UINT            m_msgID;
};

inline BOOL WallisFlt(unsigned short int  *pImg,int colsS,int rowsS)
{
    class CSpVZImageWls : public CSpWlsImage
    {
    public:
        CSpVZImageWls(){m_pImg=NULL;};
        virtual ~CSpVZImageWls(){m_pImg=NULL;};    
		void    Attach(unsigned short int  *pImg, int cols, int rows){ m_pImg = pImg; m_nCols = cols; m_nRows = rows; };
		BOOL    Read(unsigned short int  *pBuf, int rowIdx){
			if (rowIdx<0 || rowIdx >= m_nRows) memset(pBuf, 0, m_nCols*sizeof(unsigned short int ));
            else memcpy( pBuf,m_pImg+rowIdx*m_nCols,m_nCols*sizeof(unsigned short int  ) );
            return m_nCols;
        };
        int  GetRows(){ return m_nRows; };
        int  GetCols(){ return m_nCols; };
        int  GetPixelBytes(){ return 2; };                
    protected:
		unsigned short int   *m_pImg;
        int m_nCols,m_nRows;
    }wlsImage;
	unsigned short int *img_backup = new unsigned short int[colsS*rowsS];
	memcpy(img_backup, pImg, sizeof(unsigned short int)*colsS*rowsS);
	wlsImage.Attach( pImg,colsS,rowsS );
    CSpWlsFile wlsFlt;  if ( !wlsFlt.CalWlsPar(&wlsImage) ) return FALSE;    
    for ( int r=0;r<rowsS;r++ ) wlsFlt.WlsFlt( pImg+r*colsS,r,colsS );     

	int x, y;
	for (y = 0; y < rowsS; y++)
		for (x = 0; x < colsS; x++)
		{
			if (img_backup[y*colsS+x] == 0)
			{
				pImg[y*colsS + x] = 0;
			}
		}

	delete[]img_backup; img_backup = NULL;
    return TRUE;
}

#endif

