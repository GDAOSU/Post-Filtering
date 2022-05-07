/********************************************************************
created:	2016/04/09
author:		Zhang Yanfeng
e-mail:     zhang_yanfeng_3d@foxmail.com
purpose:	This file is for image guided filtering of cost matrix.
*********************************************************************/
#ifndef _PIXELLABIGF_H_
#define _PIXELLABIGF_H_
#include <windows.h>

#ifdef PIXELLABIGF_EXPORTS
#define PIXELLABIGF_API _declspec(dllexport)
#else

// #ifndef _X64
// #define PIXELLABIGF_API _declspec(dllimport)
// #pragma comment(lib,"PixelLabIGF.lib")
// #pragma message("Automatically linking width PixelLabIGF.lib")
// 
// #else

#define PIXELLABIGF_API _declspec(dllimport)
#pragma comment(lib,"PixelLabIGF_x64.lib")
#pragma message("Automatically linking width PixelLabIGF_x64.lib")
//#endif

#endif

/*****************函数说明***************/
//函数名：ImageGuidedFilter
//功能：对输入的代价矩阵进行图像引导滤波
//参数说明：
//costmatrix：输入代价矩阵，三维数组，维度为width*height*numLevel
//width：图像宽度
//height：图像高度
//numLevel：标号数目
//bufImg：引导图像内存
//numBand：引导图像波段数
//p1：滤波窗口大小
//p2：滤波参数
/*****************函数说明***************/
PIXELLABIGF_API void ImageGuidedFilter(float *costmatrix, int width, int height, int numLevel, unsigned char* bufImg, int numBand, float p1 = 5, float p2 = 0.001);

/*****************函数说明***************/
//函数名：ImageGuidedFilter
//功能：对输入的浮点型数据进行图像引导滤波
//参数说明：
//bufData：输入浮点型图像数据
//width：图像宽度
//height：图像高度
//bufImg：引导图像内存
//numBand：引导图像波段数
//p1：滤波窗口大小
//p2：滤波参数
/*****************函数说明***************/
PIXELLABIGF_API void ImageGuidedFilter(float *bufData, int width, int height, unsigned char* bufImg, int numBand, float p1 = 5, float p2 = 0.001);
#endif