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

/*****************����˵��***************/
//��������ImageGuidedFilter
//���ܣ�������Ĵ��۾������ͼ�������˲�
//����˵����
//costmatrix��������۾�����ά���飬ά��Ϊwidth*height*numLevel
//width��ͼ����
//height��ͼ��߶�
//numLevel�������Ŀ
//bufImg������ͼ���ڴ�
//numBand������ͼ�񲨶���
//p1���˲����ڴ�С
//p2���˲�����
/*****************����˵��***************/
PIXELLABIGF_API void ImageGuidedFilter(float *costmatrix, int width, int height, int numLevel, unsigned char* bufImg, int numBand, float p1 = 5, float p2 = 0.001);

/*****************����˵��***************/
//��������ImageGuidedFilter
//���ܣ�������ĸ��������ݽ���ͼ�������˲�
//����˵����
//bufData�����븡����ͼ������
//width��ͼ����
//height��ͼ��߶�
//bufImg������ͼ���ڴ�
//numBand������ͼ�񲨶���
//p1���˲����ڴ�С
//p2���˲�����
/*****************����˵��***************/
PIXELLABIGF_API void ImageGuidedFilter(float *bufData, int width, int height, unsigned char* bufImg, int numBand, float p1 = 5, float p2 = 0.001);
#endif