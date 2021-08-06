#include "stdafx.h"
#include "BundleAdjust.h"
#include "imagebase.h"
#include "math.h"
#include <direct.h>
#include "walis_HTYY.hpp"
#include "stdio.h"
#include "WuHrsMch.h"
#include <vector>
#include "CVClass.h"
#include "converter_utm_latlon.h"
#include "ellipsoid_utm_info.h"
#include "ImageProcess.h"


#include "Eigen/Dense" 
using namespace Eigen;

#define EPSION 1.0e-20

// #include "../src/SiftGPU/SiftGPU.h"
// #include "../src/SiftGPU/SiftGPU.cpp"
// #include "../include/GL/glew.h"
// #pragma comment(lib, "../lib/siftgpu.lib")

// #define Overlap 3
// #define avgH 100
//#define PI 3.14159265359
// #define OverlapThresh 0.6
// #define tile_size 5000
// #define overlap_size 200
// #define rectification_resolution 0.5
// #define pyrSize 3;
// #define pyrLevel 3
// #define winSize 15
// #define SearchRange_top 20
// #define minGroundHeight -430
// #define maxGroundHeight 5100
// #define EpipolarHeightStep 50
// #define MinMchNum 20

#define EPS 1.0e-12
#define ROTATE(a, i, j, k, l, n) g=a[i*n+j]; h=a[k*n+l]; a[i*n+j] = g-s*(h+g*tau); \
  a[k*n+l] = h+s*(g-h*tau);

CBundleAdjust::CBundleAdjust()
{
	m_avgH = 0;
	m_rectification_GSD = 0.5;
	m_rect_overlap_size = 200;
	m_rect_tile_size = 10000;
	strcpy(m_zoneID, "");
	m_pair_overlap_area = 0.3;
	m_epi_stepH = 50;
	m_Virtual_GCP_Plane_Size = 50;
	m_Virtual_GCP_Height_Size = 20;
}


CBundleAdjust::~CBundleAdjust()
{
}

int CBundleAdjust::BundleAdjustment_Sat(char *input_path, char *output_path)
{
	//Read prj file
	CFileOperation file;
	file.FindDirPath_prj(input_path, m_prj_path, true);
	_mkdir(m_prj_path);

// 	//1. generate image tiles in AOI region
// 	printf("*********************************************************************\n");
// 	printf("*The first step is to cut aoi images so that we do not need to      *\n");
// 	printf("*comput the parts that we do not need.                              *\n");
// 	printf("*********************************************************************\n");
// 	printf("\nGenerating AOI images and rpc files\n");
// 	if (!Generate_AOI_Images(input_path))
// 	{
// 		printf("Cannot generate AOI Images!");
// 		return 0;
// 	}
// 	printf("Finish cutting AOI images\n");
// 
// 	//2. Plane rectification
// 	printf("*********************************************************************\n");
// 	printf("*The second step is image plane rectification so that we can make   *\n");
// 	printf("*resolution and rotation of each image same, which is helpful in    *\n");
// 	printf("*feature matching.                                                  *\n");
// 	printf("*********************************************************************\n");
// 	printf("\nStarting plane rectification\n");
// 	if (!Generate_Plane_Rectification(input_path))
// 	{
// 		printf("Plane rectification failed\n");
// 		return 0;
// 	}
// 	printf("Finish plane rectification\n");
// 
// 	//3. Selecting stereo pair
// 	printf("*********************************************************************\n");
// 	printf("*The third step is pair selection which can help us reduce time     *\n");
// 	printf("*complexity of feature matching.                                    *\n");
// 	printf("*********************************************************************\n");
// 	printf("\nStarting pair selection\n");
// 	if (!Pair_selection(input_path))
// 	{
// 		printf("Pair selection failed\n");
// 		return 0;
// 	}
// 	printf("Finish the pair selection\n");
// 
// 	//4. Feature Matching
// 	printf("*********************************************************************\n");
// 	printf("*The fourth step is feature matching based on epipolar line         *\n");
// 	printf("*constraint.                                                        *\n");
// 	printf("*********************************************************************\n");
// 	printf("\nStarting feature matching\n");
// 	if (!Feature_matching(input_path))
// 	{
// 		printf("Feature matching failed\n");
// 		return 0;
// 	}
// 	printf("Finish the feature matching\n");

	//5. Bundle adjustment
	printf("*********************************************************************\n");
	printf("*The fifth step is bundle adjustment         *\n");
	printf("*********************************************************************\n");
	printf("Start Bundle adjustment procedure\n");
	BundleAdjustment_RPC(input_path);
//  
 	return 1;
}

int CBundleAdjust::Generate_AOI_Images(char *input_path)
{
	CFileOperation file;
	Long_Strip_Img *strips;
	PrjPara m_para;
	CRPC_Fun rpcf;
	file.ReadPrjFile(input_path, m_para);

	int stripNum = 0;
	int error = file.GetAllFilePathsInDir(m_para.img_rpb_folder_path, strips, stripNum);
	printf("Found totally %d Image Strips\n", stripNum);
	if (error == 0) return 0;

	//Read KML, the bounding box of aoi regions
	double lat_kml[4], lon_kml[4], hei_kml[4];
	//int node_num = 4;
	if (strcmp(m_para.AOI_kml_path,"")==0)
	{
		//这块先空着，以后再改进
		printf("Invalid kml path\n");
		return 0;
	}
	else
	{
		if (!file.ReadKml4(m_para.AOI_kml_path, lat_kml, lon_kml, hei_kml))
		{
			printf("KML reading failed!\n");
			for (int i = 0; i < stripNum; i++)
			{
				for (int j = 0; j < strips[i].patchNum; j++)
				{
					delete[]strips[i].img_paths[j];
					delete[]strips[i].rpb_paths[j];
				}
				delete[]strips[i].img_paths;
				delete[]strips[i].rpb_paths;
			}
			delete[]strips; strips = NULL;

			return 0;
		}
	}

	printf("The bounding box of aoi region is:\n");
	printf("Lat\t\tLon\n");
	for (int i = 0; i < 4;i++)
	{
		printf("%lf\t%lf\n", lat_kml[i], lon_kml[i]);
	}

	//Find the path of output aoi images
	char aoi_folder_path[Str_Max_Len] = "";
	char aoi_list_path[Str_Max_Len] = "";

	file.FindAllPath(input_path, "img&rpc", aoi_list_path, aoi_folder_path);

	_mkdir(aoi_folder_path);

	//output aoi images in this folder
	printf("Computing aoi images of each strips\n");
	Long_Strip_Img *strips_aoi = new Long_Strip_Img[stripNum];
	CRPC_Fun pRpc;
	CImageProcess iproc;
	for (int i = 0; i < stripNum;i++)
	{
		printf("Processing %d/%d strips\n", i + 1, stripNum);
		//create folder to store the aoi image of current strips
		strcpy(strips_aoi[i].Root_folder_path, aoi_folder_path);
		char single_strip_name[Str_Max_Len];
		file.GetFileName(strips[i].Root_folder_path, single_strip_name);
		strcat(strips_aoi[i].Root_folder_path, "\\");
		strcat(strips_aoi[i].Root_folder_path, single_strip_name);
		_mkdir(strips_aoi[i].Root_folder_path);

		//check overlap of each patch and aoi regions
		strips_aoi[i].patchNum = 0;
		int min_px, min_py, max_px, max_py;
		int min_px_all,  max_px_all;
		bool ifcover;
		for (int j = 0; j < strips[i].patchNum;j++)
		{
			pRpc.Check_AOI_Overlap(strips[i].img_paths[j], strips[i].rpb_paths[j], lat_kml, lon_kml, hei_kml, 
				m_para.AOI_cutting_extra_size, min_px, min_py, max_px, max_py, ifcover);

			if (ifcover == true)
			{
				if (strips_aoi[i].patchNum==0)
				{
					min_px_all = min_px;
					max_px_all = max_px;
				}
				else
				{
					if (min_px_all > min_px) min_px_all = min_px;
					if (max_px_all < max_px) max_px_all = max_px;
				}
				strips_aoi[i].patchNum++;
			}
		}
		if (strips_aoi[i].patchNum == 0)
		{
			remove(strips_aoi[i].Root_folder_path);
		}
		else
		{
			strips_aoi[i].img_paths = new char *[strips_aoi[i].patchNum];
			strips_aoi[i].rpb_paths = new char *[strips_aoi[i].patchNum];
			for (int j = 0; j < strips[i].patchNum; j++)
			{
				strips_aoi[i].img_paths[j] = new char[Str_Max_Len];
				strips_aoi[i].rpb_paths[j] = new char[Str_Max_Len];
			}

			int cur_patch = 0;
			
			for (int j = 0; j < strips[i].patchNum; j++)
			{
				pRpc.Check_AOI_Overlap(strips[i].img_paths[j], strips[i].rpb_paths[j], lat_kml, lon_kml, hei_kml,
					m_para.AOI_cutting_extra_size, min_px, min_py, max_px, max_py, ifcover);

				if (ifcover == true)
				{
					char img_name[Str_Max_Len];
					char img_path[Str_Max_Len];
					char rpb_path[Str_Max_Len];
					file.GetFileName(strips[i].img_paths[j], img_name);
					strcpy(img_path, strips_aoi[i].Root_folder_path);
					strcat(img_path, "\\");
					strcat(img_path, img_name);
					strcpy(rpb_path, img_path);
					strcat(img_path, "_aoi.TIF");
					strcat(rpb_path, "_aoi.RPB");

					//save aoi image path
					strcpy(strips_aoi[i].img_paths[cur_patch], img_path);
					iproc.SaveAOIFromGivenPath16(strips[i].img_paths[j], img_path, min_px_all, min_py,
						max_px_all - min_px_all + 1, max_py - min_py + 1);

					//save aoi rpb path
					strcpy(strips_aoi[i].rpb_paths[cur_patch], rpb_path);
					pRpc.RPC_update_From_AOI(strips[i].rpb_paths[j], rpb_path, min_px_all, min_py);

					cur_patch++;
				}
			}
		}
	}

	//For the same strip, check whether one patch can be totally covered by the other patch
	//If the current patch is not covered by other pathces, then output it 
	//Write img&rpc list
	printf("Checking the validation of each aoi images\n");
	int new_strip_num = 0;
	for (int i = 0; i < stripNum; i++)
	{
		if (strips_aoi[i].patchNum != 0)
		{
			new_strip_num++;
		}
	}

	FILE *fp = fopen(aoi_list_path, "w");
	fprintf(fp, "StripNum: %d\n", new_strip_num);
	for (int i = 0; i < stripNum;i++)
	{
		if (strips_aoi[i].patchNum != 0)
		{
			//file.AddCharacterToPath(strips_aoi[i].Root_folder_path, '\\');
			fprintf(fp, "folder_path: %s\n", strips_aoi[i].Root_folder_path);
			int patchNum = strips_aoi[i].patchNum;
			int new_patchNum = 0;
			for (int j = 0; j < patchNum; j++)
			{
				bool ifbecovered = false;
				for (int k = 0; k < patchNum; k++)
				{
					if (k != j)
					{
						if (!pRpc.Check_Patch_Coverage_In_SameStrip(strips_aoi[i].img_paths[j], strips_aoi[i].rpb_paths[j],
							strips_aoi[i].img_paths[k], strips_aoi[i].rpb_paths[k]))
						{
							ifbecovered = true;
							break;
						}
					}
				}
				if (ifbecovered == false)
				{
					new_patchNum++;
					/*fprintf(fp, "%s\t%s\n", strips_aoi[i].img_paths[j], strips_aoi[i].rpb_paths[j]);*/
				}
				else
				{
					remove(strips_aoi[i].img_paths[j]);
					remove(strips_aoi[i].rpb_paths[j]);
				}
			}

			fprintf(fp, "PatchNum: %d\n", new_patchNum);
			for (int j = 0; j < patchNum; j++)
			{
				bool ifbecovered = false;
				for (int k = 0; k < patchNum; k++)
				{
					if (k != j)
					{
						if (!pRpc.Check_Patch_Coverage_In_SameStrip(strips_aoi[i].img_paths[j], strips_aoi[i].rpb_paths[j],
							strips_aoi[i].img_paths[k], strips_aoi[i].rpb_paths[k]))
						{
							ifbecovered = true;
							break;
						}
					}
				}
				if (ifbecovered == false)
				{
					fprintf(fp, "imgPath: %s\nrpcPath: %s\n", strips_aoi[i].img_paths[j], strips_aoi[i].rpb_paths[j]);
				}
			}
		}
	}
	fclose(fp); fp = NULL;

	//sort patches
	printf("Sort Strip Patches from Top to Bottom\n");
	Long_Strip_Img *aoi_imgs;
	fp = fopen(aoi_list_path, "r");
	if (fp == NULL)
	{
		printf("aoi images information does not exist\n");
		return 0;
	}
	int num_aoi;
	fscanf(fp, "%*s %d", &num_aoi);
	aoi_imgs = new Long_Strip_Img[num_aoi];

	for (int i = 0; i < num_aoi; i++)
	{
		fscanf(fp, "%*s %s", aoi_imgs[i].Root_folder_path);
		fscanf(fp, "%*s %d", &aoi_imgs[i].patchNum);

		aoi_imgs[i].img_paths = new char *[aoi_imgs[i].patchNum];
		aoi_imgs[i].rpb_paths = new char *[aoi_imgs[i].patchNum];

		for (int j = 0; j < aoi_imgs[i].patchNum; j++)
		{
			aoi_imgs[i].img_paths[j] = new char[MaxLen];
			aoi_imgs[i].rpb_paths[j] = new char[MaxLen];
		}

		for (int j = 0; j < aoi_imgs[i].patchNum; j++)
		{
			fscanf(fp, "%*s %s", aoi_imgs[i].img_paths[j]);
			fscanf(fp, "%*s %s", aoi_imgs[i].rpb_paths[j]);
		}
	}
	fclose(fp); fp = NULL;

	for (int i = 0; i < num_aoi; i++)
	{
		rpcf.SortPatches(aoi_imgs[i]);
	}

	//rewrite file
	fp = fopen(aoi_list_path, "w");
	fprintf(fp, "StripNum: %d\n", num_aoi);
	for (int i = 0; i < stripNum; i++)
	{
		if (aoi_imgs[i].patchNum != 0)
		{
			//file.AddCharacterToPath(strips_aoi[i].Root_folder_path, '\\');
			fprintf(fp, "folder_path: %s\n", aoi_imgs[i].Root_folder_path);
			int patchNum = aoi_imgs[i].patchNum;
			fprintf(fp, "PatchNum: %d\n", patchNum);
			for (int j = 0; j < patchNum; j++)
			{
				fprintf(fp, "imgPath: %s\nrpcPath: %s\n", aoi_imgs[i].img_paths[j], aoi_imgs[i].rpb_paths[j]);
			}
		}
	}
	fclose(fp); fp = NULL;

	//free memory
	for (int i = 0; i < stripNum;i++)
	{
		for (int j = 0; j < strips[i].patchNum;j++)
		{
			delete[]strips[i].img_paths[j];
			delete[]strips[i].rpb_paths[j];
		}
		delete[]strips[i].img_paths; 
		delete[]strips[i].rpb_paths;
	}
	delete[]strips; strips = NULL;

	for (int i = 0; i < stripNum; i++)
	{
		if (strips_aoi[i].patchNum!=0)
		{
			for (int j = 0; j < strips_aoi[i].patchNum; j++)
			{
				delete[]strips_aoi[i].img_paths[j];
				delete[]strips_aoi[i].rpb_paths[j];
			}
			delete[]strips_aoi[i].img_paths;
			delete[]strips_aoi[i].rpb_paths;
		}
	}
	delete[]strips_aoi; strips_aoi = NULL;

	for (int i = 0; i < num_aoi;i++)
	{
		if (aoi_imgs[i].patchNum != 0)
		{
			for (int j = 0; j < aoi_imgs[i].patchNum; j++)
			{
				delete[]aoi_imgs[i].img_paths[j];
				delete[]aoi_imgs[i].rpb_paths[j];
			}
			delete[]aoi_imgs[i].img_paths;
			delete[]aoi_imgs[i].rpb_paths;
		}
	}
	delete[]aoi_imgs; aoi_imgs = NULL;

	return 1;
}

int CBundleAdjust::Generate_Plane_Rectification(char *input_path)
{
	//define class objects
	CImageProcess ipro;
	CFileOperation file;
	CRPC_Fun rpcf;

	PrjPara para;
	file.ReadPrjFile(input_path, para);
	if (para.Plane_rectification_tile_size> 0 )
	{
		m_rect_tile_size = para.Plane_rectification_tile_size;
	}
	if (para.Plane_rectification_overlap_size > 0)
	{
		m_rect_overlap_size = para.Plane_rectification_overlap_size;
	}
	ipro.m_tile_size = m_rect_tile_size;
	ipro.m_overlap_size = m_rect_overlap_size;

	char rectification_folder[MaxLen];
	char rectification_list[MaxLen];
	file.FindAllPath(input_path, "Plane_Rectification", rectification_list, rectification_folder);
	_mkdir(rectification_folder);
	printf("Creating plane rectification folder:   %s\n", rectification_folder);

	//find the path of aoi images
	char aoi_img_folder[MaxLen];
	char aoi_img_list[MaxLen];
	file.FindAllPath(input_path, "img&rpc", aoi_img_list, aoi_img_folder);

	Long_Strip_Img *aoi_imgs;
	FILE *fp = fopen(aoi_img_list, "r");
	if (fp == NULL)
	{
		printf("aoi images information do not exist\n");
		return 0;
	}
	int num_aoi;
	fscanf(fp, "%*s %d", &num_aoi);
	printf("Find %d strips\n", num_aoi);
	aoi_imgs = new Long_Strip_Img[num_aoi];

	for (int i = 0; i < num_aoi;i++)
	{
		fscanf(fp, "%*s %s", aoi_imgs[i].Root_folder_path);
		fscanf(fp, "%*s %d", &aoi_imgs[i].patchNum);

		aoi_imgs[i].img_paths = new char *[aoi_imgs[i].patchNum];
		aoi_imgs[i].rpb_paths = new char *[aoi_imgs[i].patchNum];

		for (int j = 0; j < aoi_imgs[i].patchNum;j++)
		{
			aoi_imgs[i].img_paths[j] = new char[MaxLen];
			aoi_imgs[i].rpb_paths[j] = new char[MaxLen];
		}

		for (int j = 0; j<aoi_imgs[i].patchNum; j++)
		{
			fscanf(fp, "%*s %s", aoi_imgs[i].img_paths[j]);
			fscanf(fp, "%*s %s", aoi_imgs[i].rpb_paths[j]);
		}
	}
	fclose(fp); fp = NULL;

	//create sub_folder for each plane_rectification result
	Long_Strip_Img *rect_imgs = new Long_Strip_Img[num_aoi];
	for (int i = 0; i < num_aoi;i++)
	{
		char filename[MaxLen];
		file.GetFileName(aoi_imgs[i].Root_folder_path, filename);
		strcpy(rect_imgs[i].Root_folder_path, rectification_folder);
		strcat(rect_imgs[i].Root_folder_path, "\\");
		strcat(rect_imgs[i].Root_folder_path, filename);
		_mkdir(rect_imgs[i].Root_folder_path);

		rect_imgs[i].patchNum = aoi_imgs[i].patchNum;
	}

	for (int i = 0; i < num_aoi; i++)
	{
		rect_imgs[i].img_paths = new char *[rect_imgs[i].patchNum];
		rect_imgs[i].rpb_paths = new char *[rect_imgs[i].patchNum];
		rect_imgs[i].refined_rpb_paths = new char *[rect_imgs[i].patchNum];

		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			rect_imgs[i].img_paths[j] = new char[MaxLen];
			rect_imgs[i].rpb_paths[j] = new char[MaxLen];
			rect_imgs[i].refined_rpb_paths[j] = new char[MaxLen];
		}

		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			char filename[MaxLen];
			file.GetFileName(aoi_imgs[i].img_paths[j], filename);
			strcpy(rect_imgs[i].img_paths[j], rect_imgs[i].Root_folder_path);
			strcpy(rect_imgs[i].rpb_paths[j], rect_imgs[i].Root_folder_path);
			strcat(rect_imgs[i].img_paths[j], "\\");
			strcat(rect_imgs[i].rpb_paths[j], "\\");
			strcat(rect_imgs[i].img_paths[j], filename);
			strcat(rect_imgs[i].rpb_paths[j], filename);
			strcat(rect_imgs[i].img_paths[j], "_rec.TIF");
			strcat(rect_imgs[i].rpb_paths[j], "_rec.tfw");
		}
	}

// 	//output the path of each aoi patch
// 	for (int i = 0; i < num_aoi;i++)
// 	{
// 		printf("The information of %d / %d strip\n", i + 1, num_aoi);
// 		printf("Root path: %s\n", rect_imgs[i].Root_folder_path);
// 		printf("PatchNum: %d\n", rect_imgs[i].patchNum);
// 		for (int j = 0; j < rect_imgs[i].patchNum; j++)
// 		{
// 			printf("image path: %s\n", rect_imgs[i].img_paths[j]);
// 			printf("rpb file path: %s\n", rect_imgs[i].rpb_paths[j]);
// 		}
// 	}

	//output the average height and GSD of aoi regions
	m_avgH = 0;
	m_rectification_GSD = 0;
	int num_hei = 0;
	for (int i = 0; i < num_aoi;i++)
	{
		for (int j = 0; j < aoi_imgs[i].patchNum;j++)
		{
			double GSD_tmp, height_tmp;
			char zoneID[10];
			rpcf.RPC_compute_GSD_and_Height(aoi_imgs[i].img_paths[j], aoi_imgs[i].rpb_paths[j], GSD_tmp, height_tmp, zoneID);
			if (i==0 && j==0)
			{
				strcpy(m_zoneID, zoneID);
			}
			m_avgH += height_tmp;
			//m_rectification_GSD += GSD_tmp;
			m_rectification_GSD = max(GSD_tmp, m_rectification_GSD);
			num_hei++;
		}
	}
	m_avgH /= num_hei;
	//m_rectification_GSD /= num_hei;
	ipro.m_avgH = m_avgH;
	ipro.m_rectification_resolution = m_rectification_GSD;
	strcpy(ipro.m_zone, m_zoneID);

	printf("\nThe average height of aoi region is %lf   meters\n", ipro.m_avgH);
	printf("\nThe GSD of plane rectification is %lf   meters\n", ipro.m_rectification_resolution);
	printf("\nThe tile size of plane rectification is %d  pixels\n", ipro.m_tile_size);
	printf("\nZone number of AOI is %s\n", ipro.m_zone);

	//Plane rectification
	int iter = 0;
	for (int i = 0; i < num_aoi;i++)
	{
		for (int j = 0; j < rect_imgs[i].patchNum;j++)
		{
			char single_band_path[MaxLen];
			file.FindDirPath_prj(aoi_imgs[i].img_paths[j], single_band_path, true);
			strcat(single_band_path, "_single.TIF");
			ipro.MultiBands2SingleBand(aoi_imgs[i].img_paths[j], single_band_path);
			ipro.Plane_Rectification(single_band_path, aoi_imgs[i].rpb_paths[j], rect_imgs[i].img_paths[j]); 
			printf("current progress: %lf  %%\n", (double)(iter + 1) / num_hei * 100);
			remove(single_band_path);
			iter++;
		}
	}

	//recompute the rpb file
	printf("Re-generation RPC for plane rectified images\n");
	rpcf.ComputeRPCForPlaneRectification(aoi_imgs, rect_imgs, num_aoi,
		m_Virtual_GCP_Plane_Size, m_Virtual_GCP_Height_Size, m_zoneID, m_avgH);

	//Writing List file
	printf("Writing Plane rectification results list\n");
	fp = fopen(rectification_list, "w");
	fprintf(fp, "Average_height: %lf\n", m_avgH);
	fprintf(fp, "Zone_Number: %s\n", m_zoneID);
	fprintf(fp, "Strip_Num: %d\n", num_aoi);
	for (int i = 0; i < num_aoi;i++)
	{
		fprintf(fp, "Root_folder: %s\n", rect_imgs[i].Root_folder_path);
		fprintf(fp, "Patch_Num: %d\n", rect_imgs[i].patchNum);
		for (int j = 0; j < rect_imgs[i].patchNum;j++)
		{
			fprintf(fp, "Img_Path: %s\n", rect_imgs[i].img_paths[j]);
			fprintf(fp, "tfw_Path: %s\n", rect_imgs[i].rpb_paths[j]);
			fprintf(fp, "rpb_Path: %s\n", rect_imgs[i].refined_rpb_paths[j]);
		}
	}
	fclose(fp);

	//free memory;
	for (int i = 0; i < num_aoi;i++)
	{
		for (int j = 0; j < aoi_imgs[i].patchNum; j++)
		{
			delete[]aoi_imgs[i].img_paths[j];
			delete[]aoi_imgs[i].rpb_paths[j];

			delete[]rect_imgs[i].img_paths[j];
			delete[]rect_imgs[i].rpb_paths[j];
			delete[]rect_imgs[i].refined_rpb_paths[j];
		}
		delete[]aoi_imgs[i].img_paths;
		delete[]aoi_imgs[i].rpb_paths;

		delete[]rect_imgs[i].img_paths;
		delete[]rect_imgs[i].rpb_paths;
		delete[]rect_imgs[i].refined_rpb_paths;
	}
	delete[]aoi_imgs; aoi_imgs = NULL;
	delete[]rect_imgs; rect_imgs = NULL;
	return 1;
}

int CBundleAdjust::Pair_selection(char *input_path)
{
	//define class objects
	CImageProcess ipro;
	CFileOperation file;
	CRPC_Fun rpcf;
	CLL_UTM geo_conv;

	//Read prj file
	PrjPara para;
	file.ReadPrjFile(input_path, para);
	if (para.Pair_overlap_area > 0)
	{
		m_pair_overlap_area = para.Pair_overlap_area;
	}

	printf("Minimum overlap percentage between pairs is: %lf\n", m_pair_overlap_area);

	//Read img&rpc results
	char aoi_img_folder[MaxLen];
	char aoi_img_list[MaxLen];
	file.FindAllPath(input_path, "img&rpc", aoi_img_list, aoi_img_folder);

	Long_Strip_Img *aoi_imgs;
	FILE *fp = fopen(aoi_img_list, "r");
	if (fp == NULL)
	{
		printf("aoi images information do not exist\n");
		return 0;
	}
	int num_aoi;
	fscanf(fp, "%*s %d", &num_aoi);
	printf("Find %d strips\n", num_aoi);
	aoi_imgs = new Long_Strip_Img[num_aoi];

	for (int i = 0; i < num_aoi; i++)
	{
		fscanf(fp, "%*s %s", aoi_imgs[i].Root_folder_path);
		fscanf(fp, "%*s %d", &aoi_imgs[i].patchNum);

		aoi_imgs[i].img_paths = new char *[aoi_imgs[i].patchNum];
		aoi_imgs[i].rpb_paths = new char *[aoi_imgs[i].patchNum];

		for (int j = 0; j < aoi_imgs[i].patchNum; j++)
		{
			aoi_imgs[i].img_paths[j] = new char[MaxLen];
			aoi_imgs[i].rpb_paths[j] = new char[MaxLen];
		}

		for (int j = 0; j < aoi_imgs[i].patchNum; j++)
		{
			fscanf(fp, "%*s %s", aoi_imgs[i].img_paths[j]);
			fscanf(fp, "%*s %s", aoi_imgs[i].rpb_paths[j]);
		}
	}
	fclose(fp); fp = NULL;

	//Read plane rectification results
	char rect_folder[MaxLen];
	char rect_list[MaxLen];
	file.FindAllPath(input_path, "Plane_Rectification", rect_list, rect_folder);

	Long_Strip_Img *rect_imgs;
	fp = fopen(rect_list, "r");
	if (fp == NULL)
	{
		printf("plane rectification information do not exist\n");
		return 0;
	}
	//int num_aoi;
	fscanf(fp, "%*s %lf", &m_avgH);
	fscanf(fp, "%*s %s", m_zoneID);
	fscanf(fp, "%*s %d", &num_aoi);
	printf("Average Height of AOI is: %lf\n", m_avgH);
	printf("Zone number of AOI is: %s\n", m_zoneID);
	printf("Find %d strips\n", num_aoi);
	if (num_aoi<2)
	{
		printf("Lack sufficient pairs\n");
		return 0;
	}

	rect_imgs = new Long_Strip_Img[num_aoi];

	for (int i = 0; i < num_aoi; i++)
	{
		fscanf(fp, "%*s %s", rect_imgs[i].Root_folder_path);
		fscanf(fp, "%*s %d", &rect_imgs[i].patchNum);

		rect_imgs[i].img_paths = new char *[rect_imgs[i].patchNum];
		rect_imgs[i].rpb_paths = new char *[rect_imgs[i].patchNum];
		rect_imgs[i].refined_rpb_paths = new char *[rect_imgs[i].patchNum];

		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			rect_imgs[i].img_paths[j] = new char[MaxLen];
			rect_imgs[i].rpb_paths[j] = new char[MaxLen];
			rect_imgs[i].refined_rpb_paths[j] = new char[MaxLen];
		}

		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			fscanf(fp, "%*s %s", rect_imgs[i].img_paths[j]);
			fscanf(fp, "%*s %s", rect_imgs[i].rpb_paths[j]);
			fscanf(fp, "%*s %s", rect_imgs[i].refined_rpb_paths[j]);
		}
	}
	fclose(fp); fp = NULL;

	//create pair selection path
	char pair_selection_folder[MaxLen];
	char pair_selection_list[MaxLen];
	file.FindAllPath(input_path, "Pair_selection", pair_selection_list, pair_selection_folder);
	printf("The pair selection folder path: %s\n", pair_selection_folder);
	printf("The pair selection information list path: %s\n", pair_selection_list);
	_mkdir(pair_selection_folder);

	int *imw, *imh, *bands, *bits;
	char **imgPaths, **tfwPaths;
	int *stripID;
	char **patchID;
	int total_patch_Num = 0;
	for (int i = 0; i < num_aoi; i++)
	{
		total_patch_Num += rect_imgs[i].patchNum;
	}
	imgPaths = new char *[total_patch_Num];
	tfwPaths = new char *[total_patch_Num];
	patchID = new char *[total_patch_Num];
	stripID = new int[total_patch_Num];
	for (int i = 0; i < total_patch_Num;i++)
	{
		imgPaths[i] = new char[MaxLen];
		tfwPaths[i] = new char[MaxLen];
		patchID[i] = new char[20];
	}
	int iter = 0;
	for (int i = 0; i < num_aoi;i++)
	{
		for (int j = 0; j < rect_imgs[i].patchNum;j++)
		{
			strcpy(imgPaths[iter], rect_imgs[i].img_paths[j]);
			strcpy(tfwPaths[iter], rect_imgs[i].rpb_paths[j]);
			stripID[iter] = i;

			char ID[10];
			sprintf_s(ID, "%d_%d", i, j);
			strcpy(patchID[iter], ID);

			iter++;
		}
	}

	//read size information of each patch
	ipro.ReadImgInfo(imgPaths, total_patch_Num, imw, imh, bands, bits);

// 	//read rpc information
// 	RPC *rpcs;
// 	rpcf.ReadRpcInfo(rpbPaths, total_patch_Num, rpcs);
// 
// 	//Acquire the plane position of corners for each image
// 	if (m_avgH==0) //if same as the default value
// 	{
// 		rpcf.Get_Average_HeightFromRPCs(rpcs, total_patch_Num, m_avgH);
// 	}
// 	printf("The average height of aoi region is %lf\n", m_avgH);
// 	double *height = new double[total_patch_Num];
// 	double *lat_corners, *lon_corners;
// 	for (int i = 0; i < total_patch_Num; i++)
// 	{
// 		height[i] = m_avgH;
// 	}
// 	rpcf.GetCornerPostions(imw, imh, height, rpcs, total_patch_Num, lat_corners, lon_corners);
// 
// 	//check the zone number
// 	if (strcmp(m_zoneID,"")==0)
// 	{
// 		double x, y;
// 		geo_conv.LL2UTM(rpcs[0].latOffset, rpcs[0].longOffset, x, y, m_zoneID);
// 	}
// 	printf("The zone number of aoi region is %s\n", m_zoneID);

	//LL to UTM
	double *ux_corners, *uy_corners;
	ux_corners = new double[total_patch_Num * 4];
	uy_corners = new double[total_patch_Num * 4];

	for (int i = 0; i < total_patch_Num;i++)
	{
		double A, B, C, D, E, F;
		rpcf.ReadTfwFile(tfwPaths[i], A, B, C, D, E, F);
		ux_corners[i * 4 + 0] = A*0 +B*0 +C;
		uy_corners[i * 4 + 0] = D*0 +E*0 +F;

		ux_corners[i * 4 + 1] = A*(imw[i] - 1) + B*0 +C;
		uy_corners[i * 4 + 1] = D*(imw[i] - 1) + E*0 +F;

		ux_corners[i * 4 + 2] = A*(imw[i] - 1) + B*(imh[i] - 1) + C;
		uy_corners[i * 4 + 2] = D*(imw[i] - 1) + E*(imh[i] - 1) + F;

		ux_corners[i * 4 + 3] = A * 0 + B*(imh[i] - 1) + C;
		uy_corners[i * 4 + 3] = D * 0 + E*(imh[i] - 1) + F;
	}

// 	for (int i = 0; i < total_patch_Num;i++)
// 	{
// 		geo_conv.LL2UTM_fixedZone(lat_corners[i * 4 + 0], lon_corners[i * 4 + 0], m_zoneID, ux_corners[i * 4 + 0], uy_corners[i * 4 + 0]);
// 		geo_conv.LL2UTM_fixedZone(lat_corners[i * 4 + 1], lon_corners[i * 4 + 1], m_zoneID, ux_corners[i * 4 + 1], uy_corners[i * 4 + 1]);
// 		geo_conv.LL2UTM_fixedZone(lat_corners[i * 4 + 2], lon_corners[i * 4 + 2], m_zoneID, ux_corners[i * 4 + 2], uy_corners[i * 4 + 2]);
// 		geo_conv.LL2UTM_fixedZone(lat_corners[i * 4 + 3], lon_corners[i * 4 + 3], m_zoneID, ux_corners[i * 4 + 3], uy_corners[i * 4 + 3]);
// 	}

	//compute area of each image
	double *S = NULL;
	S = new double[total_patch_Num];
	for (int i = 0; i < total_patch_Num;i++)
	{
		double tx[4], ty[4];
		tx[0] = ux_corners[i * 4 + 0]; ty[0] = uy_corners[i * 4 + 0];
		tx[1] = ux_corners[i * 4 + 1]; ty[1] = uy_corners[i * 4 + 1];
		tx[2] = ux_corners[i * 4 + 2]; ty[2] = uy_corners[i * 4 + 2];
		tx[3] = ux_corners[i * 4 + 3]; ty[3] = uy_corners[i * 4 + 3];

		rpcf.ComputeImageProjectionArea(tx, ty, S[i]);
	}

	//3.2 compute overlap area and overlap degree
	fp = fopen(pair_selection_list, "w");

	int **tmplist = new int *[total_patch_Num];
	for (int i = 0; i < total_patch_Num;i++)
	{
		tmplist[i] = new int[total_patch_Num - 1];
	}
	int *tmpNum = new int[total_patch_Num];
	memset(tmpNum, 0, sizeof(int)*total_patch_Num);
	for (int i = 0; i < total_patch_Num - 1; i++)
		for (int j = i + 1; j < total_patch_Num; j++)
		{
			if (stripID[i] == stripID[j])
			{
				continue;
			}
			double tS;
			rpcf.ComputeOverlapArea(ux_corners, uy_corners, S, i, j, tS);

			double overlap_degree = tS / min(S[i], S[j]);

			if (overlap_degree>m_pair_overlap_area)
			{
				tmplist[i][tmpNum[i]] = j;
				tmplist[j][tmpNum[j]] = i;
				tmpNum[i]++;
				tmpNum[j]++;

				printf("%s   %s\t%lf  %%\n", patchID[i], patchID[j], overlap_degree*100);

				int stripID_i, stripID_j;
				int pID_i, pID_j;
				sscanf(patchID[i], "%d_%d", &stripID_i, &pID_i);
				sscanf(patchID[j], "%d_%d", &stripID_j, &pID_j);
				fprintf(fp, "%s\t%s\n", patchID[i], patchID[j]);

				fprintf(fp, "%s\n", aoi_imgs[stripID_i].img_paths[pID_i]);
				fprintf(fp, "%s\n", aoi_imgs[stripID_i].rpb_paths[pID_i]);
				fprintf(fp, "%s\n", rect_imgs[stripID_i].img_paths[pID_i]);
				fprintf(fp, "%s\n", rect_imgs[stripID_i].rpb_paths[pID_i]);
				fprintf(fp, "%s\n", rect_imgs[stripID_i].refined_rpb_paths[pID_i]);

				fprintf(fp, "%s\n", aoi_imgs[stripID_j].img_paths[pID_j]);
				fprintf(fp, "%s\n", aoi_imgs[stripID_j].rpb_paths[pID_j]);
				fprintf(fp, "%s\n", rect_imgs[stripID_j].img_paths[pID_j]);
				fprintf(fp, "%s\n", rect_imgs[stripID_j].rpb_paths[pID_j]);
				fprintf(fp, "%s\n", rect_imgs[stripID_j].refined_rpb_paths[pID_j]);
			}
		}

	fclose(fp); fp = NULL;

	//free memory;
	for (int i = 0; i < num_aoi; i++)
	{
		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			delete[]rect_imgs[i].img_paths[j];
			delete[]rect_imgs[i].rpb_paths[j];
			delete[]rect_imgs[i].refined_rpb_paths[j];

			delete[]aoi_imgs[i].img_paths[j];
			delete[]aoi_imgs[i].rpb_paths[j];
		}
		delete[]rect_imgs[i].img_paths;
		delete[]rect_imgs[i].rpb_paths;
		delete[]rect_imgs[i].refined_rpb_paths;

		delete[]aoi_imgs[i].img_paths;
		delete[]aoi_imgs[i].rpb_paths;
	}
	for (int i = 0; i < total_patch_Num;i++)
	{
		delete[]imgPaths[i]; imgPaths[i] = NULL;
		delete[]tfwPaths[i]; tfwPaths[i] = NULL;
		delete[]patchID[i]; patchID[i] = NULL;
		delete[]tmplist[i]; tmplist[i] = NULL;
	}
	delete[]patchID; patchID = NULL;
	delete[]imgPaths; imgPaths = NULL;
	delete[]tfwPaths; tfwPaths = NULL;
	delete[]rect_imgs; rect_imgs = NULL;
	delete[]aoi_imgs; aoi_imgs = NULL;
	delete[]imh; imh = NULL;
	delete[]imw; imw = NULL;
	delete[]bands; bands = NULL;
	delete[]bits; bits = NULL;
	delete[]stripID; stripID = NULL;
	delete[]ux_corners; ux_corners = NULL;
	delete[]uy_corners; uy_corners = NULL;
	delete[]S; S = NULL;
	delete[]tmplist; tmplist = NULL;
	delete[]tmpNum; tmpNum = NULL;

// 	delete[]rpcs; rpcs = NULL;
// 	delete[]height; height = NULL;
// 	delete[]lat_corners; lat_corners = NULL;
// 	delete[]lon_corners; lon_corners = NULL;

	return 1;
}

int CBundleAdjust::Feature_matching(char *input_path)
{
	//define class objects
	CImageProcess ipro;
	CFileOperation file;
	CRPC_Fun rpcf;
	CLL_UTM geo_conv;
	CImageFeatureMatching mch;

	//Read prj file
	PrjPara para;
	file.ReadPrjFile(input_path, para);
	if (para.Plane_rectification_tile_size > 0)
	{
		m_rect_tile_size = para.Plane_rectification_tile_size;
	}
	mch.m_tile_size = m_rect_tile_size;
	printf("The tile size of feature matching is: %d\n", m_rect_tile_size);

	if (para.mch_epi_stepH>0)
	{
		m_epi_stepH = para.mch_epi_stepH;
	}
	mch.m_stepH = m_epi_stepH;

	printf("Read aoi image, plane rectification and pair selection information\n");
	char aoi_folder_path[MaxLen], aoi_list_path[MaxLen];
	char rect_folder_path[MaxLen], rect_list_path[MaxLen];
	char pair_folder_path[MaxLen], pair_list_path[MaxLen];
	char mch_folder_path[MaxLen], mch_list_path[MaxLen];

	file.FindAllPath(input_path, "img&rpc", aoi_list_path, aoi_folder_path);
	file.FindAllPath(input_path, "Plane_Rectification", rect_list_path, rect_folder_path);
	file.FindAllPath(input_path, "Pair_selection", pair_list_path, pair_folder_path);
	file.FindAllPath(input_path, "Feature_Matching", mch_list_path, mch_folder_path);

	_mkdir(mch_folder_path);

	Long_Strip_Img *aoi_imgs;
	int num_aoi;
	FILE *fp = fopen(aoi_list_path, "r");
	if (fp == NULL)
	{
		printf("aoi images information do not exist\n");
		return 0;
	}
	fscanf(fp, "%*s %d", &num_aoi);
	if (num_aoi < 2)
	{
		printf("Lack sufficient images\n");
		return 0;
	}
	aoi_imgs = new Long_Strip_Img[num_aoi];

	for (int i = 0; i < num_aoi; i++)
	{
		fscanf(fp, "%*s %s", aoi_imgs[i].Root_folder_path);
		fscanf(fp, "%*s %d", &aoi_imgs[i].patchNum);

		aoi_imgs[i].img_paths = new char *[aoi_imgs[i].patchNum];
		aoi_imgs[i].rpb_paths = new char *[aoi_imgs[i].patchNum];

		for (int j = 0; j < aoi_imgs[i].patchNum; j++)
		{
			aoi_imgs[i].img_paths[j] = new char[MaxLen];
			aoi_imgs[i].rpb_paths[j] = new char[MaxLen];
		}

		for (int j = 0; j < aoi_imgs[i].patchNum; j++)
		{
			fscanf(fp, "%*s %s", aoi_imgs[i].img_paths[j]);
			fscanf(fp, "%*s %s", aoi_imgs[i].rpb_paths[j]);
		}
	}
	fclose(fp); fp = NULL;

	//Read Plane rectified images
	Long_Strip_Img *rect_imgs;
	fp = fopen(rect_list_path, "r");
	if (fp == NULL)
	{
		printf("plane rectification information do not exist\n");
		return 0;
	}
	//int num_aoi;
	fscanf(fp, "%*s %lf", &m_avgH);
	fscanf(fp, "%*s %s", m_zoneID);
	fscanf(fp, "%*s %d", &num_aoi);

	rect_imgs = new Long_Strip_Img[num_aoi];

	for (int i = 0; i < num_aoi; i++)
	{
		fscanf(fp, "%*s %s", rect_imgs[i].Root_folder_path);
		fscanf(fp, "%*s %d", &rect_imgs[i].patchNum);

		rect_imgs[i].img_paths = new char *[rect_imgs[i].patchNum];
		rect_imgs[i].rpb_paths = new char *[rect_imgs[i].patchNum];
		rect_imgs[i].refined_rpb_paths = new char *[rect_imgs[i].patchNum];

		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			rect_imgs[i].img_paths[j] = new char[MaxLen];
			rect_imgs[i].rpb_paths[j] = new char[MaxLen];
			rect_imgs[i].refined_rpb_paths[j] = new char[MaxLen];
		}

		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			fscanf(fp, "%*s %s", rect_imgs[i].img_paths[j]);
			fscanf(fp, "%*s %s", rect_imgs[i].rpb_paths[j]);
			fscanf(fp, "%*s %s", rect_imgs[i].refined_rpb_paths[j]);
		}
	}
	fclose(fp); fp = NULL;

	//1. Feature detection
	char feature_folder_path[MaxLen];
	strcpy(feature_folder_path, mch_folder_path);
	strcat(feature_folder_path, "\\");
	strcat(feature_folder_path, "Features");
	_mkdir(feature_folder_path);
	strcat(feature_folder_path, "\\");

	int total_patch_num = 0;
	for (int i = 0; i < num_aoi; i++)
		for (int j = 0; j < rect_imgs[i].patchNum; j++)
			total_patch_num++;

	int iter = 0;
	printf("\nStarting Harris Feature Detection...\n");
	char all_feat_pt_list[MaxLen];
	strcpy(all_feat_pt_list, feature_folder_path);
	strcat(all_feat_pt_list, "feat_list.txt");
	fp = fopen(all_feat_pt_list, "w");
	fprintf(fp, "%d\n", total_patch_num);

	for (int i = 0; i < num_aoi; i++)
	{
		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			char cur_path[MaxLen];
			char sub_folder_name[100];
			sprintf_s(sub_folder_name, "%d_%d", i, j);
			strcpy(cur_path, feature_folder_path);
			strcat(cur_path, sub_folder_name);
			strcat(cur_path, "\\");
			_mkdir(cur_path);

			char output_pt_path[MaxLen];
			strcpy(output_pt_path, cur_path);
			strcat(output_pt_path, "featrues.txt");

			int feaNum = 0;
			mch.HarrisFeatureDetection(rect_imgs[i].img_paths[j], output_pt_path, feaNum);

			fprintf(fp, "%s\t%s\n", sub_folder_name, output_pt_path);

			printf("%d_%d\t", i, j);
			printf("Detect %d feature points\t", feaNum);
			printf("current progress... %lf  %%\n", (double)(iter + 1) / total_patch_num * 100);
			iter++;
		}
	}
	fclose(fp); fp = NULL;
	printf("Finish feature detection\n");

	//detect repeated features between patches all_feat_pt_list
	mch.DetectRepeatFeatures(all_feat_pt_list, rect_imgs, num_aoi);

	//////////////////////////////////////////////////////////////////////////
// 	fp = fopen("D:\\OSU期间的文章\\Conferece\\ASPRS\\2019\\first_paper\\sat_list\\Feature_Matching\\pair_matching\\0_0&1_0\\pair_mch_pt.txt", "r");
// 	char rpc_left_path[512] = "D:\\OSU期间的文章\\Conferece\\ASPRS\\2019\\first_paper\\sat_list\\Plane_Rectification\\057772981010\\11OCT22113415-P2AS-057772981010_01_P001_aoi_rec.RPB";
// 	char rpc_right_path[512] = "D:\\OSU期间的文章\\Conferece\\ASPRS\\2019\\first_paper\\sat_list\\Plane_Rectification\\057772981020\\11OCT22113356-P2AS-057772981020_01_P001_aoi_rec.RPB";
// 	char ID_left[100], ID_right[100];
// 	fscanf(fp, "%s %s", ID_left, ID_right);
// 	int Num_correspondence = 0;
// 	fscanf(fp, "%d", &Num_correspondence);
// 	double *ix_left, *iy_left, *ix_right, *iy_right;
// 	ix_left = new double[Num_correspondence];
// 	iy_left = new double[Num_correspondence];
// 	ix_right = new double[Num_correspondence];
// 	iy_right = new double[Num_correspondence];
// 
// 	for (int i = 0; i < Num_correspondence; i++)
// 	{
// 		fscanf(fp, "%lf %lf %lf %lf", &ix_left[i], &iy_left[i], &ix_right[i], &iy_right[i]);
// 	}
// 	fclose(fp); fp = NULL;
// 
// 	RPC rpc_left, rpc_right;
// 	rpcf.ReadRpc_singleFile(rpc_left_path, rpc_left);
// 	rpcf.ReadRpc_singleFile(rpc_right_path, rpc_right);
// 
// 	//Relative orientation
// 	bool *inliers = new bool[Num_correspondence];
// 	rpcf.RelativeOrientation_bias_constant(ix_left, iy_left, ix_right, iy_right, Num_correspondence, rpc_left, rpc_right, inliers);
// 	int outlier_num = 0;
// 	for (int i = 0; i < Num_correspondence; i++)
// 	{
// 		if (inliers[i] == false)
// 		{
// 			outlier_num++;
// 		}
// 	}
// 	printf("Detect %d ourliers\n", outlier_num);
// 	return 1;
	//////////////////////////////////////////////////////////////////////////

	//2. pair matching 
	printf("\nStarting pair matching...\n");
	char pair_matching_path[MaxLen];
	strcpy(pair_matching_path, mch_folder_path);
	strcat(pair_matching_path, "\\");
	strcat(pair_matching_path, "pair_matching");
	_mkdir(pair_matching_path);
	strcat(pair_matching_path, "\\");

	int pairNum = 0;
	file.GetLineNuminFiles(pair_list_path, pairNum);
	if (pairNum % 11 != 0 || pairNum < 11)
	{
		printf("Pair selection information is incorrect\n");
		//free memory
		for (int i = 0; i < num_aoi; i++)
		{
			for (int j = 0; j < rect_imgs[i].patchNum; j++)
			{
				delete[]rect_imgs[i].img_paths[j];
				delete[]rect_imgs[i].rpb_paths[j];

				delete[]aoi_imgs[i].img_paths[j];
				delete[]aoi_imgs[i].rpb_paths[j];
			}
			delete[]rect_imgs[i].img_paths;
			delete[]rect_imgs[i].rpb_paths;

			delete[]aoi_imgs[i].img_paths;
			delete[]aoi_imgs[i].rpb_paths;
		}
		delete[]rect_imgs; rect_imgs = NULL;
		delete[]aoi_imgs; aoi_imgs = NULL;

		return 0;
	}
	else
		pairNum = pairNum / 11;

	//Conclude all information for pair matching
	printf("Found %d pairs for matching\n", pairNum);
	PairInfor *pairs_info = new PairInfor[pairNum];
	fp = fopen(pair_list_path, "r");
	for (int i = 0; i < pairNum;i++)
	{
		fscanf(fp, "%s %s", pairs_info[i].leftID, pairs_info[i].rightID);
		fscanf(fp, "%s", pairs_info[i].left_aoi_img);
		fscanf(fp, "%s", pairs_info[i].left_aoi_rpb);
		fscanf(fp, "%s", pairs_info[i].left_rect_img);
		fscanf(fp, "%s", pairs_info[i].left_rect_tfw);
		fscanf(fp, "%s", pairs_info[i].left_rect_rpb);

		fscanf(fp, "%s", pairs_info[i].right_aoi_img);
		fscanf(fp, "%s", pairs_info[i].right_aoi_rpb);
		fscanf(fp, "%s", pairs_info[i].right_rect_img);
		fscanf(fp, "%s", pairs_info[i].right_rect_tfw);
		fscanf(fp, "%s", pairs_info[i].right_rect_rpb);
	}
	fclose(fp); fp = NULL;

	for (int i = 0; i < pairNum;i++)
	{
		fp = fopen(all_feat_pt_list, "r");
		int ptfileNum = 0;
		fscanf(fp, "%d", &ptfileNum);
		bool ifleft = false, ifright = false;
		for (int j = 0; j < ptfileNum; j++)
		{
			char cur_ID[100], cur_fea_path[MaxLen];
			fscanf(fp, "%s %s", cur_ID, cur_fea_path);
			if (strcmp(pairs_info[i].leftID,cur_ID)==0)
			{
				strcpy(pairs_info[i].left_rect_feat, cur_fea_path);
				ifleft = true;
			}
			if (strcmp(pairs_info[i].rightID, cur_ID) == 0)
			{
				strcpy(pairs_info[i].right_rect_feat, cur_fea_path);
				ifright = true;
			}

			if (ifleft==true && ifright == true)
			{
				break;
			}
		}
		fclose(fp); fp = NULL;

		if (ifleft==false || ifright == false)
		{
			printf("The feature list is inconsistent with pair selection list\n");

			//free memory
			delete[]pairs_info; pairs_info = NULL;
			for (int i = 0; i < num_aoi; i++)
			{
				for (int j = 0; j < rect_imgs[i].patchNum; j++)
				{
					delete[]rect_imgs[i].img_paths[j];
					delete[]rect_imgs[i].rpb_paths[j];

					delete[]aoi_imgs[i].img_paths[j];
					delete[]aoi_imgs[i].rpb_paths[j];
				}
				delete[]rect_imgs[i].img_paths;
				delete[]rect_imgs[i].rpb_paths;

				delete[]aoi_imgs[i].img_paths;
				delete[]aoi_imgs[i].rpb_paths;
			}
			delete[]rect_imgs; rect_imgs = NULL;
			delete[]aoi_imgs; aoi_imgs = NULL;

			return  0;
		}
	}

	//create each sub_folder to save the pair matching results
	char mch_pair_list[MaxLen];
	strcpy(mch_pair_list, pair_matching_path);
	strcat(mch_pair_list, "mch_pair_list.txt");
	fp = fopen(mch_pair_list, "w");

	if (m_avgH == 0)
	{
		FILE *fp_tmp = fopen(rect_list_path, "r");
		fscanf(fp, "%*s %lf", &m_avgH);
		fclose(fp_tmp); fp_tmp = NULL;
	}
	mch.m_avg_height = m_avgH;
	printf("The height plane of rectification result is: %lf  m\n", mch.m_avg_height);
	
	if (strcmp(m_zoneID,"")==0)
	{
		FILE *fp_tmp = fopen(rect_list_path, "r");
		fscanf(fp, "%*s %*lf");
		fscanf(fp, "%*s %s", m_zoneID);
		fclose(fp_tmp); fp_tmp = NULL;
	}
	strcpy(mch.m_zone, m_zoneID);
	printf("The zoneID of AOI region is: %s\n", mch.m_zone);

	if (para.mch_win_size!=0)
	{
		mch.m_mch_win_size = para.mch_win_size;
	}
	printf("The size of matching window is : %d pixels\n", mch.m_mch_win_size);

	if (para.mch_epi_search_range!=0)
	{
		mch.m_epi_search_range = para.mch_epi_search_range;
	}
	printf("The search range along epiplar  line is: %d pixels\n", mch.m_epi_search_range);
	printf("The height step size for epipolar feature matching is: %lf  m\n", mch.m_stepH);

	//Start Pair Matching
	FILE *fp_pair_mch = fopen(mch_pair_list, "w");
	fprintf(fp_pair_mch, "%d\n", pairNum);
	for (int i = 0; i < pairNum;i++)
	{
		char sub_folder_path[MaxLen];
		strcpy(sub_folder_path, pair_matching_path);
		char sub_name[100];
		sprintf_s(sub_name, "%s&%s", pairs_info[i].leftID, pairs_info[i].rightID);
		strcat(sub_folder_path, sub_name);
		_mkdir(sub_folder_path);

		strcpy(pairs_info[i].left_right_corrpendence_path, sub_folder_path);
		strcat(pairs_info[i].left_right_corrpendence_path, "\\");
		strcat(pairs_info[i].left_right_corrpendence_path, "pair_mch_pt.txt");

		//pair matching
		mch.Pair_Matching_Epi(pairs_info[i], fp_pair_mch);

		printf("current progress...  %lf  %%\n", (double)(i + 1) / pairNum * 100);
	}
	fclose(fp); fp = NULL;
	fclose(fp_pair_mch); fp_pair_mch = NULL;

	printf("Finish pair matching...\n");

	printf("Starting mutli-view point tracking\n");

	//3. multi-view Point tracking
	mch.FindCommonPts_Multi_view(pairs_info, pairNum, rect_imgs, num_aoi, mch_list_path);

	//free memory
	delete[]pairs_info; pairs_info = NULL;
	for (int i = 0; i < num_aoi; i++)
	{
		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			delete[]rect_imgs[i].img_paths[j];
			delete[]rect_imgs[i].rpb_paths[j];
			delete[]rect_imgs[i].refined_rpb_paths[j];

			delete[]aoi_imgs[i].img_paths[j];
			delete[]aoi_imgs[i].rpb_paths[j];
		}
		delete[]rect_imgs[i].img_paths;
		delete[]rect_imgs[i].rpb_paths;
		delete[]rect_imgs[i].refined_rpb_paths;

		delete[]aoi_imgs[i].img_paths;
		delete[]aoi_imgs[i].rpb_paths;
	}
	delete[]rect_imgs; rect_imgs = NULL;
	delete[]aoi_imgs; aoi_imgs = NULL;

	return 1;
}

int CBundleAdjust::BundleAdjustment_RPC(char *input_path)
{
	//Define object class
	CFileOperation file;
	CRPC_Fun rpcf;
	FILE *fp;

	//create folder for bundle adjustment;
	char Bundle_adj_folder[MaxLen], Bundle_adj_list[MaxLen];
	file.FindAllPath(input_path, "Bundle_Adjustment", Bundle_adj_list, Bundle_adj_folder);
	_mkdir(Bundle_adj_folder);

	//Read rect image and rpb file
	printf("Read rectified image information\n");
	char rect_folder_path[MaxLen], rect_list_path[MaxLen];
	file.FindAllPath(input_path, "Plane_Rectification", rect_list_path, rect_folder_path);

	Long_Strip_Img *rect_imgs;
	int num_rect;
	fp = fopen(rect_list_path, "r");
	if (fp == NULL)
	{
		printf("rectified images information do not exist\n");
		return 0;
	}
	fscanf(fp, "%*s %*lf");
	fscanf(fp, "%*s %*s");
	fscanf(fp, "%*s %d", &num_rect);
	if (num_rect < 2)
	{
		printf("Lack sufficient images\n");
		return 0;
	}

	printf("Find %d strips\n", num_rect);
	rect_imgs = new Long_Strip_Img[num_rect];

	for (int i = 0; i < num_rect; i++)
	{
		fscanf(fp, "%*s %s", rect_imgs[i].Root_folder_path);
		fscanf(fp, "%*s %d", &rect_imgs[i].patchNum);

		rect_imgs[i].img_paths = new char *[rect_imgs[i].patchNum];
		rect_imgs[i].rpb_paths = new char *[rect_imgs[i].patchNum];
		rect_imgs[i].refined_rpb_paths = new char *[rect_imgs[i].patchNum];
		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			rect_imgs[i].img_paths[j] = new char[MaxLen];
			rect_imgs[i].rpb_paths[j] = new char[MaxLen];
			rect_imgs[i].refined_rpb_paths[j] = new char[MaxLen];
		}

		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			fscanf(fp, "%*s %s", rect_imgs[i].img_paths[j]);
			fscanf(fp, "%*s %*s");
			fscanf(fp, "%*s %s", rect_imgs[i].rpb_paths[j]);
		}
	}
	fclose(fp); fp = NULL;

	//create sub_folders for saving bundle adjustment results
	for (int i = 0; i < num_rect;i++)
	{
		char filename[MaxLen];
		file.GetFileName(rect_imgs[i].Root_folder_path, filename);

		char filepath[MaxLen];
		strcpy(filepath, Bundle_adj_folder);
		strcat(filepath, "\\");
		strcat(filepath, filename);

		_mkdir(filepath);
		strcat(filepath, "\\");

		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			char Bundle_result_name[MaxLen];
			file.GetFileName(rect_imgs[i].img_paths[j], Bundle_result_name);
			strcat(Bundle_result_name, "_bundle.rpb");
			strcpy(rect_imgs[i].refined_rpb_paths[j], filepath);
			strcat(rect_imgs[i].refined_rpb_paths[j], Bundle_result_name);
		}
	}

	//Read multi-view feature points
	char mch_folder_path[MaxLen], mch_list_path[MaxLen];
	file.FindAllPath(input_path, "Feature_Matching", mch_list_path, mch_folder_path);
	fp = fopen(mch_list_path, "r");
	char str[MaxLen];
	fgets(str, MaxLen, fp);
	int ptNum_ground = 0;
	int ptNum_img = 0;
	double Deltax_total = 0, Deltay_total = 0;
	double delta_xy_total = 0;
	while (strcmp(str,"END\n"))
	{
		int single_num = 0;
		sscanf(str, "%*lf %*lf %*lf %d", &single_num);
		for (int i = 0; i < single_num;i++)
		{
			fgets(str, MaxLen, fp);
			double deltax_single, deltay_single;
			sscanf(str, "%*s %*lf %*lf %lf %lf", &deltax_single, &deltay_single);

			Deltax_total += fabs(deltax_single);
			Deltay_total += fabs(deltay_single);
			delta_xy_total += sqrt(deltax_single*deltax_single + deltay_single*deltay_single);
			ptNum_img++;
		}
		ptNum_ground++;
		fgets(str, MaxLen, fp);
	}
	fclose(fp); fp = NULL;

	Deltax_total /= ptNum_img;
	Deltay_total /= ptNum_img;
	delta_xy_total = delta_xy_total/ ptNum_img;

	printf("Totally %d ground points\n", ptNum_ground);
	printf("Totally %d image points\n", ptNum_img);
	printf("Average re-projection errors before bundle adjustment are: avgx = %lf avgy = %lf  avg_xy = %lf\n", Deltax_total, Deltay_total, delta_xy_total);

	//Here, I should give a corase registration among images
	char pair_matching_path[MaxLen];
	strcpy(pair_matching_path, mch_folder_path);
	strcat(pair_matching_path, "\\");
	strcat(pair_matching_path, "pair_matching");
	_mkdir(pair_matching_path);
	strcat(pair_matching_path, "\\");
	char mch_pair_list[MaxLen];
	strcpy(mch_pair_list, pair_matching_path);
	strcat(mch_pair_list, "mch_pair_list.txt");

	//CImageFeatureMatching mch;

	//copy the mch_list_path to Bundle_adj_list
// 	CopyFile(mch_list_path, Bundle_adj_list, FALSE);
// 	for (int i = 0; i < num_rect;i++)
// 	{
// 		int patchNum = rect_imgs[i].patchNum;
// 		for (int j = 0; j < patchNum;j++)
// 		{
// 			CopyFile(rect_imgs[i].rpb_paths[j], rect_imgs[i].refined_rpb_paths[j], FALSE);
// 		}
// 	}

//	return 1;

	//Accurate Bundle adjustment
	fp = fopen(Bundle_adj_list, "r");
	ptNum_ground = 0;
	fgets(str, MaxLen, fp);
	while (strcmp(str, "END\n"))
	{
		int single_num = 0;
		double tLat, tLon, tHei;
		sscanf(str, "%lf %lf %lf %d", &tLat, &tLon, &tHei, &single_num);
		ptNum_ground++;
		for (int i = 0; i < single_num; i++)
		{
			fgets(str, MaxLen, fp);
			char tID[20];
			double tx, ty;
			double deltax_single, deltay_single;
			sscanf(str, "%s %lf %lf %lf %lf", tID, &tx, &ty, &deltax_single, &deltay_single);
		}

		fgets(str, MaxLen, fp);
	}

	rewind(fp);
	Ground_Img_Pt *ground_img_pts = new Ground_Img_Pt[ptNum_ground];
	fgets(str, MaxLen, fp);
	int iter = 0;
	while (strcmp(str, "END\n"))
	{
		int single_num = 0;
		double tLat, tLon, tHei;
		sscanf(str, "%lf %lf %lf %d", &tLat, &tLon, &tHei, &single_num);

		ground_img_pts[iter].Lat = tLat;
		ground_img_pts[iter].Lon = tLon;
		ground_img_pts[iter].Hei = tHei;
		ground_img_pts[iter].ptNum = single_num;
		ground_img_pts[iter].img_pts = new ImgPtInfo[single_num];
		for (int i = 0; i < single_num; i++)
		{
			fgets(str, MaxLen, fp);
			char tID[20];
			double tx, ty;
			double deltax_single, deltay_single;
			sscanf(str, "%s %lf %lf %lf %lf", tID, &tx, &ty, &deltax_single, &deltay_single);

			strcpy(ground_img_pts[iter].img_pts[i].imgID, tID);
			ground_img_pts[iter].img_pts[i].x = tx;
			ground_img_pts[iter].img_pts[i].y = ty;
			ground_img_pts[iter].img_pts[i].deltax = deltax_single;
			ground_img_pts[iter].img_pts[i].deltay = deltay_single;
		}

		iter++;
		fgets(str, MaxLen, fp);
	}
	fclose(fp); fp = NULL;

	//bundle adjustment of RPC model
	rpcf.RPC_BundleAdjustment(ground_img_pts, ptNum_ground, rect_imgs, num_rect, Bundle_adj_list);

	//free memory
	for (int i = 0; i < num_rect; i++)
	{
		for (int j = 0; j < rect_imgs[i].patchNum; j++)
		{
			delete[]rect_imgs[i].img_paths[j];
			delete[]rect_imgs[i].rpb_paths[j];
			delete[]rect_imgs[i].refined_rpb_paths[j];
		}
		delete[]rect_imgs[i].img_paths;
		delete[]rect_imgs[i].rpb_paths;
		delete[]rect_imgs[i].refined_rpb_paths;
	}
	delete[]rect_imgs; rect_imgs = NULL;
	for (int i = 0; i < ptNum_ground;i++)
	{
		delete[]ground_img_pts[i].img_pts; ground_img_pts[i].img_pts = NULL;
	}
	delete[]ground_img_pts; ground_img_pts = NULL;

	return 1;
}

//////////////////////////////////////////////////////////////////////////Using check points to check the acutal accuracy
//Read tracking point result
// 	char chectpointlist[MaxLen] = "D:\\OSU期间的文章\\Conferece\\ASPRS\\2019\\first_paper\\sat_list\\Feature_Matching\\checkpoints.txt";
// 	//	if (1)
// 	{
// 		fp = fopen(chectpointlist, "r");
// 		char str[MaxLen];
// 		fgets(str, MaxLen, fp);
// 		ptNum_ground = 0;
// 		ptNum_img = 0;
// 		double Deltax_total = 0, Deltay_total = 0;
// 		while (strcmp(str, "END\n"))
// 		{
// 			int single_num = 0;
// 			sscanf(str, "%*lf %*lf %*lf %d", &single_num);
// 			for (int i = 0; i < single_num; i++)
// 			{
// 				fgets(str, MaxLen, fp);
// 				double deltax_single, deltay_single;
// 				sscanf(str, "%*s %*lf %*lf %lf %lf", &deltax_single, &deltay_single);
// 
// 				Deltax_total += fabs(deltax_single);
// 				Deltay_total += fabs(deltay_single);
// 				ptNum_img++;
// 			}
// 			ptNum_ground++;
// 			fgets(str, MaxLen, fp);
// 		}
// 		fclose(fp); fp = NULL;
// 
// 		Deltax_total /= ptNum_img;
// 		Deltay_total /= ptNum_img;
// 
// 		Ground_Img_Pt *ground_img_pts = new Ground_Img_Pt[ptNum_ground];
// 		fp = fopen(chectpointlist, "r");
// 		fgets(str, MaxLen, fp);
// 		int iter = 0;
// 		while (strcmp(str, "END\n"))
// 		{
// 			int single_num = 0;
// 			double tLat, tLon, tHei;
// 			sscanf(str, "%lf %lf %lf %d", &tLat, &tLon, &tHei, &single_num);
// 			ground_img_pts[iter].Lat = tLat;
// 			ground_img_pts[iter].Lon = tLon;
// 			ground_img_pts[iter].Hei = tHei;
// 			ground_img_pts[iter].ptNum = single_num;
// 			ground_img_pts[iter].img_pts = new ImgPtInfo[single_num];
// 			for (int i = 0; i < single_num; i++)
// 			{
// 				fgets(str, MaxLen, fp);
// 				char tID[20];
// 				double tx, ty;
// 				double deltax_single, deltay_single;
// 				sscanf(str, "%s %lf %lf %lf %lf", tID, &tx, &ty, &deltax_single, &deltay_single);
// 
// 				strcpy(ground_img_pts[iter].img_pts[i].imgID, tID);
// 				ground_img_pts[iter].img_pts[i].x = tx;
// 				ground_img_pts[iter].img_pts[i].y = ty;
// 				ground_img_pts[iter].img_pts[i].deltax = deltax_single;
// 				ground_img_pts[iter].img_pts[i].deltay = deltay_single;
// 			}
// 
// 			iter++;
// 			fgets(str, MaxLen, fp);
// 		}
// 		fclose(fp); fp = NULL;
// 
// 		double lat, lon, hei;
// 		double *x_tmp;
// 		double *y_tmp;
// 		RPC *rpc_tmp;
// 		double error = 0;
// 		int totalNum = 0;
// 		for (int i = 0; i < ptNum_ground; i++)
// 		{
// 			int ptNum = ground_img_pts[i].ptNum;
// 			x_tmp = new double[ptNum];
// 			y_tmp = new double[ptNum];
// 			rpc_tmp = new RPC[ptNum];
// 
// 			for (int j = 0; j < ptNum; j++)
// 			{
// 				x_tmp[j] = ground_img_pts[i].img_pts[j].x;
// 				y_tmp[j] = ground_img_pts[i].img_pts[j].y;
// 				char *pt_ID = ground_img_pts[i].img_pts[j].imgID;
// 				int pt_stripID, pt_patchID;
// 				sscanf(pt_ID, "%d_%d", &pt_stripID, &pt_patchID);
// 				rpcf.ReadRpc_singleFile(aoi_imgs[pt_stripID].refined_rpb_paths[pt_patchID], rpc_tmp[j]);
// 			}
// 
// 			rpcf.Multi_view_Forward_intersection(x_tmp, y_tmp, rpc_tmp, ptNum, lat, lon, hei);
// 
// 			for (int j = 0; j < ptNum; j++)
// 			{
// 				double reproj_x, reproj_y;
// 				rpcf.RPC_Ground2Image(lat, lon, hei, rpc_tmp[j], reproj_y, reproj_x);
// 
// 				error += sqrt((reproj_y - y_tmp[j])*(reproj_y - y_tmp[j]) + (reproj_x - x_tmp[j])*(reproj_x - x_tmp[j]));
// 				totalNum++;
// 			}
// 			delete[]x_tmp;
// 			delete[]y_tmp;
// 			delete[]rpc_tmp;
// 		}
// 
// 		error /= totalNum;
// 
// 		printf("plane error = %lf pixels\n", error);
// 
// 		return 1;
// 
// 		//
// 		delete[]ground_img_pts; ground_img_pts = NULL;
// 	}

//////////////////////////////////////////////////////////////////////////

// int CBundleAdjust::ReadStereoListInfor(char *Stereo_list_path, MchPtInfor *& mchPairs, int &pairNum)
// {
// 	pairNum = 0;
// 	GetLineNuminFiles(Stereo_list_path, pairNum);
// 	mchPairs = new MchPtInfor[pairNum];
// 
// 	FILE *fp_list = fopen(Stereo_list_path, "r");
// 	int i, j;
// 	for (i = 0; i < pairNum;i++)
// 	{
// 		char single_MchPt_Path[MaxLen];
// 		fscanf(fp_list, "%s", single_MchPt_Path);
// 		char ID_names[MaxLen];
// 		GetFileName(single_MchPt_Path, ID_names);
// 		int ID1, ID2;
// 		sscanf(ID_names, "%d_%d", &ID1, &ID2);
// 		int ptNum = 0;
// 		GetLineNuminFiles(single_MchPt_Path, ptNum);
// 
// 		mchPairs[i].ID1 = ID1;
// 		mchPairs[i].ID2 = ID2;
// 		mchPairs[i].ptNum = ptNum;
// 
// 		mchPairs[i].corresList = new CorrespondencePts[ptNum];
// 
// 		FILE *fp_corres = fopen(single_MchPt_Path, "r");
// 		for (j = 0; j < ptNum;j++)
// 		{
// 			fscanf(fp_corres, "%lf %lf %lf %lf", &mchPairs[i].corresList[j].x1, &mchPairs[i].corresList[j].y1,
// 				&mchPairs[i].corresList[j].x2, &mchPairs[i].corresList[j].y2);
// 		}
// 		fclose(fp_corres); fp_corres = NULL;
// 	}
// 	fclose(fp_list); fp_list = NULL;
// 	return 1;
// }
// 

// /*
// 
// 
// 
// //2
// maxd = 0;
// avgd = 0;
// double alpha2 = 0.1*alpha;
// LeastSquares_adapt(A, B, X, 2 * ptNum, 78, alpha2);
// 
// for (int i = 0; i < 20; i++)
// {
// rpc_ridge[1].lineNumCoef[i] = X[i];
// }
// 
// rpc_ridge[1].lineDenCoef[0] = 1;
// for (int i = 1; i < 20; i++)
// {
// rpc_ridge[1].lineDenCoef[i] = X[i + 19];
// }
// 
// for (int i = 0; i < 20; i++)
// {
// rpc_ridge[1].sampNumCoef[i] = X[i + 39];
// }
// 
// rpc_ridge[1].sampDenCoef[0] = 1;
// for (int i = 1; i < 20; i++)
// {
// rpc_ridge[1].sampDenCoef[i] = X[i + 58];//59
// }
// 
// for (int i = 0; i < ptNum; i++)
// {
// double tlat, tlon, thei;
// tlat = lat[i];
// tlon = lon[i];
// thei = hei[i];
// 
// double tx, ty;
// RPC_Ground2Image(tlat, tlon, thei, rpc_ridge[1], ty, tx);
// 
// double td;
// td = sqrt((tx - ix[i])*(tx - ix[i]) + (ty - iy[i])*(ty - iy[i]));
// 
// if (td > maxd)
// maxd = td;
// 
// avgd += td;
// }
// avgd /= ptNum;
// 
// max_dis[1] = maxd;
// 
// //3
// maxd = 0;
// avgd = 0;
// double alpha3 = 0.01 * alpha;
// LeastSquares_adapt(A, B, X, 2 * ptNum, 78, alpha3);
// 
// for (int i = 0; i < 20; i++)
// {
// rpc_ridge[2].lineNumCoef[i] = X[i];
// }
// 
// rpc_ridge[2].lineDenCoef[0] = 1;
// for (int i = 1; i < 20; i++)
// {
// rpc_ridge[2].lineDenCoef[i] = X[i + 19];
// }
// 
// for (int i = 0; i < 20; i++)
// {
// rpc_ridge[2].sampNumCoef[i] = X[i + 39];
// }
// 
// rpc_ridge[2].sampDenCoef[0] = 1;
// for (int i = 1; i < 20; i++)
// {
// rpc_ridge[2].sampDenCoef[i] = X[i + 58];//59
// }
// 
// for (int i = 0; i < ptNum; i++)
// {
// double tlat, tlon, thei;
// tlat = lat[i];
// tlon = lon[i];
// thei = hei[i];
// 
// double tx, ty;
// RPC_Ground2Image(tlat, tlon, thei, rpc_ridge[2], ty, tx);
// 
// double td;
// td = sqrt((tx - ix[i])*(tx - ix[i]) + (ty - iy[i])*(ty - iy[i]));
// 
// if (td > maxd)
// maxd = td;
// 
// avgd += td;
// }
// avgd /= ptNum;
// 
// max_dis[2] = maxd;
// 
// //4
// maxd = 0;
// avgd = 0;
// double alpha4 = 10 * alpha;
// LeastSquares_adapt(A, B, X, 2 * ptNum, 78, alpha4);
// 
// for (int i = 0; i < 20; i++)
// {
// rpc_ridge[3].lineNumCoef[i] = X[i];
// }
// 
// rpc_ridge[3].lineDenCoef[0] = 1;
// for (int i = 1; i < 20; i++)
// {
// rpc_ridge[3].lineDenCoef[i] = X[i + 19];
// }
// 
// for (int i = 0; i < 20; i++)
// {
// rpc_ridge[3].sampNumCoef[i] = X[i + 39];
// }
// 
// rpc_ridge[3].sampDenCoef[0] = 1;
// for (int i = 1; i < 20; i++)
// {
// rpc_ridge[3].sampDenCoef[i] = X[i + 58];//59
// }
// 
// for (int i = 0; i < ptNum; i++)
// {
// double tlat, tlon, thei;
// tlat = lat[i];
// tlon = lon[i];
// thei = hei[i];
// 
// double tx, ty;
// RPC_Ground2Image(tlat, tlon, thei, rpc_ridge[3], ty, tx);
// 
// double td;
// td = sqrt((tx - ix[i])*(tx - ix[i]) + (ty - iy[i])*(ty - iy[i]));
// 
// if (td > maxd)
// maxd = td;
// 
// avgd += td;
// }
// avgd /= ptNum;
// 
// max_dis[3] = maxd;
// */
// 
// //recompute alpha
// // 		double XTX[1];
// // 		double *XT = new double[78];
// // 		memcpy(XT, X, sizeof(double)*78);
// // 		MultiMatrix(XT, X, XTX, 1, 1, 78);
// // 
// // 		double s = 0;
// // 		s /= ptNum;
// // 		alpha = 78 * s / XTX[0];
// // 
// // 		delete[]XT; XT = NULL;
// 
// int CBundleAdjust::ComputeSystemErrorBetweenTiles(int sx1, int sy1, int w1, int h1,
// 	int sx2, int sy2, int w2, int h2,
// 	RPC rpc1, RPC rpc2, double tx, double ty, 
// 	double &ax1, double &bx1, double &cx1, double &ay1, double &by1, double &cy1)
// {
// 	ax1 = 0; bx1 = 0; cx1 = 0;
// 	ay1 = 0; by1 = 0; cy1 = 0;
// 
// 	int x, y;
// 	double avgHei = 0;
// 	int num = 0;
// 	int step = 100;
// 
// 	int stepx, stepy;
// 	stepx = w1 / step;
// 	stepy = h1 / step;
// 	vector<double> txs, tys;
// 	vector<double> ys;
// 
// 	if (stepx > 100) stepx = 100;
// 	if (stepy > 100) stepy = 100;
// 
// 	for (y = 0; y < h2; y += stepy)
// 		for (x = 0; x < w2; x += stepx)
// 		{
// 			double lat, lon;
// 			double px, py;
// 			double px2, py2;
// 			px2 = sx2 + x; py2 = sy2 + y;
// 
// 			RPC_Image2Ground(py2, px2, avgHei, rpc2, lat, lon);
// 			RPC_Ground2Image(lat, lon, avgHei, rpc1, py, px);
// 
// 			if (px >= sx1 && px <= sx1 + w1 - 1 && py >= sy1 && py <= sy1 + h1 - 1)
// 			{
// 				txs.push_back(px - px2);
// 				tys.push_back(py - py2);
// 				ys.push_back(y);
// 				num++;
// 			}
// 		}
// 
// 	for (int i = 0; i < num;i++)
// 	{
// 		tys[i] -= (int)(ty + 0.5);
// 	}
// 
// 	FILE *fp = fopen("D:\\out.txt", "w");
// 	for (int i = 0; i < num;i++)
// 	{
// 		fprintf(fp, "%lf\t%lf\t%lf\n", ys[i], txs[i], tys[i]);
// 	}
// 	fclose(fp); fp = NULL;
// 
// 	//compute the accurate translate value
// 	double tx_acc = 0, ty_acc = 0;
// 	for (int i = 0; i < num; i++)
// 	{
// 		tx_acc += fabs(txs[i]);
// 		ty_acc += fabs(tys[i]);
// 	}
// 	tx_acc /= num;
// 	ty_acc /= num;
// 
// 	//////////////////////////////////////////////////////////////////////////temp code
// // 	cx1 = 0; cy1 = 0;
// // 	for (int i = 0; i < num; i++)
// // 	{
// // 		cx1 += txs[i];
// // 		cy1 += tys[i];
// // 	}
// // 	cx1 /= num;
// // 	cy1 /= num;
// // 
// // 	txs.clear();
// // 	tys.clear();
// // 	ys.clear();
// // 
// // 	vector<double>().swap(txs);
// // 	vector<double>().swap(tys);
// // 	vector<double>().swap(ys);
// // 
// // 	return 1;
// 	//////////////////////////////////////////////////////////////////////////
// 
// 	double *A = new double[num * 3];
// 	double *L = new double[num];
// 	double X[3];
// 	double a0, b0, c0;
// 
// 	//firstly compute the coefficient of x 
// 	//compute the intial value
// 	a0 = 0;
// 	for (int i = 0; i < num; i++)
// 	{
// 		if (fabs(a0) < fabs(txs[i]))
// 		{ 
// 			a0 = txs[i];
// 		}
// 	}
// 
// 	c0 = 0;
// 	double num_c = 0;
// 	for (int i = 0; i < num; i++)
// 	{
// 		if (ys[i]==0)
// 		{
// 			c0 += asin(txs[i] / a0);
// 			num_c++;
// 		}
// 	}
// 	c0 /= num_c;
// 
// // 	if (txs[num - 1] * txs[0]<0)
// // 	{
// // 		b0 = fabs(2 * PI + asin(txs[num - 1] / a0) - asin(txs[0] / a0)) / (ys[num - 1] - ys[0]);
// // 	}
// // 	else
// // 	{
// // 		b0 = fabs(asin(txs[num - 1] / a0) - asin(txs[0] / a0)) / (ys[num - 1] - ys[0]);
// // 	}
// 
// 	double b0_try[4];
// 	double dis_b[4];
// 	b0_try[0] = fabs(asin(txs[num - 1] / a0) - asin(txs[0] / a0)) / (ys[num - 1] - ys[0]);
// 	b0_try[1] = fabs(PI - asin(txs[num - 1] / a0) - asin(txs[0] / a0)) / (ys[num - 1] - ys[0]);
// 	b0_try[2] = fabs(asin(txs[num - 1] / a0) - (2 * PI + asin(txs[0] / a0))) / (ys[num - 1] - ys[0]);
// 	b0_try[3] = fabs(2 * PI + asin(txs[num - 1] / a0) - asin(txs[0] / a0)) / (ys[num - 1] - ys[0]);
// 
// 	for (int k = 0; k < 4;k++)
// 	{
// 		dis_b[k] = 0;
// 		for (int i = 0; i < num; i++)
// 		{
// 			double deltax = a0*sin(b0_try[k] * ys[i] + c0);
// 			dis_b[k] += fabs(txs[i] - deltax);
// 		}
// 		dis_b[k] /= num;
// 	}
// 
// 	b0 = b0_try[0];
// 	double best_dis = dis_b[0];
// 	for (int i = 1; i < 4;i++)
// 	{
// 		if (best_dis>dis_b[i])
// 		{
// 			best_dis = dis_b[i];
// 			b0 = b0_try[i];
// 		}
// 	}
// 
// 	int Iter = 0;
// 	while (tx_acc>0.001 && Iter<10)
// 	{
// 		for (int i = 0; i < num;i++)
// 		{
// 			A[i * 3 + 0] = sin(b0*ys[i] + c0);
// 			A[i * 3 + 1] = cos(b0*ys[i] + c0)*a0*ys[i];
// 			A[i * 3 + 2] = cos(b0*ys[i] + c0)*a0;
// 
// 			L[i] = txs[i] - (a0*sin(b0*ys[i] + c0));
// 		}
// 
// 		double alpha = 0;
// 		LeastSquares(A, L, X, num, 3, alpha);
// 
// 		a0 += X[0];
// 		b0 += X[1];
// 		c0 += X[2];
// 
// 		tx_acc = 0;
// 		for (int i = 0; i < num; i++)
// 		{
// 			double deltax = a0*sin(b0*ys[i] + c0);
// 			tx_acc += fabs(txs[i] - deltax);
// 		}
// 		tx_acc /= num;
// 
// 		Iter++;
// 
// 		ax1 = a0; bx1 = b0; cx1 = c0;
// 	}
// 
// 	//secondly, compute the coefficient of y
// 	//compute the intial value
// 	a0 = 0;
// 	for (int i = 0; i < num; i++)
// 	{
// 		if (fabs(a0) < fabs(tys[i]))
// 		{
// 			a0 = tys[i];
// 		}
// 	}
// 
// 	c0 = 0;
// 	num_c = 0;
// 	for (int i = 0; i < num; i++)
// 	{
// 		if (ys[i] == 0)
// 		{
// 			c0 += asin(tys[i] / a0);
// 			num_c++;
// 		}
// 	}
// 	c0 /= num_c;
// 
// // 	if (tys[num - 1] * tys[0] < 0)
// // 	{
// // 		b0 = fabs(2 * PI + asin(tys[num - 1] / a0) - asin(tys[0] / a0)) / (ys[num - 1] - ys[0]);
// // 	}
// // 	else
// // 	{
// // 		b0 = fabs(asin(tys[num - 1] / a0) - asin(tys[0] / a0)) / (ys[num - 1] - ys[0]);
// // 	}
// 
// 	b0_try[0] = fabs(asin(tys[num - 1] / a0) - asin(tys[0] / a0)) / (ys[num - 1] - ys[0]);
// 	b0_try[1] = fabs(PI - asin(tys[num - 1] / a0) - asin(tys[0] / a0)) / (ys[num - 1] - ys[0]);
// 	b0_try[2] = fabs(asin(tys[num - 1] / a0) - (2*PI + asin(tys[0] / a0))) / (ys[num - 1] - ys[0]);
// 	b0_try[3] = fabs(2*PI + asin(tys[num - 1] / a0) - asin(tys[0] / a0)) / (ys[num - 1] - ys[0]);
// 
// 	for (int k = 0; k < 4; k++)
// 	{
// 		dis_b[k] = 0;
// 		for (int i = 0; i < num; i++)
// 		{
// 			double deltay = a0*sin(b0_try[k] * ys[i] + c0);
// 			dis_b[k] += fabs(tys[i] - deltay);
// 		}
// 		dis_b[k] /= num;
// 	}
// 
// 	b0 = b0_try[0];
// 	best_dis = dis_b[0];
// 	for (int i = 1; i < 4; i++)
// 	{
// 		if (best_dis > dis_b[i])
// 		{
// 			best_dis = dis_b[i];
// 			b0 = b0_try[i];
// 		}
// 	}
// 
// 	Iter = 0;
// 	while (ty_acc > 0.001 && Iter < 10)
// 	{
// 		for (int i = 0; i < num; i++)
// 		{
// 			A[i * 3 + 0] = sin(b0*ys[i] + c0);
// 			A[i * 3 + 1] = cos(b0*ys[i] + c0)*a0*ys[i];
// 			A[i * 3 + 2] = cos(b0*ys[i] + c0)*a0;
// 
// 			L[i] = tys[i] - (a0*sin(b0*ys[i] + c0));
// 		}
// 
// 		double alpha = 0;
// 		LeastSquares(A, L, X, num, 3, alpha);
// 
// 		a0 += X[0];
// 		b0 += X[1];
// 		c0 += X[2];
// 
// 		ty_acc = 0;
// 		for (int i = 0; i < num; i++)
// 		{
// 			double deltay = a0*sin(b0*ys[i] + c0);
// 			ty_acc += fabs(tys[i] - deltay);
// 		}
// 		ty_acc /= num;
// 
// 		Iter++;
// 
// 		ay1 = a0; by1 = b0; cy1 = c0;
// 	}
// 
// 
// 	//free memory
// 	txs.clear();
// 	tys.clear();
// 	ys.clear();
// 
// 	vector<double>().swap(txs);
// 	vector<double>().swap(tys);
// 	vector<double>().swap(ys);
// 
// 	delete[]A; A = NULL;
// 	delete[]L; L = NULL;
// 
// 	return 1;
// }
// 
// int CBundleAdjust::ComputeCoordinateBeforeCorrection(double x_acc, double y_acc, double ax1, double bx1, double cx1,
// 	double ay1, double by1, double cy1, double &x, double &y, double &dx, double &dy)
// {
// 	double A[2*2] = { 0 }, L[2] = { 0 };
// 	double X[2];
// 
// 	double x_y = 1;
// 	double x_x = 1;
// 	y = y_acc;
// 	x = x_acc;
// 
// 	MatrixXd Ge(2, 2);
// 	VectorXd He(2);
// 	int I = 0;
// 	while (fabs(x_y)>0.0001 || fabs(x_x) > 0.0001)
// 	{
// 		A[0] = 1 + ay1*cos(by1*y + cy1)*by1;
// 		A[1] = 0;
// 
// 		A[2] = ax1*cos(bx1*y + cx1)*bx1;
// 		A[3] = 1;
// 
// 		L[0] = y_acc - y - (ay1*sin(by1*y + cy1));
// 		L[1] = x_acc - x - (ax1*sin(bx1*y + cx1));
// 		
// 		for (int i = 0; i < 2; i++)
// 			for (int j = 0; j < 2; j++)
// 			{
// 				Ge(i, j) = A[i*2 + j];
// 			}
// 		for (int i = 0; i < 2; i++)
// 		{
// 			He(i) = L[i];
// 		}
// 		VectorXd Xe = Ge.fullPivLu().solve(He);
// 		for (int i = 0; i < 2; i++)
// 		{
// 			X[i] = Xe(i);
// 		}
// 
// 		x_y = X[0];
// 		x_x = X[1];
// 
// 		y += x_y;
// 		x += x_x;
// 
// 		if (I>10)
// 		{
// 			break;
// 		}
// 		I++;
// 	}
// 
// 	dx = ax1*sin(bx1*y + cx1);
// 	dy = ay1*sin(by1*y + cy1);
// 
// 	//Finally, compute dx and y
// 	return 1;
// }
// int CBundleAdjust::Compute_RPC_Merge_Img_Tiles(char **imgPaths, char **rpcPaths, int coverNum, int *sx, int *sy, 
// 	int *imgw, int *imgh, int *imgID, int origin_ID,
// 	char *merge_result_path, char *merge_rpc_path)
// {
// 
// 	return 1;
// }
// 
// int CBundleAdjust::ReadKML(char *kmlPath, double *&lat, double *&lon, double *&hei, int &nodeNum)
// {
// 	//Read kml
// 	FILE *fp = fopen(kmlPath, "r");
// 	char str[255]; //each line
// 	char str_content[255]; //contest of each line
// 	char str_coordinate[255] = ""; //coordinates that we need
// 	while (!feof(fp))
// 	{
// 		fgets(str, 255, fp);
// 		sscanf(str, "%s", str_content);
// 		if (strcmp(str_content, "<coordinates>") == 0)
// 		{
// 			fgets(str, 255, fp);
// 			sscanf(str, "%s", str_content);
// 			while (strcmp(str_content, "</coordinates>") != 0)
// 			{
// 				strcat(str_coordinate, str);
// 				strcat(str_coordinate, "\t");
// 				fgets(str, 255, fp);
// 				sscanf(str, "%s", str_content);
// 			}
// 
// 			break;
// 		}
// 	}
// 
// 	int ptNum = 0;
// 	for (int i = 0; i < strlen(str_coordinate); i++)
// 	{
// 		if (str_coordinate[i] == ',')
// 		{
// 			ptNum++;
// 		}
// 	}
// 	ptNum /= 2;
// 
// 	//	double *Lat, *Lon, *Hei;
// 	nodeNum = ptNum;
// 	lat = new double[ptNum];
// 	lon = new double[ptNum];
// 	hei = new double[ptNum];
// 
// 	char **data_type = new char *[ptNum];
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		data_type[i] = new char[255];
// 	}
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		strcpy(data_type[i], "");
// 	}
// 	for (int i = 0; i < ptNum; i++)
// 		for (int j = 0; j < ptNum; j++)
// 		{
// 			if (j == i)
// 			{
// 				strcat(data_type[i], "%lf,%lf,%lf ");
// 			}
// 			else
// 			{
// 				strcat(data_type[i], "%*lf,%*lf,%*lf ");
// 
// 			}
// 		}
// 
// 
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		sscanf(str_coordinate, data_type[i], &lon[i], &lat[i], &hei[i]);
// 	}
// 
// 	//found bounding box of aoi
// 	// 	avgHei = 0;
// 	// 	for (int i = 0; i < ptNum; i++)
// 	// 	{
// 	// 		avgHei += Hei[i];
// 	// 	}
// 	// 	avgHei /= ptNum;
// 	// 
// 	// 	LL2UTM(Lat[0], Lon[0], minx, miny, zoneID);
// 	// 	maxx = minx; maxy = miny;
// 	// 	for (int i = 0; i < ptNum; i++)
// 	// 	{
// 	// 		double tX, tY;
// 	// 		LL2UTM_fixedZone(Lat[i], Lon[i], zoneID, tX, tY);
// 	// 
// 	// 		if (tX < minx)
// 	// 		{
// 	// 			minx = tX;
// 	// 		}
// 	// 		if (tY < miny)
// 	// 		{
// 	// 			miny = tY;
// 	// 		}
// 	// 		if (tX > maxx)
// 	// 		{
// 	// 			maxx = tX;
// 	// 		}
// 	// 		if (tY > maxy)
// 	// 		{
// 	// 			maxy = tY;
// 	// 		}
// 	// 	}
// 
// 	//free memory
// 	fclose(fp); fp = NULL;
// 	// 	delete[]Lat; Lat = NULL;
// 	// 	delete[]Lon; Lon = NULL;
// 	// 	delete[]Hei; Hei = NULL;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		delete[]data_type[i]; data_type[i] = NULL;
// 	}
// 	delete[]data_type; data_type = NULL;
// 
// 	return 1;
// }
// 
// int CBundleAdjust::Compute_AOI_images(char *input_path, char *kml_path, char *output_folder)
// {
// 	//Read img and rpb file
// 	FILE *fp_rpc = fopen(input_path, "r");
// 	int imgNum = 0;
// 	fscanf(fp_rpc, "%d", &imgNum);
// 	if (imgNum <= 0)
// 	{
// 		fclose(fp_rpc);
// 		return 0;
// 	}
// 
// 	char **img_paths, **rpc_paths;
// 	img_paths = new char *[imgNum];
// 	rpc_paths = new char *[imgNum];
// 	for (int i = 0; i < imgNum;i++)
// 	{
// 		img_paths[i] = new char[MaxLen];
// 		rpc_paths[i] = new char[MaxLen];
// 	}
// 	for (int i = 0; i < imgNum;i++)
// 	{
// 		fscanf(fp_rpc, "%s", img_paths[i]);
// 		fscanf(fp_rpc, "%s", rpc_paths[i]);
// 	}
// 	fclose(fp_rpc); fp_rpc = NULL;
// 
// 	//Read kml file
// 	double *lat, *lon, *hei;
// 	int nodeNum = 0;
// 	ReadKML(kml_path, lat, lon, hei, nodeNum);
// 	if (nodeNum<3)
// 	{
// 		for (int i = 0; i < imgNum; i++)
// 		{
// 			delete[]img_paths[i]; img_paths[i] = NULL;
// 			delete[]rpc_paths[i]; rpc_paths[i] = NULL;
// 		}
// 		delete[]img_paths; img_paths = NULL;
// 		delete[]rpc_paths; rpc_paths = NULL;
// 		if (nodeNum!=0)
// 		{
// 			delete[]lat; lat = NULL;
// 			delete[]lon; lon = NULL;
// 			delete[]hei; hei = NULL;
// 		}
// 	}
// 
// 	//define the ZoneID;
// 	char zoneID[10];
// 	double tX, tY;
// 	LL2UTM(lat[0], lon[0], tX, tY, zoneID);
// 
// 	//create output folder
// 	_mkdir(output_folder);
// 
// 	//check the overlap of image tiles and AOI
// 	int cover_Num = 0;
// 	int extra_size = 200;
// 	for (int i = 0; i < imgNum; i++)
// 	{
// 		int min_px, min_py, max_px, max_py;
// 		bool ifcover;
// 		Check_AOI_Overlap(img_paths[i], rpc_paths[i], lat, lon, hei, nodeNum, extra_size,
// 			min_px, min_py, max_px, max_py, ifcover);
// 
// 		if (ifcover == true)
// 		{
// 			cover_Num++;
// 		}
// 	}
// 
// 	if (cover_Num == 0)
// 	{
// 		for (int i = 0; i < imgNum; i++)
// 		{
// 			delete[]img_paths[i]; img_paths[i] = NULL;
// 			delete[]rpc_paths[i]; rpc_paths[i] = NULL;
// 		}
// 		delete[]img_paths; img_paths = NULL;
// 		delete[]rpc_paths; rpc_paths = NULL;
// 		delete[]lat; lat = NULL;
// 		delete[]lon; lon = NULL;
// 		delete[]hei; hei = NULL;
// 
// 		return 0;
// 	}
// 
// 	int *sx, *sy, *imgw, *imgh, *imgID;
// 	sx = new int[cover_Num];
// 	sy = new int[cover_Num];
// 	imgw = new int[cover_Num];
// 	imgh = new int[cover_Num];
// 	imgID = new int[cover_Num];
// 
// 	int I = 0;
// 	for (int i = 0; i < imgNum; i++)
// 	{
// 		int min_px, min_py, max_px, max_py;
// 		bool ifcover;
// 		Check_AOI_Overlap(img_paths[i], rpc_paths[i], lat, lon, hei, nodeNum, extra_size,
// 			min_px, min_py, max_px, max_py, ifcover);
// 
// 		if (ifcover == true)
// 		{
// 			sx[I] = min_px;
// 			sy[I] = min_py;
// 			imgw[I] = max_px - min_px + 1;
// 			imgh[I] = max_py - min_py + 1;
// 			imgID[I] = i;
// 			I++;
// 		}
// 	}
// 
// 	//align all sx and imgw
// 	int min_sx = sx[0], max_ex = sx[0] + imgw[0] - 1;
// 	for (int i = 1; i < cover_Num;i++)
// 	{
// 		if (min_sx > sx[i]) min_sx = sx[i];
// 		if (max_ex < sx[i] + imgw[i] - 1) max_ex = sx[i] + imgw[i] - 1;
// 	}
// 	for (int i = 0; i < cover_Num;i++)
// 	{
// 		sx[i] = min_sx;
// 		imgw[i] = max_ex - min_sx + 1;
// 	}
// 
// 	//sort patches
// 	if (cover_Num!=1)
// 	{
// 		SortPatches(sx, sy, imgw, imgh, imgID, cover_Num, img_paths, rpc_paths);
// 	}
// 
// 	//Merge tiles and compute the corresponding RPC files
// 	int Maxsize = 20000;
// 	Merge_Img_Tiles(img_paths, rpc_paths, cover_Num, sx, sy, imgw, imgh, imgID, zoneID, output_folder);
// 
// 	//free memory
// 	for (int i = 0; i < imgNum;i++)
// 	{
// 		delete[]img_paths[i]; img_paths[i] = NULL;
// 		delete[]rpc_paths[i]; rpc_paths[i] = NULL;
// 	}
// 	delete[]img_paths; img_paths = NULL;
// 	delete[]rpc_paths; rpc_paths = NULL;
// 	delete[]lat; lat = NULL;
// 	delete[]lon; lon = NULL;
// 	delete[]hei; hei = NULL;
// 	delete[]sx; sx = NULL;
// 	delete[]sy; sy = NULL;
// 	delete[]imgw; imgw = NULL;
// 	delete[]imgh; imgh = NULL;
// 	delete[]imgID; imgID = NULL;
// 	return 1;
// }
// 
// int CBundleAdjust::Merge_Img_Two_Tiles(char imgPaths[2][255], char rpcPaths[2][255], int *sx, int *sy, 
// 	int *imgw, int *imgh, int origin_ID, char *zoneID,
// 	char *merge_result_path, char *merge_rpc_path)
// {
// 	//Read image tile
// 	int imgw_origin, sx_origin;
// 	imgw_origin = imgw[origin_ID];
// 	sx_origin = sx[origin_ID];
// 	unsigned short int **img_data = new unsigned short int *[2];
// 	int bands;
// 	for (int i = 0; i < 2; i++)
// 	{
// 		CImageBase img;
// 		img.Open(imgPaths[i]);
// 		bands = img.GetBands();
// 		img_data[i] = new unsigned short int[imgw_origin * imgh[i] * bands];
// 		img.Read(img_data[i], sx_origin, sy[i], imgw_origin, imgh[i]);
// 		img.Close();
// 	}
// 
// 	//Read rpc files
// 	RPC rpcs[2];
// 	for (int i = 0; i < 2; i++)
// 	{
// 		ReadRpc_singleFile(rpcPaths[i], rpcs[i]);
// 	}
// 
// 	//Remember to sort the tiles before merging
// 	int imgHeight;
// 	int translate_y;
// 	double tx, ty;
// 	ComputeTranslateBetweenTiles(img_data[0], sx_origin, sy[0], imgw_origin, imgh[0],
// 		img_data[1], sx_origin, sy[1], imgw_origin, imgh[1], rpcs[0], rpcs[1], tx, ty);
// 
//  	translate_y = (int)(ty - (sy[0] - sy[1]) + 0.5); //y1 = y2 + translate_y
// 
// 	imgHeight = imgh[1] + translate_y;
// 
// 	unsigned short int *img_total = new unsigned short int[imgw_origin * imgHeight*bands];
// 
// 	int x, y;
// 	int k;
// 	for (y = 0; y < imgHeight; y++)
// 		for (x = 0; x < imgw_origin; x++)
// 		{
// 			if (y<translate_y)
// 			{
// 				for (k = 0; k < bands;k++)
// 				{
// 					img_total[k*imgw_origin*imgHeight + y*imgw_origin + x] 
// 						= img_data[0][k*imgw_origin*imgh[0] + y*imgw_origin + x];
// 				}
// 			}
// 			else if (y>imgh[0] - 1)
// 			{
// 				for (k = 0; k < bands; k++)
// 				{
// 					img_total[k*imgw_origin*imgHeight + y*imgw_origin + x]
// 						= img_data[1][k*imgw_origin*imgh[1] + (y - translate_y)*imgw_origin + x];
// 				}
// 			}
// 			else
// 			{
// 				for (k = 0; k < bands; k++)
// 				{
// 					img_total[k*imgw_origin*imgHeight + y*imgw_origin + x]
// 						= (img_data[0][k*imgw_origin*imgh[0] + y*imgw_origin + x]
// 						+ img_data[1][k*imgw_origin*imgh[1] + (y - translate_y)*imgw_origin + x]) / 2;
// 				}
// 			}
// 		}
// 
// 	SaveImageFile(merge_result_path, (unsigned char *)img_total, imgw_origin, imgHeight, bands, 16);
// 
// 	delete[]img_total; img_total = NULL;
// 
// 	//compute corresponding rpc files
// 	//1. compute the header information of the final rpc
// 	RPC rpc;
// 	rpc.lineOffset = (imgHeight - 1) / 2;
// 	rpc.sampOffset = (imgw_origin - 1) / 2;
// 	rpc.lineScale = rpc.lineOffset + 1;
// 	rpc.sampScale = rpc.sampOffset + 1;
// 
// 	rpc.heightOffset = (rpcs[0].heightOffset + rpcs[1].heightOffset) / 2;
// 	rpc.heightScale = max(rpcs[0].heightScale, rpcs[1].heightScale);
// 
// 	double minHeight = rpc.heightOffset - rpc.heightScale;
// 	double maxHeight = rpc.heightOffset + rpc.heightScale;
// 
// 	double lat1[4], lon1[4];
// 	double lat2[4], lon2[4];
// 	RPC_Image2Ground(sy[0], sx[0], minHeight, rpcs[0], lat1[0], lon1[0]);
// 	RPC_Image2Ground(sy[0] + imgh[0], sx[0], minHeight, rpcs[0], lat1[1], lon1[1]);
// 	RPC_Image2Ground(sy[0] + imgh[0], sx[0] + imgw_origin, minHeight, rpcs[0], lat1[2], lon1[2]);
// 	RPC_Image2Ground(sy[0], sx[0] + imgw_origin, minHeight, rpcs[0], lat1[3], lon1[3]);
// 
// 	RPC_Image2Ground(sy[1], sx[1], minHeight, rpcs[1], lat2[0], lon2[0]);
// 	RPC_Image2Ground(sy[1] + imgh[1], sx[1], minHeight, rpcs[1], lat2[1], lon2[1]);
// 	RPC_Image2Ground(sy[1] + imgh[1], sx[1] + imgw_origin, minHeight, rpcs[1], lat2[2], lon2[2]);
// 	RPC_Image2Ground(sy[1], sx[1] + imgw_origin, minHeight, rpcs[1], lat2[3], lon2[3]);
// 
// 	double minLat, maxLat, minLon, maxLon;
// 	minLat = lat1[0]; maxLat = lat1[0];
// 	minLon = lon1[0]; maxLon = lon1[0];
// 	for (int i = 1; i < 4; i++)
// 	{
// 		if (minLat > lat1[i])
// 			minLat = lat1[i];
// 		if (maxLat < lat1[i])
// 			maxLat = lat1[i];
// 		if (minLon > lon1[i])
// 			minLon = lon1[i];
// 		if (maxLon < lon1[i])
// 			maxLon = lon1[i];
// 	}
// 	for (int i = 0; i < 4; i++)
// 	{
// 		if (minLat > lat2[i])
// 			minLat = lat2[i];
// 		if (maxLat < lat2[i])
// 			maxLat = lat2[i];
// 		if (minLon > lon2[i])
// 			minLon = lon2[i];
// 		if (maxLon < lon2[i])
// 			maxLon = lon2[i];
// 	}
// 
// 	rpc.latOffset = (minLat + maxLat) / 2;
// 	rpc.longOffset = (minLon + maxLon) / 2;
// 
// 	rpc.latScale = maxLat - rpc.latOffset;
// 	rpc.longScale = maxLon - rpc.longOffset;
// 
// 	//2. compute the corresponding GCPs and the image points
// 	int step_plane = 50;
// 	int step_height = 30;
// 	int imgWidth = imgw_origin;
// 	int ptnum = (imgHeight / step_plane) * (imgWidth / step_plane)*((int)(maxHeight - minHeight) / step_height);
// 	ptnum = 0;
// 	double h = 0;
// 	for (y = 0; y < imgHeight; y += step_plane)
// 		for (x = 0; x < imgWidth; x += step_plane)
// 			for (h = minHeight; h <= maxHeight; h += step_height)
// 			{
// 				ptnum++;
// 			}
// 	double *px, *py;
// 	double *lat, *lon, *hei;
// 	px = new double[ptnum];
// 	py = new double[ptnum];
// 	lat = new double[ptnum];
// 	lon = new double[ptnum];
// 	hei = new double[ptnum];
// 	memset(px, 0, sizeof(double)*ptnum);
// 	memset(py, 0, sizeof(double)*ptnum);
// 	memset(lat, 0, sizeof(double)*ptnum);
// 	memset(lon, 0, sizeof(double)*ptnum);
// 	memset(hei, 0, sizeof(double)*ptnum);
// 
// 	int I = 0;
// 	h = 0;
// 	for (y = 0; y < imgHeight; y += step_plane)
// 		for (x = 0; x < imgw[origin_ID]; x += step_plane)
// 			for (h = minHeight; h <= maxHeight; h += step_height)
// 			{
// 				px[I] = x;
// 				py[I] = y;
// 				hei[I] = h;
// 				if (y<translate_y)
// 				{
// 					RPC_Image2Ground(sy[0] + py[I], sx_origin + px[I], hei[I], rpcs[0], lat[I], lon[I]);
// 				}
// 				else if (y>imgh[0] - 1)
// 				{
// 					RPC_Image2Ground(sy[1] + (py[I] - translate_y), sx_origin + px[I], hei[I], rpcs[1], lat[I], lon[I]);
// 				}
// 				else
// 				{
// 					double lat1_tmp, lon1_tmp;
// 					double lat2_tmp, lon2_tmp;
// 					RPC_Image2Ground(sy[0] + py[I], sx_origin + px[I], hei[I], rpcs[0], lat1_tmp, lon1_tmp);
// 					RPC_Image2Ground(sy[1] + (py[I] - translate_y), sx_origin + px[I], hei[I], rpcs[1], lat2_tmp, lon2_tmp);
// 					lat[I] = (lat1_tmp + lat2_tmp) / 2;
// 					lon[I] = (lon1_tmp + lon2_tmp) / 2;
// 				}
// 
// 				I++;
// 			}
// 
// 	//3. finally compute the corresponding RPC parameters
// 	double maxd = 0;
// 	ComputeRPCfromVirtualGCPs(px, py, lat, lon, hei, ptnum, rpc, maxd);
// 
// 	char acc_path[255];
// 	FindPrjDir(merge_rpc_path, acc_path);
// 
// 	//4. output RPC file
// 	FILE *fp_in = fopen(rpcPaths[0], "r");
// 	FILE *fp_out = fopen(merge_rpc_path, "w");
// 
// 	char str[255];
// 	fgets(str, 255, fp_in);
// 	fprintf(fp_out, "%s", str);
// 	fgets(str, 255, fp_in);
// 	fprintf(fp_out, "%s", str);
// 	fgets(str, 255, fp_in);
// 	fprintf(fp_out, "%s", str);
// 	fgets(str, 255, fp_in);
// 	fprintf(fp_out, "%s", str);
// 	fgets(str, 255, fp_in);
// 	fprintf(fp_out, "%s", str);
// 	fgets(str, 255, fp_in);
// 	fprintf(fp_out, "%s", str);
// 
// 	fprintf(fp_out, "\tlineOffset = %lf;\n", rpc.lineOffset);
// 
// 	fprintf(fp_out, "\tsampOffset = %lf;\n", rpc.sampOffset);
// 
// 	fprintf(fp_out, "\tlatOffset = %lf;\n", rpc.latOffset);
// 
// 	fprintf(fp_out, "\tlongOffset = %lf;\n", rpc.longOffset);
// 
// 	fprintf(fp_out, "\theightOffset = %lf;\n", rpc.heightOffset);
// 
// 	fprintf(fp_out, "\tlineScale = %lf;\n", rpc.lineScale);
// 
// 	fprintf(fp_out, "\tsampScale = %lf;\n", rpc.sampScale);
// 
// 	fprintf(fp_out, "\tlatScale = %lf;\n", rpc.latScale);
// 
// 	fprintf(fp_out, "\tlongScale = %lf;\n", rpc.longScale);
// 
// 	fprintf(fp_out, "\theightScale = %lf;\n", rpc.heightScale);
// 
// 	fprintf(fp_out, "\t%s\n", "lineNumCoef = (");
// 	int j;
// 	for (j = 0; j < 19; j++)
// 	{
// 		fprintf(fp_out, "			%e,\n", rpc.lineNumCoef[j]);
// 	}
// 	fprintf(fp_out, "			%e);\n", rpc.lineNumCoef[j]);
// 
// 	fprintf(fp_out, "\t%s\n", "lineDenCoef = (");
// 	for (j = 0; j < 19; j++)
// 	{
// 		fprintf(fp_out, "			%e,\n", rpc.lineDenCoef[j]);
// 	}
// 	fprintf(fp_out, "			%e);\n", rpc.lineDenCoef[j]);
// 
// 	fprintf(fp_out, "\t%s\n", "sampNumCoef = (");
// 	for (j = 0; j < 19; j++)
// 	{
// 		fprintf(fp_out, "			%e,\n", rpc.sampNumCoef[j]);
// 	}
// 	fprintf(fp_out, "			%e);\n", rpc.sampNumCoef[j]);
// 
// 	fprintf(fp_out, "\t%s\n", "sampDenCoef = (");
// 	for (j = 0; j < 19; j++)
// 	{
// 		fprintf(fp_out, "			%e,\n", rpc.sampDenCoef[j]);
// 	}
// 	fprintf(fp_out, "			%e);\n", rpc.sampDenCoef[j]);
// 
// 	fprintf(fp_out, "%s\n", "END_GROUP = IMAGE");
// 	fprintf(fp_out, "%s\n", "END;");
// 
// 	fclose(fp_out); fp_out = NULL;
// 	fclose(fp_in); fp_in = NULL;
// 
// 	//free memory
// 	for (int i = 0; i < 2; i++)
// 	{
// 		delete[]img_data[i]; img_data[i] = NULL;
// 	}
//  	delete[]img_data; img_data = NULL;
// 	delete[]px; px = NULL;
// 	delete[]py; py = NULL;
// 	delete[]lat; lat = NULL;
// 	delete[]lon; lon = NULL;
// 	delete[]hei; hei = NULL;
// 
// 	return 1;
// }
// 
// int CBundleAdjust::Merge_Img_Tiles(char **imgPaths, char **rpcPaths, int coverNum, int *sx, int *sy, int *imgw, int *imgh, 
// 	int *imgID, char *zoneID, char *output_folder)
// {
// 	//we merge every two tiles
// 	for (int i = 0; i < coverNum - 1;i+=2)
// 	{
// 		int bands;
// 		//Read image tile
// 		unsigned short int **img_data = new unsigned short int *[2];
// 		CImageBase img;
// 		int ID = imgID[i];
// 		img.Open(imgPaths[ID]);
// 		bands = img.GetBands();
// 		img_data[0] = new unsigned short int[imgw[i] * imgh[i] * bands];
// 		img.Read(img_data[0], sx[i], sy[i], imgw[i], imgh[i]);
// 		img.Close();
// 
// 		ID = imgID[i + 1];
// 		img.Open(imgPaths[ID]);
// 		img_data[1] = new unsigned short int[imgw[i + 1] * imgh[i + 1] * bands];
// 		img.Read(img_data[1], sx[i + 1], sy[i + 1], imgw[i + 1], imgh[i + 1]);
// 		img.Close();
// 
// 		//Read rpc files
// 		RPC rpcs[2];
// 		ID = imgID[i];
// 		ReadRpc_singleFile(rpcPaths[ID], rpcs[0]);
// 
// 		ID = imgID[i + 1];
// 		ReadRpc_singleFile(rpcPaths[ID], rpcs[1]);
// 
// 		//Merge the two tiles
// 		double tx, ty;
// 		ComputeTranslateBetweenTiles(sx[i], sy[i], imgw[i], imgh[i], sx[i + 1], sy[i + 1], imgw[i + 1], imgh[i + 1],
// 			rpcs[0], rpcs[1], tx, ty);
// 
// 		//compute the system error between the two patches
// 		double ax1 = 0 , bx1 = 0, cx1 = 0;
// 		double ay1 = 0, by1 = 0, cy1 = 0;
// 		ComputeSystemErrorBetweenTiles(sx[i], sy[i], imgw[i], imgh[i], sx[i + 1], sy[i + 1], imgw[i + 1], imgh[i + 1],
// 			rpcs[0], rpcs[1], tx, ty, ax1, bx1, cx1, ay1, by1, cy1);
// 
// 		int translate_y = (int)(ty - (sy[i] - sy[i + 1]) + 0.5); //ty = y - y2 = (sy + py) - (sy2 + py2) = (sy - sy2) + (py - py2)
// 
// 		int imgHeight = imgh[i + 1] + translate_y;
// 		int imgWidth = imgw[i];
// 		unsigned short int *img_total = new unsigned short int[imgWidth * imgHeight * bands];
// 
// 		int x, y;
// 		int k;
// 		for (y = 0; y < imgHeight; y++)
// 			for (x = 0; x < imgWidth; x++)
// 			{
// 				if (y<translate_y)
// 				{
// 					for (k = 0; k < bands;k++)
// 					{
// 						img_total[k*imgWidth*imgHeight + y*imgWidth + x] 
// 							= img_data[0][k*imgWidth*imgh[i] + y*imgWidth + x];
// 					}
// 				}
// 				else if (y>imgh[i] - 1)
// 				{
// 					for (k = 0; k < bands; k++)
// 					{
// 						img_total[k*imgWidth*imgHeight + y*imgWidth + x]
// 							= img_data[1][k*imgWidth*imgh[i + 1] + (y - translate_y)*imgWidth + x];
// 					}
// 				}
// 				else
// 				{
// 					for (k = 0; k < bands; k++)
// 					{
// 						img_total[k*imgWidth*imgHeight + y*imgWidth + x] = (img_data[0][k*imgWidth*imgh[i] + y*imgWidth + x]
// 							+ img_data[1][k*imgWidth*imgh[i + 1] + (y - translate_y)*imgWidth + x]) / 2;
// 					}
// 				}
// 			}
// 
// 		//get the img save path
// 		char output_path[MaxLen];
// 		strcpy(output_path, output_folder);
// 		strcat(output_path, "\\");
// 		
// 		char img_name[MaxLen];
// 		sprintf(img_name, "P%d_P%d.tif", i, i + 1);
// 
// 		strcat(output_path, img_name);
// 
// 		SaveImageFile(output_path, (unsigned char *)img_total, imgWidth, imgHeight, bands, 16);
// 
// 		//compute RPC file for the merged tiles
// 		//1. compute the header information of the final rpc
// 		RPC rpc;
// 		rpc.lineOffset = (imgHeight - 1) / 2;
// 		rpc.sampOffset = (imgWidth - 1) / 2;
// 		rpc.lineScale = rpc.lineOffset + 1;
// 		rpc.sampScale = rpc.sampOffset + 1;
// 
// 		rpc.heightOffset = (rpcs[0].heightOffset + rpcs[1].heightOffset) / 2;
// 		rpc.heightScale = max(rpcs[0].heightScale, rpcs[1].heightScale);
// 
// 		double minHeight = rpc.heightOffset - rpc.heightScale;
// 		double maxHeight = rpc.heightOffset + rpc.heightScale;
// 
// 		double lat1[4], lon1[4];
// 		double lat2[4], lon2[4];
// 		RPC_Image2Ground(sy[i], sx[i], minHeight, rpcs[0], lat1[0], lon1[0]);
// 		RPC_Image2Ground(sy[i] + imgh[i], sx[i], minHeight, rpcs[0], lat1[1], lon1[1]);
// 		RPC_Image2Ground(sy[i] + imgh[i], sx[i] + imgWidth, minHeight, rpcs[0], lat1[2], lon1[2]);
// 		RPC_Image2Ground(sy[i], sx[i] + imgWidth, minHeight, rpcs[0], lat1[3], lon1[3]);
// 
// 		RPC_Image2Ground(sy[i + 1], sx[i + 1], minHeight, rpcs[1], lat2[0], lon2[0]);
// 		RPC_Image2Ground(sy[i + 1] + imgh[i + 1], sx[i + 1], minHeight, rpcs[1], lat2[1], lon2[1]);
// 		RPC_Image2Ground(sy[i + 1] + imgh[i + 1], sx[i + 1] + imgWidth, minHeight, rpcs[1], lat2[2], lon2[2]);
// 		RPC_Image2Ground(sy[i + 1], sx[i + 1] + imgWidth, minHeight, rpcs[1], lat2[3], lon2[3]);
// 
// 		double minLat, maxLat, minLon, maxLon;
// 		minLat = lat1[0]; maxLat = lat1[0];
// 		minLon = lon1[0]; maxLon = lon1[0];
// 		for (int j = 1; j < 4; j++)
// 		{
// 			if (minLat > lat1[j]) minLat = lat1[j];
// 			if (maxLat < lat1[j]) maxLat = lat1[j];
// 			if (minLon > lon1[j]) minLon = lon1[j];
// 			if (maxLon < lon1[j]) maxLon = lon1[j];
// 		}
// 		for (int j = 0; j < 4; j++)
// 		{
// 			if (minLat > lat2[j]) minLat = lat2[j];
// 			if (maxLat < lat2[j]) maxLat = lat2[j];
// 			if (minLon > lon2[j]) minLon = lon2[j];
// 			if (maxLon < lon2[j]) maxLon = lon2[j];
// 		}
// 
// 		rpc.latOffset = (minLat + maxLat) / 2;
// 		rpc.longOffset = (minLon + maxLon) / 2;
// 
// 		rpc.latScale = maxLat - rpc.latOffset;
// 		rpc.longScale = maxLon - rpc.longOffset;
// 
// 		//2. compute the corresponding GCPs and the image points
// 		int step_h = imgHeight/100; //200
// 		int step_w = imgWidth / 100; //200
// 		int step_ele = (maxHeight - minHeight) / 25;
// 		
// 		if (step_h < 1) step_h = 1;
// 		if (step_w < 1) step_w = 1;
// 		if (step_ele < 1) step_ele = 1;
// 
// 		int ptnum = 0;
// 		double h = 0;
// 		for (y = -step_h; y < imgHeight + step_h; y += step_h)
// 			for (x = -step_w; x < imgWidth + step_w; x += step_w)
// 				for (h = minHeight-step_ele; h <= maxHeight + step_ele; h += step_ele)
// 				{
// 					ptnum++;
// 				}
// 		double *px, *py;
// 		double *lat, *lon, *hei;
// 		px = new double[ptnum];
// 		py = new double[ptnum];
// 		lat = new double[ptnum];
// 		lon = new double[ptnum];
// 		hei = new double[ptnum];
// 		memset(px, 0, sizeof(double)*ptnum);
// 		memset(py, 0, sizeof(double)*ptnum);
// 		memset(lat, 0, sizeof(double)*ptnum);
// 		memset(lon, 0, sizeof(double)*ptnum);
// 		memset(hei, 0, sizeof(double)*ptnum);
// 
// 		int I = 0;
// 		h = 0;
// 		//double MaxX = 0, MaxY = 0;
// 		//considering that it is normal that the two patches are not consistent
// 		//it is better to firstly register them
// 		int overlapNum = 0;
// 		for (y = -step_h; y < imgHeight + step_h; y += step_h)
// 			for (x = -step_w; x < imgWidth + step_w; x += step_w)
// 				for (h = minHeight - step_ele; h <= maxHeight + step_ele; h += step_ele)
// 				{
// 					if (y >= translate_y &&  y <= imgh[i] - 1) // the overlap region
// 					{
// 						overlapNum++;
// 					}
// 				}
// 
// 		double *X1, *Y1, *Z1, *X2, *Y2, *Z2;
// 		X1 = new double[overlapNum];
// 		Y1 = new double[overlapNum];
// 		Z1 = new double[overlapNum];
// 		X2 = new double[overlapNum];
// 		Y2 = new double[overlapNum];
// 		Z2 = new double[overlapNum];
// 
// 		I = 0;
// 		for (y = -step_h; y < imgHeight + step_h; y += step_h)
// 			for (x = -step_w; x < imgWidth + step_w; x += step_w)
// 				for (h = minHeight - step_ele; h <= maxHeight + step_ele; h += step_ele)
// 				{
// 					double tx, ty, th;
// 					tx = x;
// 					ty = y;
// 					th = h;
// 					if (y >= translate_y &&  y <= imgh[i] - 1) // the overlap region
// 					{
// 						double lat1_tmp, lon1_tmp;
// 						double lat2_tmp, lon2_tmp;
// 						RPC_Image2Ground(sy[i] + ty, sx[i] + tx, th, rpcs[0], lat1_tmp, lon1_tmp);
// 
// 						double dx = 0, dy = 0;
// 						dx = ax1*sin(bx1*(sy[i + 1] + (ty - translate_y)) + cx1);
// 						dy = ay1*sin(by1*(sy[i + 1] + (ty - translate_y)) + cy1);
// 						RPC_Image2Ground(sy[i + 1] + (ty - translate_y) - dy, sx[i + 1] + tx - dx, th, rpcs[1], lat2_tmp, lon2_tmp);
// 
// 						LL2UTM_fixedZone(lat1_tmp, lon1_tmp, zoneID, X1[I], Y1[I]);
// 						LL2UTM_fixedZone(lat2_tmp, lon2_tmp, zoneID, X2[I], Y2[I]);
// 
// 						Z1[I] = th;  Z2[I] = th;
// 
// 						I++;
// 					}
// 				}
// 
// 		double tX = 0, tY = 0;
// 		for (int j = 0; j < overlapNum;j++)
// 		{
// 			tX += fabs(X1[j] - X2[j]);
// 			tY += fabs(Y1[j] - Y2[j]);
// 		}
// 		tX /= overlapNum;
// 		tY /= overlapNum;
// 
// 		//free memory
// 		delete[]X1; X1 = NULL;
// 		delete[]Y1; Y1 = NULL;
// 		delete[]Z1; Z1 = NULL;
// 		delete[]X2; X2 = NULL;
// 		delete[]Y2; Y2 = NULL;
// 		delete[]Z2; Z2 = NULL;
// 			
// 		//generate virtual GCPs
// //		double maxdy = 0, maxdx = 0;
// 		I = 0;
// 		for (y = -step_h; y < imgHeight + step_h; y += step_h)
// 			for (x = -step_w; x < imgWidth + step_w; x += step_w)
// 				for (h = minHeight - step_ele; h <= maxHeight + step_ele; h += step_ele)
// 				{
// 					px[I] = x;
// 					py[I] = y;
// 					hei[I] = h;
// 					if (y<translate_y)
// 					{
// 						RPC_Image2Ground(sy[i] + py[I], sx[i] + px[I], hei[I], rpcs[0], lat[I], lon[I]);
// 					}
// 					else if (y>imgh[i] - 1)
// 					{
// 						double lat2_tmp, lon2_tmp;
// 						/*RPC_Image2Ground(sy[i + 1] + (py[I] - translate_y), sx[i + 1] + px[I], hei[I], rpcs[1], lat[I], lon[I]);*/
// 
// 						double dx = 0, dy = 0;
// // 						dx = ax1*sin(bx1*(sy[i + 1] + (py[I] - translate_y)) + cx1);
// // 						dy = ay1*sin(by1*(sy[i + 1] + (py[I] - translate_y)) + cy1);
// // 						dx = cx1; dy = cy1;
// // 						double x2, y2;
// // 						ComputeCoordinateBeforeCorrection(sx[i + 1] + px[I], sy[i + 1] + (py[I] - translate_y),
// // 							ax1, bx1, cx1,
// // 							ay1, by1, cy1, x2, y2, dx, dy);
// // 						RPC_Image2Ground(y2, x2, hei[I], rpcs[1], lat2_tmp, lon2_tmp);
// 
// 						RPC_Image2Ground(sy[i + 1] + (py[I] - translate_y) - dy, sx[i + 1] + px[I] - dx,
// 							hei[I], rpcs[1], lat2_tmp, lon2_tmp);
// 						
// 
// 						lat[I] = lat2_tmp;
// 						lon[I] = lon2_tmp;
// 					}
// 					else
// 					{
// 						double lat1_tmp, lon1_tmp;
// 						double lat2_tmp, lon2_tmp;
// 						RPC_Image2Ground(sy[i] + py[I], sx[i] + px[I], hei[I], rpcs[0], lat1_tmp, lon1_tmp);
// 
// 						double dx = 0, dy = 0;
// // 						dx = ax1*sin(bx1*(sy[i + 1] + (py[I] - translate_y)) + cx1);
// // 						dy = ay1*sin(by1*(sy[i + 1] + (py[I] - translate_y)) + cy1);
// // 						dx = cx1; dy = cy1;
// 
// // 						double x2, y2;
// // 						double dx2, dy2;
// // 						ComputeCoordinateBeforeCorrection(sx[i + 1] + px[I], sy[i + 1] + (py[I] - translate_y),
// // 							ax1, bx1, cx1,
// // 							ay1, by1, cy1, x2, y2, dx2, dy2);
// // 						RPC_Image2Ground(y2, x2, hei[I], rpcs[1], lat2_tmp, lon2_tmp);
// 
// 						RPC_Image2Ground(sy[i + 1] + (py[I] - translate_y) - dy, sx[i + 1] + px[I] - dx, 
// 							hei[I], rpcs[1], lat2_tmp, lon2_tmp);
// 
// // 						if (maxdy<fabs(dy2 - dy))
// // 						{
// // 							maxdy = fabs(dy2 - dy);
// // 						}
// // 
// // 						if (maxdx < fabs(dx2 - dx))
// // 						{
// // 							maxdx = fabs(dx2 - dx);
// // 						}
// 						
// 
// 						lat[I] = (lat1_tmp + lat2_tmp) / 2;
// 						lon[I] = (lon1_tmp + lon2_tmp) / 2;
// 					}
// 
// 					I++;
// 				}
// 
// 		//3. finally compute the corresponding RPC parameters
// 		double maxd = 0;
// 		ptnum = I;
// 		ComputeRPCfromVirtualGCPs(px, py, lat, lon, hei, ptnum, rpc, maxd);
// 
// 		char acc_path[255];
// 		FindPrjDir(output_path, acc_path);
// 		strcat(acc_path, "_acc.txt");
// 		FILE *fp_acc = fopen(acc_path, "w");
// 		fprintf(fp_acc, "%lf\n", maxd);
// 		fclose(fp_acc); fp_acc = NULL;
// 
// 		//4. output RPC file
// 		char merge_rpc_path[MaxLen];
// 		strcpy(merge_rpc_path, output_folder);
// 		strcat(merge_rpc_path, "\\");
// 
// 		char rpc_name[MaxLen];
// 		sprintf(rpc_name, "P%d_P%d.RPB", i, i + 1);
// 
// 
// 		strcat(merge_rpc_path, rpc_name);
// 		ID = imgID[i];
// 		FILE *fp_in = fopen(rpcPaths[ID], "r");
// 		FILE *fp_out = fopen(merge_rpc_path, "w");
// 
// 		char str[255];
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 
// 		fprintf(fp_out, "\tlineOffset = %lf;\n", rpc.lineOffset);
// 
// 		fprintf(fp_out, "\tsampOffset = %lf;\n", rpc.sampOffset);
// 
// 		fprintf(fp_out, "\tlatOffset = %lf;\n", rpc.latOffset);
// 
// 		fprintf(fp_out, "\tlongOffset = %lf;\n", rpc.longOffset);
// 
// 		fprintf(fp_out, "\theightOffset = %lf;\n", rpc.heightOffset);
// 
// 		fprintf(fp_out, "\tlineScale = %lf;\n", rpc.lineScale);
// 
// 		fprintf(fp_out, "\tsampScale = %lf;\n", rpc.sampScale);
// 
// 		fprintf(fp_out, "\tlatScale = %lf;\n", rpc.latScale);
// 
// 		fprintf(fp_out, "\tlongScale = %lf;\n", rpc.longScale);
// 
// 		fprintf(fp_out, "\theightScale = %lf;\n", rpc.heightScale);
// 
// 		fprintf(fp_out, "\t%s\n", "lineNumCoef = (");
// 		int j;
// 		for (j = 0; j < 19; j++)
// 		{
// 			fprintf(fp_out, "			%e,\n", rpc.lineNumCoef[j]);
// 		}
// 		fprintf(fp_out, "			%e);\n", rpc.lineNumCoef[j]);
// 
// 		fprintf(fp_out, "\t%s\n", "lineDenCoef = (");
// 		for (j = 0; j < 19; j++)
// 		{
// 			fprintf(fp_out, "			%e,\n", rpc.lineDenCoef[j]);
// 		}
// 		fprintf(fp_out, "			%e);\n", rpc.lineDenCoef[j]);
// 
// 		fprintf(fp_out, "\t%s\n", "sampNumCoef = (");
// 		for (j = 0; j < 19; j++)
// 		{
// 			fprintf(fp_out, "			%e,\n", rpc.sampNumCoef[j]);
// 		}
// 		fprintf(fp_out, "			%e);\n", rpc.sampNumCoef[j]);
// 
// 		fprintf(fp_out, "\t%s\n", "sampDenCoef = (");
// 		for (j = 0; j < 19; j++)
// 		{
// 			fprintf(fp_out, "			%e,\n", rpc.sampDenCoef[j]);
// 		}
// 		fprintf(fp_out, "			%e);\n", rpc.sampDenCoef[j]);
// 
// 		fprintf(fp_out, "%s\n", "END_GROUP = IMAGE");
// 		fprintf(fp_out, "%s\n", "END;");
// 
// 		fclose(fp_out); fp_out = NULL;
// 		fclose(fp_in); fp_in = NULL;
// 
// 
// 		//free memory
// 		delete[]img_data[0]; img_data[0] = NULL;
// 		delete[]img_data[1]; img_data[1] = NULL;
// 		delete[]img_data; img_data = NULL;
// 		delete[]img_total; img_total = NULL;
// 		delete[]px; px = NULL;
// 		delete[]py; py = NULL;
// 		delete[]lat; lat = NULL;
// 		delete[]lon; lon = NULL;
// 		delete[]hei; hei = NULL;
// 	}
// 	if (coverNum%2==1) //the last one
// 	{
// 		int bands;
// 		//Read image tile
// 		unsigned short int *img_data;
// 		CImageBase img;
// 		int ID = imgID[coverNum - 1];
// 		img.Open(imgPaths[ID]);
// 		bands = img.GetBands();
// 		img_data = new unsigned short int[imgw[coverNum - 1] * imgh[coverNum - 1] * bands];
// 		img.Read(img_data, sx[coverNum - 1], sy[coverNum - 1], imgw[coverNum - 1], imgh[coverNum - 1]);
// 		img.Close();
// 
// 		//Read rpc files
// 		RPC Rpcs;
// 		ID = imgID[coverNum - 1];
// 		ReadRpc_singleFile(rpcPaths[ID], Rpcs);
// 
// 		//get the img save path
// 		char output_path[MaxLen];
// 		strcpy(output_path, output_folder);
// 		strcat(output_path, "\\");
// 
// 		char img_name[MaxLen];
// 		sprintf(img_name, "P%d.tif", coverNum - 1);
// 
// 		strcat(output_path, img_name);
// 
// 		SaveImageFile(output_path, (unsigned char *)img_data, imgw[coverNum - 1], imgh[coverNum - 1], bands, 16);
// 
// 		//Compute RPC for the tile
// 		//1. compute the header information of the final rpc
// 		int imgHeight = imgh[coverNum - 1];
// 		int imgWidth = imgw[coverNum - 1];
// 		RPC rpc;
// 		rpc.lineOffset = (imgHeight - 1) / 2;
// 		rpc.sampOffset = (imgWidth - 1) / 2;
// 		rpc.lineScale = rpc.lineOffset + 1;
// 		rpc.sampScale = rpc.sampOffset + 1;
// 
// 		rpc.heightOffset = Rpcs.heightOffset;
// 		rpc.heightScale = Rpcs.heightScale;
// 
// 		double minHeight = rpc.heightOffset - rpc.heightScale;
// 		double maxHeight = rpc.heightOffset + rpc.heightScale;
// 
// 		double lat1[4], lon1[4];
// 		RPC_Image2Ground(sy[coverNum - 1], sx[coverNum - 1], minHeight, Rpcs, lat1[0], lon1[0]);
// 		RPC_Image2Ground(sy[coverNum - 1] + imgHeight, sx[coverNum - 1], minHeight, Rpcs, lat1[1], lon1[1]);
// 		RPC_Image2Ground(sy[coverNum - 1] + imgHeight, sx[coverNum - 1] + imgWidth, minHeight, Rpcs, lat1[2], lon1[2]);
// 		RPC_Image2Ground(sy[coverNum - 1], sx[coverNum - 1] + imgWidth, minHeight, Rpcs, lat1[3], lon1[3]);
// 
// 		double minLat, maxLat, minLon, maxLon;
// 		minLat = lat1[0]; maxLat = lat1[0];
// 		minLon = lon1[0]; maxLon = lon1[0];
// 		for (int j = 1; j < 4; j++)
// 		{
// 			if (minLat > lat1[j]) minLat = lat1[j];
// 			if (maxLat < lat1[j]) maxLat = lat1[j];
// 			if (minLon > lon1[j]) minLon = lon1[j];
// 			if (maxLon < lon1[j]) maxLon = lon1[j];
// 		}
// 
// 		rpc.latOffset = (minLat + maxLat) / 2;
// 		rpc.longOffset = (minLon + maxLon) / 2;
// 
// 		rpc.latScale = maxLat - rpc.latOffset;
// 		rpc.longScale = maxLon - rpc.longOffset;
// 
// 		//2. compute the corresponding GCPs and the image points
// 		int step_h = /*100*/imgHeight / 100;
// 		int step_w = /*100*/imgWidth / 100;
// 		int step_ele = /*50*/(maxHeight - minHeight) / 25;
// 
// 		if (step_h < 1) step_h = 1;
// 		if (step_w < 1) step_w = 1;
// 		if (step_ele < 1) step_ele = 1;
// 
// 		int ptnum = 0;
// 		double h = 0;
// 		for (int y = -step_h; y < imgHeight + step_h; y += step_h)
// 			for (int x = -step_w; x < imgWidth + step_w; x += step_w)
// 				for (h = minHeight - step_ele; h <= maxHeight + step_ele; h += step_ele)
// 				{
// 					ptnum++;
// 				}
// 		double *px, *py;
// 		double *lat, *lon, *hei;
// 		px = new double[ptnum];
// 		py = new double[ptnum];
// 		lat = new double[ptnum];
// 		lon = new double[ptnum];
// 		hei = new double[ptnum];
// 		memset(px, 0, sizeof(double)*ptnum);
// 		memset(py, 0, sizeof(double)*ptnum);
// 		memset(lat, 0, sizeof(double)*ptnum);
// 		memset(lon, 0, sizeof(double)*ptnum);
// 		memset(hei, 0, sizeof(double)*ptnum);
// 
// 		int I = 0;
// 		h = 0;
// 		for (int y = -step_h; y < imgHeight + step_h; y += step_h)
// 			for (int x = -step_w; x < imgWidth + step_w; x += step_w)
// 				for (h = minHeight - step_ele; h <= maxHeight + step_ele; h += step_ele)
// 				{
// 					px[I] = x;
// 					py[I] = y;
// 					hei[I] = h;
// 				
// 					RPC_Image2Ground(sy[coverNum - 1] + py[I], sx[coverNum - 1] + px[I], hei[I], Rpcs, lat[I], lon[I]);
// 
// 					I++;
// 				}
// 
// 		//3. finally compute the corresponding RPC parameters
// 		double maxd = 0;
// 		ComputeRPCfromVirtualGCPs(px, py, lat, lon, hei, ptnum, rpc, maxd);
// 
// 		char acc_path[255];
// 		FindPrjDir(output_path, acc_path);
// 		strcat(acc_path, "_acc.txt");
// 		FILE *fp_acc = fopen(acc_path, "w");
// 		fprintf(fp_acc, "%lf\n", maxd);
// 		fclose(fp_acc); fp_acc = NULL;
// 
// 
// 		//4. output RPC file
// 		char merge_rpc_path[MaxLen];
// 		strcpy(merge_rpc_path, output_folder);
// 		strcat(merge_rpc_path, "\\");
// 
// 		char rpc_name[MaxLen];
// 		sprintf(rpc_name, "P%d.RPB", coverNum - 1);
// 
// 
// 		strcat(merge_rpc_path, rpc_name);
// 		ID = imgID[coverNum - 1];
// 		FILE *fp_in = fopen(rpcPaths[ID], "r");
// 		FILE *fp_out = fopen(merge_rpc_path, "w");
// 
// 		char str[255];
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 		fgets(str, 255, fp_in);
// 		fprintf(fp_out, "%s", str);
// 
// 		fprintf(fp_out, "\tlineOffset = %lf;\n", rpc.lineOffset);
// 
// 		fprintf(fp_out, "\tsampOffset = %lf;\n", rpc.sampOffset);
// 
// 		fprintf(fp_out, "\tlatOffset = %lf;\n", rpc.latOffset);
// 
// 		fprintf(fp_out, "\tlongOffset = %lf;\n", rpc.longOffset);
// 
// 		fprintf(fp_out, "\theightOffset = %lf;\n", rpc.heightOffset);
// 
// 		fprintf(fp_out, "\tlineScale = %lf;\n", rpc.lineScale);
// 
// 		fprintf(fp_out, "\tsampScale = %lf;\n", rpc.sampScale);
// 
// 		fprintf(fp_out, "\tlatScale = %lf;\n", rpc.latScale);
// 
// 		fprintf(fp_out, "\tlongScale = %lf;\n", rpc.longScale);
// 
// 		fprintf(fp_out, "\theightScale = %lf;\n", rpc.heightScale);
// 
// 		fprintf(fp_out, "\t%s\n", "lineNumCoef = (");
// 		int j;
// 		for (j = 0; j < 19; j++)
// 		{
// 			fprintf(fp_out, "			%e,\n", rpc.lineNumCoef[j]);
// 		}
// 		fprintf(fp_out, "			%e);\n", rpc.lineNumCoef[j]);
// 
// 		fprintf(fp_out, "\t%s\n", "lineDenCoef = (");
// 		for (j = 0; j < 19; j++)
// 		{
// 			fprintf(fp_out, "			%e,\n", rpc.lineDenCoef[j]);
// 		}
// 		fprintf(fp_out, "			%e);\n", rpc.lineDenCoef[j]);
// 
// 		fprintf(fp_out, "\t%s\n", "sampNumCoef = (");
// 		for (j = 0; j < 19; j++)
// 		{
// 			fprintf(fp_out, "			%e,\n", rpc.sampNumCoef[j]);
// 		}
// 		fprintf(fp_out, "			%e);\n", rpc.sampNumCoef[j]);
// 
// 		fprintf(fp_out, "\t%s\n", "sampDenCoef = (");
// 		for (j = 0; j < 19; j++)
// 		{
// 			fprintf(fp_out, "			%e,\n", rpc.sampDenCoef[j]);
// 		}
// 		fprintf(fp_out, "			%e);\n", rpc.sampDenCoef[j]);
// 
// 		fprintf(fp_out, "%s\n", "END_GROUP = IMAGE");
// 		fprintf(fp_out, "%s\n", "END;");
// 
// 		fclose(fp_out); fp_out = NULL;
// 		fclose(fp_in); fp_in = NULL;
// 
// 		//free memory
// 		delete[]img_data; img_data = NULL;
// 		delete[]px; px = NULL;
// 		delete[]py; py = NULL;
// 		delete[]lat; lat = NULL;
// 		delete[]lon; lon = NULL;
// 		delete[]hei; hei = NULL;
// 	}
// 
// 	return 1;
// }
// 
// int CBundleAdjust::Merge_Img_Tiles(char **imgPaths, char **rpcPaths, int coverNum,
// 	int *sx, int *sy, int *imgw, int *imgh, int *imgID, int origin_ID, char *zoneID,
// 	char *merge_result_path, char *merge_rpc_path)
// {
// 	//Read image tile
// 	unsigned short int **img_data = new unsigned short int *[coverNum];
// 	for (int i = 0; i < coverNum;i++)
// 	{
// 		CImageBase img;
// 		int ID = imgID[i];
// 		img.Open(imgPaths[ID]);
// 		img_data[i] = new unsigned short int[imgw[origin_ID] * imgh[i]];
// 		img.Read(img_data[i], sx[origin_ID], sy[i], imgw[origin_ID], imgh[i]);
// 		img.Close();
// 	}
// 
// 	//Read rpc files
// 	RPC *rpcs;
// 	rpcs = new RPC[coverNum];
// 	for (int i = 0; i < coverNum;i++)
// 	{
// 		int ID = imgID[i];
// 		ReadRpc_singleFile(rpcPaths[ID], rpcs[i]);
// 	}
// 
// 	//Remember to sort the tiles before merging
// 	int imgHeight;
// 	int translate_y;
// 	for (int i = 0; i < coverNum - 1;i++)
// 	{
// 		double tx, ty;
// 		ComputeTranslateBetweenTiles(img_data[i], sx[origin_ID], sy[i], imgw[origin_ID], imgh[i],
// 			img_data[i + 1], sx[origin_ID], sy[i + 1], imgw[origin_ID], imgh[i + 1], rpcs[i], rpcs[i + 1], tx, ty);
// 
// 		translate_y = (int)(ty - (sy[i] - sy[i + 1]) + 0.5); //(sy1+y1) - (sy2+y2) = y1-y2 = ty - (sy1 - sy2)
// 
// 		imgHeight = imgh[i + 1] + translate_y;
// 
// 		unsigned short int *img_total = new unsigned short int[imgw[origin_ID] * imgHeight];
// 
// 		int x, y;
// 		for (y = 0; y < imgHeight; y++)
// 			for (x = 0; x < imgw[origin_ID];x++)
// 			{
// 				if (y<translate_y)
// 				{
// 					img_total[y*imgw[origin_ID] + x] = img_data[i][y*imgw[origin_ID] + x];
// 				}
// 				else if (y>imgh[i] - 1)
// 				{
// 					img_total[y*imgw[origin_ID] + x] = img_data[i + 1][(y - translate_y)*imgw[origin_ID] + x];
// 				}
// 				else
// 				{
// 					img_total[y*imgw[origin_ID] + x] = (img_data[i][y*imgw[origin_ID] + x] + img_data[i + 1][(y - translate_y)*imgw[origin_ID] + x]) / 2;
// 				}
// 			}
// 
// 		SaveImageFile(merge_result_path, (unsigned char *)img_total, imgw[origin_ID], imgHeight, 1, 16);
// 
// 		delete[]img_total; img_total = NULL;
// 	}
// 
// 	//compute corresponding rpc files
// 	//1. compute the header information of the final rpc
// 	RPC rpc;
// 	rpc.lineOffset = (imgHeight - 1) / 2;
// 	rpc.sampOffset = (imgw[origin_ID] - 1) / 2;
// 	rpc.lineScale = rpc.lineOffset + 1;
// 	rpc.sampScale = rpc.sampOffset + 1;
// 
// 	rpc.heightOffset = (rpcs[0].heightOffset + rpcs[1].heightOffset) / 2;
// 	rpc.heightScale = max(rpcs[0].heightScale, rpcs[1].heightScale);
// 
// 	double minHeight = rpc.heightOffset - rpc.heightScale;
// 	double maxHeight = rpc.heightOffset + rpc.heightScale;
// 
// 	double lat1[4], lon1[4];
// 	double lat2[4], lon2[4];
// 	RPC_Image2Ground(sy[0], sx[0], minHeight, rpcs[0], lat1[0], lon1[0]);
// 	RPC_Image2Ground(sy[0] + imgh[0], sx[0], minHeight, rpcs[0], lat1[1], lon1[1]);
// 	RPC_Image2Ground(sy[0] + imgh[0], sx[0] + imgw[origin_ID], minHeight, rpcs[0], lat1[2], lon1[2]);
// 	RPC_Image2Ground(sy[0], sx[0] + imgw[origin_ID], minHeight, rpcs[0], lat1[3], lon1[3]);
// 
// 	RPC_Image2Ground(sy[1], sx[1], minHeight, rpcs[1], lat2[0], lon2[0]);
// 	RPC_Image2Ground(sy[1] + imgh[1], sx[1], minHeight, rpcs[1], lat2[1], lon2[1]);
// 	RPC_Image2Ground(sy[1] + imgh[1], sx[1] + imgw[origin_ID], minHeight, rpcs[1], lat2[2], lon2[2]);
// 	RPC_Image2Ground(sy[1], sx[1] + imgw[origin_ID], minHeight, rpcs[1], lat2[3], lon2[3]);
// 
// 	double minLat, maxLat, minLon, maxLon;
// 	minLat = lat1[0]; maxLat = lat1[0];
// 	minLon = lon1[0]; maxLon = lon1[0];
// 	for (int i = 1; i < 4;i++)
// 	{
// 		if (minLat>lat1[i])
// 			minLat = lat1[i];
// 		if (maxLat < lat1[i])
// 			maxLat = lat1[i];
// 		if (minLon > lon1[i])
// 			minLon = lon1[i];
// 		if (maxLon < lon1[i])
// 			maxLon = lon1[i];
// 	}
// 	for (int i = 0; i < 4; i++)
// 	{
// 		if (minLat > lat2[i])
// 			minLat = lat2[i];
// 		if (maxLat < lat2[i])
// 			maxLat = lat2[i];
// 		if (minLon > lon2[i])
// 			minLon = lon2[i];
// 		if (maxLon < lon2[i])
// 			maxLon = lon2[i];
// 	}
// 
// 	rpc.latOffset = (minLat + maxLat) / 2;
// 	rpc.longOffset = (minLon + maxLon) / 2;
// 
// 	rpc.latScale = maxLat - rpc.latOffset;
// 	rpc.longScale = maxLon - rpc.longOffset;
// 
// 	//2. compute the corresponding GCPs and the image points
// 	int step_plane = 200;
// 	int step_height = 30;
// 	int imgWidth = imgw[origin_ID];
// 	int ptnum = (imgHeight / step_plane) * (imgWidth / step_plane)*((int)(maxHeight - minHeight)/step_height);
// 	int x, y;
// 	ptnum = 0;
// 	double h = 0;
// 	for (y = 0; y < imgHeight; y += step_plane)
// 		for (x = 0; x < imgw[origin_ID]; x += step_plane)
// 			for (h = minHeight; h <= maxHeight; h += step_height)
// 			{
// 				ptnum++;
// 			}
// 	double *px, *py;
// 	double *lat, *lon, *hei;
// 	px = new double[ptnum];
// 	py = new double[ptnum];
// 	lat = new double[ptnum];
// 	lon = new double[ptnum];
// 	hei = new double[ptnum];
// 
// 	int I = 0;
// 	h = 0;
// 	for (y = 0; y < imgHeight; y += step_plane)
// 		for (x = 0; x < imgw[origin_ID]; x += step_plane)
// 			for (h = minHeight; h <= maxHeight; h+=step_height)
// 			{
// 				px[I] = x;
// 				py[I] = y;
// 				hei[I] = h;
// 				if (y<translate_y)
// 				{
// 					RPC_Image2Ground(sy[0] + py[I], sx[origin_ID] + px[I], hei[I], rpcs[0], lat[I], lon[I]);
// 				}
// 				else if (y>imgh[0] - 1)
// 				{
// 					RPC_Image2Ground(sy[1] + py[I] - translate_y, sx[origin_ID] + px[I], hei[I], rpcs[1], lat[I], lon[I]);
// 					//img_total[y*imgw[origin_ID] + x] = img_data[i + 1][(y - translate_y)*imgw[origin_ID] + x];
// 				}
// 				else
// 				{
// 					double lat1_tmp, lon1_tmp;
// 					double lat2_tmp, lon2_tmp;
// 					RPC_Image2Ground(sy[0] + py[I], sx[origin_ID] + px[I], hei[I], rpcs[0], lat1_tmp, lon1_tmp);
// 					RPC_Image2Ground(sy[1] + py[I] - translate_y, sx[origin_ID] + px[I], hei[I], rpcs[1], lat2_tmp, lon2_tmp);
// 					lat[I] = (lat1_tmp + lat2_tmp) / 2;
// 					lon[I] = (lon1_tmp + lon2_tmp) / 2;
// 					//img_total[y*imgw[origin_ID] + x] = (img_data[i][y*imgw[origin_ID] + x] + img_data[i + 1][(y - translate_y)*imgw[origin_ID] + x]) / 2;
// 				}
// 
// 				I++;
// 			}
// 
// 	//3. finally compute the corresponding RPC parameters
// 	double maxd = 0;
// 	ComputeRPCfromVirtualGCPs(px, py, lat, lon, hei, ptnum, rpc, maxd);
// 
// 	//////////////////////////////////////////////////////////////////////////testing
// 	double tx, ty;
// 	tx = sx[origin_ID] + 5000;
// 	ty = sy[0] + 5000;
// 
// 	double lat_test, lon_test;
// 	RPC_Image2Ground(ty, tx, 0, rpcs[0], lat_test, lon_test);
// 
// 	double tx_test, ty_test;
// 	RPC_Ground2Image(lat_test, lon_test, 0, rpc, ty_test, tx_test);
// 	int aaa = 1;
// 	//////////////////////////////////////////////////////////////////////////
// 
// 	//free memory
// 	for (int i = 0; i < coverNum;i++)
// 	{
// 		delete[]img_data[i]; img_data[i] = NULL;
// 	}
// 	delete[]img_data; img_data = NULL;
// 	delete[]rpcs; rpcs = NULL;
// 	delete[]px; px = NULL;
// 	delete[]py; py = NULL;
// 	delete[]lat; lat = NULL;
// 	delete[]lon; lon = NULL;
// 	delete[]hei; hei = NULL;
// 	return 1;
// }

// int CBundleAdjust::Check_AOI_Overlap(char *imgPaths, char *rpcPaths, 
// 	double minX, double minY, double maxX, double maxY, char zoneID[10], double avgHei, 
// 	double &min_px, double &min_py, double &max_px, double &max_py, bool &ifcover)
// {
// 	//read image
// 	CImageBase img;
// 	int imgw, imgh;
// 	img.Open(imgPaths);
// 	imgw = img.GetCols();
// 	imgh = img.GetRows();
// 	img.Close();
// 
// 	//read rpc path
// 	RPC rpc;
// 	ReadRpc_singleFile(rpcPaths, rpc);
// 
// 	//avgHei = 200/*rpc.heightOffset*/;
// 
// 	//UTM to Lat, Lon
// 	double lat[4], lon[4];
// 	UTM2LL(minX, minY, zoneID, lat[0], lon[0]);
// 	UTM2LL(maxX, minY, zoneID, lat[1], lon[1]);
// 	UTM2LL(maxX, maxY, zoneID, lat[2], lon[2]);
// 	UTM2LL(minX, maxY, zoneID, lat[3], lon[3]);
// 
// 	//Project ground point onto images
// 	double row[4], col[4];
// 	for (int i = 0; i < 4;i++)
// 	{
// 		RPC_Ground2Image(lat[i], lon[i], avgHei, rpc, row[i], col[i]);
// 	}
// 
// 	ifcover = true;
// 	bool ifcover_row = false, ifcover_col = false;
// 	for (int i = 0; i < 4;i++)
// 	{
// 		if (row[i]>=0 && row[i]<=imgh - 1)
// 		{
// 			ifcover_row = true;
// 			break;
// 		}
// 	}
// 	for (int i = 0; i < 4; i++)
// 	{
// 		if (col[i] >= 0 && col[i] <= imgw - 1)
// 		{
// 			ifcover_col = true;
// 			break;
// 		}
// 	}
// 
// 	if (ifcover_row == true && ifcover_col == true)
// 	{
// 		ifcover = true;
// 	}
// 	else
// 	{
// 		ifcover = false;
// 		min_px = 0;
// 		min_py = 0;
// 		max_px = 0;
// 		max_py = 0;
// 	}
// 
// 	if (ifcover==true)
// 	{
// 		double min_row, max_row, min_col, max_col;
// 		min_row = row[0]; max_row = row[0];
// 		min_col = col[0]; max_col = col[0];
// 
// 		for (int i = 1; i < 4; i++)
// 		{
// 			if (min_row > row[i])
// 			{
// 				min_row = row[i];
// 			}
// 			if (max_row < row[i])
// 			{
// 				max_row = row[i];
// 			}
// 			if (min_col > col[i])
// 			{
// 				min_col = col[i];
// 			}
// 			if (max_col < col[i])
// 			{
// 				max_col = col[i];
// 			}
// 		}
// 
// 
// 		if (min_col < 0)
// 		{
// 			min_px = 0;
// 		}
// 		else
// 			min_px = min_col;
// 
// 		if (max_col > imgw - 1)
// 		{
// 			max_px = imgw - 1;
// 		}
// 		else
// 			max_px = max_col;
// 
// 		if (min_row < 0)
// 			min_py = 0;
// 		else
// 			min_py = min_row;
// 
// 		if (max_row > imgh - 1)
// 			max_py = imgh - 1;
// 		else
// 			max_py = max_row;
// 	}
// 	
// 
// 	return 1;
// }
// 
// int CBundleAdjust::Compute_AOI_IMG_RPC_From_Two_Patches(char imgPath[2][255], char rpcPath[2][255], 
// 	char *kmlPath, double extraBuffer,
// 	char *output_img_path, char *outout_rpc_path)
// {
// 	//Read KML file
// 	char zoneID[10];
// 	double minX, minY, maxX, maxY;
// 	double avgHei;
// 	ReadKML(kmlPath, minX, minY, maxX, maxY, avgHei, zoneID);
// 
// 	//create a little bit larger aoi
// 	minX -= extraBuffer;
// 	maxX += extraBuffer;
// 	minY -= extraBuffer;
// 	maxY += extraBuffer;
// 
// 	//check the overlap of image tiles and AOI
// 	int cover_Num = 2;
// 	int sx[2], sy[2], imgw[2], imgh[2];
// 	for (int i = 0; i < 2;i++)
// 	{
// 		double min_px = 0, min_py = 0, max_px = 0, max_py = 0;
// 		bool ifcover = false;
// 		Check_AOI_Overlap(imgPath[i], rpcPath[i], minX, minY, maxX, maxY, zoneID, avgHei,
// 			min_px, min_py, max_px, max_py, ifcover);
// 
// 		if (ifcover == true)
// 		{
// 			sx[i] = (int)(min_px + 0.5);
// 			sy[i] = (int)(min_py + 0.5);
// 			imgw[i] = (int)(max_px + 0.5) - (int)(min_px + 0.5) + 1;
// 			imgh[i] = (int)(max_py + 0.5) - (int)(min_py + 0.5) + 1;
// 		}
// 		else
// 			return 0;
// 	}
// 
// 	int origin_ID;
// 	for (int i = 0; i < 2; i++)
// 	{
// 		if (sy[i] != 0)
// 		{
// 			origin_ID = i;
// 			break;
// 		}
// 	}
// 
// 	//Merge the two tiles
// 	Merge_Img_Two_Tiles(imgPath, rpcPath, 
// 		sx, sy, imgw, imgh, origin_ID, zoneID,
// 		output_img_path, outout_rpc_path);
// 
// 	//free memory
// 
// 	return 1;
// }
// 
// int CBundleAdjust::Compute_AOI_IMG_RPC(char **imgPaths, char **rpcPaths, int imgNum, char *kmlPath, double extraBuffer,
// 	char *output_img_path, char *outout_rpc_path)
// {
// 	//Read KML file
// 	char zoneID[10];
// 	double minX, minY, maxX, maxY;
// 	double avgHei;
// 	ReadKML(kmlPath, minX, minY, maxX, maxY, avgHei, zoneID);
// 
// 	//create a little bit larger aoi
// 	minX -= extraBuffer;
// 	maxX += extraBuffer;
// 	minY -= extraBuffer;
// 	maxY += extraBuffer;
// 
// 	//check the overlap of image tiles and AOI
// 	int cover_Num = 0;
// 	for (int i = 0; i < imgNum;i++)
// 	{
// 		double min_px, min_py, max_px, max_py;
// 		bool ifcover;
// 		Check_AOI_Overlap(imgPaths[i], rpcPaths[i], minX, minY, maxX, maxY, zoneID, avgHei,
// 			min_px, min_py, max_px, max_py, ifcover);
// 
// 		if (ifcover==true)
// 		{
// 			cover_Num++;
// 		}
// 	}
// 
// 	if (cover_Num==0)
// 	{
// 		return 0;
// 	}
// 
// 	int *sx, *sy, *imgw, *imgh, *imgID;
// 	sx = new int[cover_Num];
// 	sy = new int[cover_Num];
// 	imgw = new int[cover_Num];
// 	imgh = new int[cover_Num];
// 	imgID = new int[cover_Num];
// 
// 	int I = 0;
// 	for (int i = 0; i < imgNum; i++)
// 	{
// 		double min_px, min_py, max_px, max_py;
// 		bool ifcover;
// 		Check_AOI_Overlap(imgPaths[i], rpcPaths[i], minX, minY, maxX, maxY, zoneID, avgHei,
// 			min_px, min_py, max_px, max_py, ifcover);
// 
// 		if (ifcover == true)
// 		{
// 			sx[I] = (int)(min_px+0.5); 
// 			sy[I] = (int)(min_py+0.5);
// 			imgw[I] = (int)(max_px+0.5) - (int)(min_px+0.5) + 1;
// 			imgh[I] = (int)(max_py + 0.5) - (int)(min_py + 0.5) + 1;
// 			imgID[I] = i;
// 			I++;
// 		}
// 	}
// 
// 	int origin_ID;
// 	for (int i = 0; i < cover_Num; i++)
// 	{
// 		if (sy[i]!=0)
// 		{
// 			origin_ID = i;
// 		}
// 	}
// 
// 
// 	//merge image tiles
// 	Merge_Img_Tiles(imgPaths, rpcPaths, cover_Num,
// 		sx, sy, imgw, imgh, imgID, origin_ID, zoneID,
// 		output_img_path, outout_rpc_path);
// 
// 	//compute corresponding RPC files
// 
// 
// 	//free memory
// 	delete[]sx; sx = NULL;
// 	delete[]sy; sy = NULL;
// 	delete[]imgw; imgw = NULL;
// 	delete[]imgh; imgh = NULL;
// 	delete[]imgID; imgID = NULL;
// 	
// 	return 1;
// }
// 
// //////////////////////////////////////////////////////////////////////////
// // 	double px2, py2;
// // 	px2 = 30853.941406;
// // 	py2 = 2563.935547;
// // 
// // 	RPC rpc1, rpc2;
// // 	ReadRpc_singleFile(rpcPaths[0], rpc1);
// // 	ReadRpc_singleFile(rpcPaths[1], rpc2);
// // 
// // 	double gx, gy, gz;
// // 	double lat, lon;
// // 	RPC_Image2Ground(py2, px2, 0, rpc2, lat, lon);
// // 
// // 	double px1, py1;
// // 	RPC_Ground2Image(lat, lon, 0, rpc1, py1, px1);
// // 	int aa = 1;
// // 		//////////////////////////////////////////////////////////////////////////
// 
// 
// void CBundleAdjust::jacobi(double *a, int n, double *d, double *v, int *nrot)
// {
// 	int j, iq, ip, ip_times_n, i;
// 	double tresh, theta, tau, t, sm, s, h, g, c, *b, *z, *vector();
// 
// 	b = (double *)malloc(sizeof(double) *n);
// 	z = (double *)malloc(sizeof(double) *n);
// 
// 
// 	for (ip_times_n = 0, ip = 0; ip < n; ++ip, ip_times_n += n)
// 	{
// 
// 		for (iq = 0; iq < n; ++iq)v[ip_times_n + iq] = 0.0;
// 		v[ip_times_n + ip] = 1.0;
// 
// 		b[ip] = d[ip] = a[ip_times_n + ip];
// 		z[ip] = 0.0;
// 	}
// 
// 	*nrot = 0;
// 	for (i = 0; i < 50; ++i)
// 	{
// 		sm = 0.0;
// 
// 		for (ip_times_n = 0, ip = 0; ip < n - 1; ip++, ip_times_n += n)
// 			for (iq = ip + 1; iq < n; iq++)
// 				sm += fabs(a[ip_times_n + iq]);
// 
// 		if (sm == 0.0)
// 		{
// 			free(b);
// 			free(z);
// 			return;
// 		}
// 
// 		tresh = (i < 3) ? 0.2*sm / (n*n) : 0.0;
// 
// 		for (ip_times_n = 0, ip = 0; ip < n - 1; ip++, ip_times_n += n)
// 		{
// 			for (iq = ip + 1; iq < n; ++iq)
// 			{
// 				g = 100.0*fabs(a[ip_times_n + iq]);
// 
// 				if (i > 3 && g < EPS)
// 					a[ip_times_n + iq] = 0.0;
// 
// 				else if (fabs(a[ip_times_n + iq]) > tresh)
// 				{
// 					h = d[iq] - d[ip];
// 					if (g < EPS)
// 						t = (fabs(a[ip_times_n + iq]) > EPS) ? (a[ip_times_n + iq]) / h : 0.0;
// 					else
// 					{
// 						theta = (fabs(h) < EPS) ? 0.0 : 0.5*h / (a[ip_times_n + iq]);
// 						t = 1.0 / (fabs(theta) + sqrt(1.0 + theta*theta));
// 						if (theta < 0.0)
// 							t = -t;
// 					}
// 					c = 1.0 / sqrt(1.0 + t*t);
// 					s = t*c;
// 					tau = s / (1.0 + c);
// 
// 					h = t*a[ip_times_n + iq];
// 					z[ip] -= h;
// 					z[iq] += h;
// 					d[ip] -= h;
// 					d[iq] += h;
// 					a[ip_times_n + iq] = 0.0;
// 
// 					for (j = 0; j < ip; j++)
// 					{
// 						ROTATE(a, j, ip, j, iq, n);
// 					}
// 					for (j = ip + 1; j < iq; j++)
// 					{
// 						ROTATE(a, ip, j, j, iq, n);
// 					}
// 					for (j = iq + 1; j < n; j++)
// 					{
// 						ROTATE(a, ip, j, iq, j, n);
// 					}
// 					for (j = 0; j < n; j++)
// 					{
// 						ROTATE(v, j, ip, j, iq, n);
// 					}
// 					++(*nrot);
// 				}
// 			}
// 		}
// 		for (ip = 0; ip < n; ++ip)
// 		{
// 			b[ip] += z[ip];
// 			d[ip] = b[ip];
// 			z[ip] = 0.0;
// 		}
// 	}
// 
// 	free(b);
// 	free(z);
// 	return;
// 
// }
// int CBundleAdjust::ExpandGroundOffSetByTwoOrderEquation(double *offset, int num)
// {
// 	int start, end;
// 	int i;
// 	for (i = 0; i < num;i++)
// 	{
// 		if (offset[i]!=0)
// 		{
// 			start = i;
// 			break;
// 		}
// 	}
// 
// 	for (i = num - 1; i >= 0;i--)
// 	{
// 		if (offset[i]!=0)
// 		{
// 			end = i;
// 			break;
// 		}
// 	}
// 
// 	int validNum = end - start + 1;
// 	double *A, *L, *X;
// 	int varNum = 3;
// 	A = new double[validNum*varNum];
// 	L = new double[validNum];
// 	X = new double[varNum];
// 
// 	for (i = start; i <= end;i++)
// 	{
// 		A[(i - start)*varNum + 0] = i*i;
// 		A[(i - start)*varNum + 1] = i;
// 		A[(i - start)*varNum + 2] = 1;
// 
// 		L[i - start] = offset[i];
// 	}
// 
// 	double alpha = 0;
// 	LeastSquares(A, L, X, validNum, varNum, alpha);
// 
// 	double a, b, c;
// 	a = X[0]; b = X[1]; c = X[2];
// 
// 	for (i = 0; i < start;i++)
// 	{
// 		offset[i] = a*i*i + b*i + c;
// 	}
// 	for (i = end + 1; i < num;i++)
// 	{
// 		offset[i] = a*i*i + b*i + c;
// 	}
// 
// 	//free memory
// 	delete[]A; A = NULL;
// 	delete[]L; L = NULL;
// 	delete[]X; X = NULL;
// 	return 1;
// }