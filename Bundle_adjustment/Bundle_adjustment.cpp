// Bundle_adjustment.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "BundleAdjust.h"
#include "LeastSquareMatching.h"
#include "time.h"
#define PI 3.14159265359
#include "PixelLabIGF.h"
#include "Simple_Matrix_Computation.h"
#include "direct.h"
#include "tinyxml.h"
#include "tinystr.h"
#include <io.h>

#define m_minH -3000
#define m_maxH 9000

typedef struct  
{
	double x, y, d;
	double P;
	double deltaC, deltaS, deltaD;
}PixelInfo;

typedef struct  
{
	char img_path[512];
	char dsm_path[512];
	int window_size;
	int iteration_times;
	char output_path[512];
	int tile_x_size;
	int tile_y_size;
	int overlsp_size;
	char filtering_methods[256];

	float std_color, std_distance, std_disparity;//common parameter that are used in most filtering methods 
	float color_weight, disp_weight, dis_weight; //unique parameters that are only used in the local patch
}Para;

int Tree_filtering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double sigma_C = 6);
int MLPA_filtering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double &time, double er = 0.01, double sigma_C = 10);
int BialteringFiltering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double sig_S, double sig_C);
int GuidedMedian_Filtering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double sig_S, double sig_C);
int LocalPatchFiltering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double P_coeff,
	double sig_C, double sig_S, double sig_D, int iteration_times);
int LocalPatchFiltering_AdaptWin(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int PatchNum, double P_coeff,
	double sig_C, double sig_S, double sig_D, int iteration_times);
int LocalPatchFiltering_AdaptWin_slow(int blockID, int blockNum, float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int PatchNum,
	double P_coeff, double sig_C, double sig_S, double sig_D, int iteration_times, float w_color, float w_dis, float w_disp,
	double &running_time);
int compute_abc(PixelInfo *array, int ptnum, double &a, double &b, double &c);
int DefineAdaptWin(int cx, int cy, int PatchNum, float *dsm_data, BYTE *img_data, int w, int h, int bands, double w_dis, double w_color, double w_disp,
	PixelInfo *adaptWin, int &validNum, PixelInfo *neighbors);
int DefineAdaptWin_fst(int PatchNum, float *dsm_data, BYTE *img_data, int w, int h, int bands, double w_dis, double w_color, double w_disp,
	PixelInfo **adaptWin, int *validNum);
int CheckIfRepeat(int x, int y, vector<PixelInfo> array);
int CheckIfRepeat(int x, int y, PixelInfo *array, int ptNum);

int Compute_horizontal_vertical_weights(BYTE *img_data, int w, int h, int bands, int sx, int sy, int ex, int ey, float sigC, double &W);

int Compute_Local_Polynomial_Coeff(double *x, double *y, double *disp_win, double *intensity_win, double *weight_win, double wr, int validNum,
	double *a, double &gama, double &beta);

int Get_DispRange(float *disp_data, int w, int h, int &minD, int &maxD);

int Tree_filtering_all_dispRange(BYTE *img, int color, float *cost, int width, int height, int dispRange,
	float sigC, int minD);

int fst_bilaterla_filtering_single_direction_all_DispRang(
	BYTE *img, int width, int height, int color, 
	int minD, int DispRange, float sigC,
	int dx, int dy, float *C, float *S
	);

int write_empty_prj(char *exe_path);
int Read_prj(const char *prj_path, Para &para);

int DSM_filtering_blockwise(Para paras);

//read and write xml
int create_empty_filter_prj(const char *exe_path);
int read_filter_prj(const char *prj_path, Para &paras);
int check_end_of_path(char *path);

//guided median filtering
void WeightedSort_Med(float *cost, float *w, int Num, float &med, int scale, float *array);
void Swap(float &a, float &b);
void BubbleSort(float *arr, int size);

int m_block_num, m_cur_block;
int _tmain(int argc, _TCHAR* argv[])
{
	if (argc!=2)
	{
		printf("The input format is wrong\n");
		printf("The correct format should be\n");
		printf("exe_path input_prj_path\n");
		create_empty_filter_prj(argv[0]);
		return 0;
	}

	CFileOperation file;
	char  *prj_path = argv[1];
	Para paras;
	//Read_prj(prj_path, paras);
	read_filter_prj(prj_path, paras);
	DSM_filtering_blockwise(paras);

	return 0;



	//////////////////////////////////////////////////////////////////////////

	return 0;
}

//read and write xml
int create_empty_filter_prj(const char *exe_path)
{
	CFileOperation file;
	char prj_path[512];
	file.FindDirPath_prj(exe_path, prj_path, false);

	strcat(prj_path, "\\filter_prj.xml");

	try
	{
		//create xml object
		TiXmlDocument *myDocument = new TiXmlDocument();

		TiXmlElement *RootElement = new TiXmlElement("Main_prj_file");
		myDocument->LinkEndChild(RootElement);

		//define output folder path
		TiXmlElement *output_folder_element = new TiXmlElement("Output_Folder");
		RootElement->LinkEndChild(output_folder_element);
		TiXmlText  *Output_Folder_path_element = new TiXmlText("");
		output_folder_element->LinkEndChild(Output_Folder_path_element);
 
		//define input DSM path
		TiXmlElement *input_DSM_element = new TiXmlElement("DSM_path");
		RootElement->LinkEndChild(input_DSM_element);
		TiXmlText  *dsm_path_element = new TiXmlText("");
		input_DSM_element->LinkEndChild(dsm_path_element);

		//define ortho image path
		TiXmlElement *Ortho_Image_element = new TiXmlElement("Ortho_path");
		RootElement->LinkEndChild(Ortho_Image_element);
		TiXmlText  *Ortho_path_element = new TiXmlText("");
		Ortho_Image_element->LinkEndChild(Ortho_path_element);
// 
// 		//define obj file path
// 		TiXmlElement *obj_element = new TiXmlElement("Obj_Model_Path");
// 		RootElement->LinkEndChild(obj_element);
// 		TiXmlText  *obj_path_element = new TiXmlText("");
// 		obj_element->LinkEndChild(obj_path_element);

		//define texture mapping related parameters
		TiXmlElement *filter_para_element = new TiXmlElement("Filtering_parameters");
		RootElement->LinkEndChild(filter_para_element);

		TiXmlElement *window_size_element = new TiXmlElement("window_size"); //choose the mapping mode
		filter_para_element->LinkEndChild(window_size_element);
		TiXmlText  *mode_para = new TiXmlText("25");
		window_size_element->LinkEndChild(mode_para);

		TiXmlElement *iteration_element = new TiXmlElement("iteration_times"); //choose the mapping mode
		filter_para_element->LinkEndChild(iteration_element);
		TiXmlText  *iterateion_para = new TiXmlText("5");
		iteration_element->LinkEndChild(iterateion_para);

		TiXmlElement *tile_size_element = new TiXmlElement("tile_size");
		filter_para_element->LinkEndChild(tile_size_element);
		tile_size_element->SetAttribute("tile_x_size", "1000");
		tile_size_element->SetAttribute("tile_y_size", "1000");
		tile_size_element->SetAttribute("overlap_size", "100");

		TiXmlComment  *filtering_method_comment = new TiXmlComment("Filtering methods include bilateral, guided_image, MLPA, tree_filter, guided_median and local_patch");
		filter_para_element->LinkEndChild(filtering_method_comment);

		TiXmlElement *method_element = new TiXmlElement("filtering_methods"); //choose the mapping mode
		filter_para_element->LinkEndChild(method_element);
		TiXmlText  *method_para = new TiXmlText("patch_filtering");
		method_element->LinkEndChild(method_para);

// 		TiXmlComment  *common_para_comment = new TiXmlComment("Common filtering parameters for different filtering methods, -1 means adaptively or default setting these parameters ");
// 		filter_para_element->LinkEndChild(common_para_comment);

		TiXmlElement *common_para_element = new TiXmlElement("common_paras");
		filter_para_element->LinkEndChild(common_para_element);
		common_para_element->SetAttribute("Std_color", "-1");
		common_para_element->SetAttribute("Std_distance", "-1");
		common_para_element->SetAttribute("Std_disparity", "-1");

		TiXmlComment  *patch_para_comment = new TiXmlComment("The following parameters only work on the window size definition of patch filtering method");
		filter_para_element->LinkEndChild(patch_para_comment);

		TiXmlElement *patch_para_element = new TiXmlElement("patch_filter_paras");
		filter_para_element->LinkEndChild(patch_para_element);
		patch_para_element->SetAttribute("color_weight", "0.3");
		patch_para_element->SetAttribute("distance_weight", "0.3");
		patch_para_element->SetAttribute("disparity_weight", "0.6");

		myDocument->SaveFile(prj_path);//save file
	}
	catch (string& e)
	{
		return 0;
	}
	return 1;

	return 1;

	return 1;
}

int read_filter_prj(const char *prj_path, Para &paras)
{
	//read prj xml file
	TiXmlDocument *myDocument = new TiXmlDocument(prj_path);
	myDocument->LoadFile();
	TiXmlElement *RootElement = myDocument->RootElement();

	//output folder path
	TiXmlElement *Output_Folder_Node = RootElement->FirstChildElement();
	if (Output_Folder_Node->GetText() == NULL)
	{
		strcpy(paras.output_path, "");
		printf("The output_folder_path cannot be empty\n");
		return 0;
	}
	else
		strcpy(paras.output_path, Output_Folder_Node->GetText());
	check_end_of_path(paras.output_path);
	_mkdir(paras.output_path);

	if (_access(paras.output_path, 0) == -1)
	{
		printf("Cannot find output folder\n");
		return 0;
	}

	//dsm path
	TiXmlElement *dsm_path_Node = Output_Folder_Node->NextSiblingElement();
	if (dsm_path_Node->GetText() == NULL)
	{
		strcpy(paras.dsm_path, "");
		printf("Cannot find DSM path\n");
		return 0;
	}
	else
		strcpy(paras.dsm_path, dsm_path_Node->GetText());
	//check_end_of_path(paras.dsm_path);

	//ortho image path
	TiXmlElement *Ortho_Image_Node = dsm_path_Node->NextSiblingElement();
	if (Ortho_Image_Node->GetText() == NULL)
	{
		strcpy(paras.img_path, "");
		printf("Cannot find Ortho path\n");
		return 0;
	}
	else
		strcpy(paras.img_path, Ortho_Image_Node->GetText());
	//check_end_of_path(paras.img_path);

	//filtering parameters
	//window size
	TiXmlElement *parameter_Node = Ortho_Image_Node->NextSiblingElement();
	TiXmlElement *window_size_node = parameter_Node->FirstChildElement();
	if (window_size_node->GetText() == NULL)
	{
		paras.window_size = 25;
	}
	else
		paras.window_size = atoi(window_size_node->GetText());

	//iteration times
	TiXmlElement *iteration_node = window_size_node->NextSiblingElement();
	if (iteration_node->GetText() == NULL)
	{
		paras.iteration_times = 5;
	}
	else
		paras.iteration_times = atoi(iteration_node->GetText());

	//tile sizes
	TiXmlElement *tile_size_node = iteration_node->NextSiblingElement();
	TiXmlAttribute *tile_x_attr = tile_size_node->FirstAttribute();
	paras.tile_x_size = atoi(tile_x_attr->Value());
	TiXmlAttribute *tile_y_attr = tile_x_attr->Next();
	paras.tile_y_size = atoi(tile_y_attr->Value());
	TiXmlAttribute *overlap_attr = tile_y_attr->Next();
	paras.overlsp_size = atoi(overlap_attr->Value());

	if (paras.tile_x_size <= 0 || paras.tile_y_size <= 0 || paras.overlsp_size <= 0)
	{
		printf("tile sizes cannot be smaller than zero\n");
		return 0;
	}

	if (paras.tile_x_size <= paras.overlsp_size || paras.tile_y_size <= paras.overlsp_size)
	{
		printf("tile sizes cannot be smaller than the overlap sizes\n");
		return 0;
	}

	//filtering method
	TiXmlElement *filtering_method_node = tile_size_node->NextSiblingElement();
	if (filtering_method_node->GetText() == NULL)
	{
		strcpy(paras.filtering_methods, "patch_filtering");
	}
	else
		strcpy(paras.filtering_methods, filtering_method_node->GetText());

	//common_paras
	TiXmlElement *common_paras_node = filtering_method_node->NextSiblingElement();
	TiXmlAttribute *std_color_attr = common_paras_node->FirstAttribute();
	paras.std_color = atof(std_color_attr->Value());
	TiXmlAttribute *std_dis_attr = std_color_attr->Next();
	paras.std_distance = atof(std_dis_attr->Value());
	TiXmlAttribute *std_disp_attr = std_dis_attr->Next();
	paras.std_disparity = atof(std_disp_attr->Value());

	if (paras.std_color <= 0 || paras.std_distance <= 0 || paras.std_disparity <= 0)
	{
		printf("The corresponding std. values cannot be negative. The default value will be used in the filtering\n");
		double sig_S_flt = sqrt(paras.window_size) / 2;
		double sig_C_flt = 10;
		double sig_D_flt = 1;

		paras.std_color = sig_C_flt;
		paras.std_distance = sig_S_flt;
		paras.std_disparity = sig_D_flt;
	}

	//patch_filter_paras
	TiXmlElement *patch_filter_node = common_paras_node->NextSiblingElement();
	TiXmlAttribute *color_weight_attr = patch_filter_node->FirstAttribute();
	paras.color_weight = atof(color_weight_attr->Value());
	TiXmlAttribute *dis_weight_attr = color_weight_attr->Next();
	paras.dis_weight = atof(dis_weight_attr->Value());
	TiXmlAttribute *disp_weight_attr = dis_weight_attr->Next();
	paras.disp_weight = atof(disp_weight_attr->Value());

	if (paras.color_weight <= 0 || paras.dis_weight <= 0 || paras.disp_weight <= 0)
	{
		printf("The weight cannot be negative. The default value will be used in the filtering\n");
		paras.color_weight = 0.3;
		paras.dis_weight = 0.3;
		paras.disp_weight = 0.6;
	}

	delete myDocument; myDocument = NULL;

	return 1;
}

int check_end_of_path(char *path)
{
	int len = strlen(path);
	if (path[len - 1] == '\\')
	{
		path[len - 1] = '\0';
	}
	return 1;
}

int DSM_filtering_blockwise(Para paras)
{
	//read image size information
	CImageBase img;
	img.Open(paras.img_path);
	int w = img.GetCols();
	int h = img.GetRows();
	int bands = img.GetBands();
	img.Close();

	//0. define output path
	char *output_folder_path = paras.output_path;
	_mkdir(output_folder_path);
	strcat(output_folder_path, "\\");
	char output_path[512];
	char output_xyz_path[512];
	char time_path[512];
	strcpy(output_path, output_folder_path);
	strcpy(output_xyz_path, output_folder_path);
	strcpy(time_path, output_folder_path);

	CFileOperation file;
	char dsm_name[100];
	file.GetFileName(paras.dsm_path, dsm_name);
	strcat(output_path, dsm_name);
	strcat(output_xyz_path, dsm_name);
	strcat(time_path, dsm_name);
	strcat(output_path, "_");
	strcat(output_xyz_path, "_");
	strcat(time_path, "_");
	strcat(output_path, paras.filtering_methods);
	strcat(output_xyz_path, paras.filtering_methods);
	strcat(time_path, paras.filtering_methods);
	strcat(output_path, ".tif");
	strcat(output_xyz_path, ".xyz");
	strcat(time_path, ".txt");

	//1. divide blocks
	int startx = 0, starty = 0;
	int endx = startx + paras.tile_x_size - 1;
	int endy = starty + paras.tile_y_size - 1;
	if (endx > w - 1) endx = w - 1;
	if (endy > h - 1) endy = h - 1;

	vector <int>sx, sy;
	vector <int> tile_w, tile_h;
	while (endy <= h - 1)
	{
		while (endx <= w - 1)
		{
			sx.push_back(startx);
			sy.push_back(starty);
			int tw, th;
			tw = endx - startx + 1;
			th = endy - starty + 1;
			tile_w.push_back(tw);
			tile_h.push_back(th);

			if (endx == w - 1)
			{
				break;
			}
			else
			{
				startx = endx + 1 - paras.overlsp_size;
				endx = startx + paras.tile_x_size - 1;
				if (endx > w - 1) endx = w - 1;
			}
		}

		if (endy == h - 1)
		{
			break;
		}
		else
		{
			startx = 0;
			endx = paras.tile_x_size - 1;
			if (endx > w - 1) endx = w - 1;
			starty = endy + 1 - paras.overlsp_size;
			endy = starty + paras.tile_y_size - 1;
			if (endy > h - 1) endy = h - 1;
		}
	}

	//2. blockwise processing
	double sig_S_flt = paras.std_distance;
	double sig_C_flt = paras.std_color;
	double sig_D_flt = paras.std_disparity;
	double P_flt = 1.0;

	int blockNum = sx.size();
	m_block_num = blockNum;
	printf("divide %d blocks\n", blockNum);
	float *dsm_data = new float[paras.tile_x_size*paras.tile_y_size];
	float *dsm_data_backup = new float[paras.tile_x_size*paras.tile_y_size];
	BYTE *img_data = new BYTE[paras.tile_x_size*paras.tile_y_size*bands];
	CImageBase img_out;
	img_out.Create(output_path, w, h, 1, 32);

	int bial_size = (int)(sqrt(paras.window_size) + 0.5);
	if (bial_size % 2 == 0) bial_size += 1;

	clock_t startTime, endTime;
	startTime = clock();
	double time = 0;
	for (int i = 0; i < blockNum; i++)
	{
		//read DSM
		m_cur_block = i + 1;
		img.Open(paras.dsm_path);
		img.Read(dsm_data, sx[i], sy[i], tile_w[i], tile_h[i]);
		img.Close();

		//forcelly define the invlade values in the orginal DSM as NAN
		for (int y = 0; y < tile_h[i]; y++)
			for (int x = 0; x < tile_w[i]; x++)
			{
				float td = dsm_data[y*tile_w[i] + x];
				if (td < m_minH || td > m_maxH)
				{
					dsm_data[y*tile_w[i] + x] = NAN;
				}
			}

		//read ortho
		img.Open(paras.img_path);
		img.Read(img_data, sx[i], sy[i], tile_w[i], tile_h[i]);
		img.Close();

		if (strcmp(paras.filtering_methods, "bilateral") == 0)
		{
			printf("Bilateral filtering process...%d/%d block | \r", i + 1, blockNum);
			for (int j = 0; j < paras.iteration_times; j++)
			{
				printf("Bilateral filtering process...%d/%d block | %dth iteration\r", i + 1, blockNum, j + 1);
				BialteringFiltering(dsm_data, img_data, tile_w[i], tile_h[i], bands, bial_size, sig_S_flt, sig_C_flt);
			}
		}
		else if (strcmp(paras.filtering_methods, "guided_image") == 0)
		{
			printf("guided image filtering process...%d/%d block | \r", i + 1, blockNum);
			BYTE *ortho_gudied = new BYTE[tile_h[i] * tile_w[i] * bands];
			for (int y = 0; y < tile_h[i]; y++)
				for (int x = 0; x < tile_w[i]; x++)
				{
					for (int k = 0; k < bands; k++)
					{
						ortho_gudied[y*tile_w[i] * bands + x*bands + k] = img_data[k*tile_w[i] * tile_h[i] + y*tile_w[i] + x];
					}
				}
			for (int j = 0; j < paras.iteration_times; j++)
			{
				printf("guided image filtering process...%d/%d block | %dth iteration\r", i + 1, blockNum, j + 1);
				ImageGuidedFilter(dsm_data, tile_w[i], tile_h[i], ortho_gudied, bands, bial_size);
			}
			delete[]ortho_gudied; ortho_gudied = NULL;
		}
		else if (strcmp(paras.filtering_methods, "MLPA") == 0)
		{
			printf("MLPA filtering process...%d/%d block | \r", i + 1, blockNum);
			for (int j = 0; j < paras.iteration_times; j++)
			{
				printf("MLPA filtering process...%d/%d block | %dth iteration\r", i + 1, blockNum, j + 1);
				double cur_time = 0;
				MLPA_filtering(dsm_data, img_data, tile_w[i], tile_h[i], bands, bial_size, cur_time);
			}
		}
		else if (strcmp(paras.filtering_methods, "tree_filter") == 0)
		{
			printf("tree filtering process...%d/%d block\r", i + 1, blockNum);
			Tree_filtering(dsm_data, img_data, tile_w[i], tile_h[i], bands, bial_size, paras.std_color);
		}
		else if (strcmp(paras.filtering_methods, "guided_median") == 0)
		{
			printf("Guided median filtering process...%d/%d block | \r", i + 1, blockNum);
			for (int j = 0; j < paras.iteration_times; j++)
			{
				printf("Guided median filtering process...%d/%d block | %dth iteration\r", i + 1, blockNum, j + 1);
				GuidedMedian_Filtering(dsm_data, img_data, tile_w[i], tile_h[i], bands, bial_size, sig_S_flt, sig_C_flt);
			}
		}
		else
		{
			printf("Local Patch filtering process...%d/%d block | \r", i + 1, blockNum);
			LocalPatchFiltering_AdaptWin_slow(i+1, blockNum, dsm_data, img_data, tile_w[i], tile_h[i], bands, paras.window_size,
				P_flt, sig_C_flt, sig_S_flt, sig_D_flt, paras.iteration_times, paras.color_weight, paras.dis_weight, paras.disp_weight,
				time);
			//printf("\n");
		}

		if (sx[i] == 0 && sy[i] == 0)
		{
			memcpy(dsm_data_backup, dsm_data, sizeof(float)*tile_w[i] * tile_h[i]);
			img_out.Write(dsm_data_backup, sx[i], sy[i], tile_w[i], tile_h[i]);
		}
		else if (sx[i] == 0)
		{
			for (int y = paras.overlsp_size / 2; y < tile_h[i]; y++)
				for (int x = 0; x < tile_w[i]; x++)
				{
					dsm_data_backup[(y - paras.overlsp_size / 2)*tile_w[i] + x] = dsm_data[y*tile_w[i] + x];
				}
			img_out.Write(dsm_data_backup, sx[i], sy[i] + paras.overlsp_size / 2, tile_w[i], tile_h[i] - paras.overlsp_size / 2);
		}
		else if (sy[i] == 0)
		{
			for (int y = 0; y < tile_h[i]; y++)
				for (int x = paras.overlsp_size / 2; x < tile_w[i]; x++)
				{
					dsm_data_backup[y*(tile_w[i] - paras.overlsp_size/2) + (x - paras.overlsp_size/2)] 
						= dsm_data[y*tile_w[i] + x];
				}
			//img_out.Write(dsm_data_backup, sx[i], sy[i] + paras.overlsp_size / 2, tile_w[i], tile_h[i] - paras.overlsp_size / 2);
			img_out.Write(dsm_data_backup, sx[i] + paras.overlsp_size / 2, sy[i], tile_w[i] - paras.overlsp_size / 2, tile_h[i]);
		}
		else
		{
			for (int y = paras.overlsp_size / 2; y < tile_h[i]; y++)
				for (int x = paras.overlsp_size / 2; x < tile_w[i]; x++)
				{
					dsm_data_backup[(y - paras.overlsp_size / 2)*(tile_w[i] - paras.overlsp_size / 2) + (x - paras.overlsp_size / 2)]
						= dsm_data[y*tile_w[i] + x];
				}
			img_out.Write(dsm_data_backup, sx[i] + paras.overlsp_size / 2, sy[i] + paras.overlsp_size / 2,
				tile_w[i] - paras.overlsp_size / 2, tile_h[i] - paras.overlsp_size / 2);
		}
	}
	img_out.Close();
	printf("\n");

	endTime = clock();
	printf("\n");
	printf("The running time of DSM filtering is %lf second\n", (double)(endTime - startTime) / CLOCKS_PER_SEC);

	time = (double)(endTime - startTime) / CLOCKS_PER_SEC;

	//output xya file and the time file
	//FILE *fp_xyz = fopen(output_xyz_path, "w");
	//for (int y = 0; y < h; y++)
	//	for (int x = 0; x < w; x++)
	//	{
	//		float td = DSM_inverse[y*w + x];
	//		if (isnan(td) == false)
	//		{
	//			fprintf(fp_xyz, "%d %d %f\n", x, h - 1 - y, td);
	//		}
	//	}
	//fclose(fp_xyz); fp_xyz = NULL;

	FILE *fp_time = fopen(time_path, "w");
	fprintf(fp_time, "%lf\n", time);
	fclose(fp_time);

	//free memory
	delete[]dsm_data; dsm_data = NULL;
	delete[]img_data; img_data = NULL;
	delete[]dsm_data_backup; dsm_data_backup = NULL;

	return 1;
}

int Read_prj(const char *prj_path, Para &para)
{
	FILE *fp = fopen(prj_path, "r");
	while (!feof(fp))
	{
		char str[512];
		fgets(str, 512, fp);
		if (strcmp(str, "DSM_path\n") == 0)
		{
			fgets(str, 512, fp);
			sscanf(str, "%s", para.dsm_path);
		}
		else if (strcmp(str, "img_path\n") == 0)
		{
			fgets(str, 512, fp);
			sscanf(str, "%s", para.img_path);
		}
		else if (strcmp(str, "window_size\n") == 0)
		{
			fgets(str, 512, fp);
			sscanf(str, "%d",& para.window_size);
		}
		else if (strcmp(str, "iteration_times\n") == 0)
		{
			fgets(str, 512, fp);
			sscanf(str, "%d", &para.iteration_times);
		}
		else if (strcmp(str, "output_dsm_path\n") == 0)
		{
			fgets(str, 512, fp);
			sscanf(str, "%s", para.output_path);
		}
		else if (strcmp(str, "tile_x_size\n") == 0)
		{
			fgets(str, 512, fp);
			sscanf(str, "%d", &para.tile_x_size);
		}
		else if (strcmp(str, "tile_y_size\n") == 0)
		{
			fgets(str, 512, fp);
			sscanf(str, "%d", &para.tile_y_size);
		}
		else if (strcmp(str, "overlsp_size\n") == 0)
		{
			fgets(str, 512, fp);
			sscanf(str, "%d", &para.overlsp_size);
		}
	}

	return 1;
}

int write_empty_prj(char *exe_path)
{
	CFileOperation file;
	char prj_path[512];
	file.FindDirPath_prj(exe_path, prj_path, false);

	strcat(prj_path, "\\prj.txt");
	FILE *fp = fopen(prj_path, "w");

	fprintf(fp, "DSM_path\n");
	fprintf(fp, "NULL\n");
	fprintf(fp, "img_path\n");
	fprintf(fp, "NULL\n");
	fprintf(fp, "window_size\n");
	fprintf(fp, "NULL\n");
	fprintf(fp, "iteration_times\n");
	fprintf(fp, "NULL\n");
	fprintf(fp, "output_dsm_path\n");
	fprintf(fp, "NULL\n");
	fprintf(fp, "tile_x_size\n");
	fprintf(fp, "NULL\n");
	fprintf(fp, "tile_y_size\n");
	fprintf(fp, "NULL\n");
	fprintf(fp, "overlsp_size\n");
	fprintf(fp, "NULL\n");

	fclose(fp);

	return 1;
}

int Get_DispRange(float *disp_data, int w, int h, int &minD, int &maxD)
{
	for (int i = 0; i < w*h; i++)
	{
		float curd = disp_data[i];
		if (isnan(curd) == false && curd>m_minH && curd < m_maxH)
		{
			minD = curd;
			maxD = curd;
			break;
		}
	}

	for (int i = 0; i < w*h; i++)
	{
		float curd = disp_data[i];
		if (isnan(curd) == false && curd>m_minH && curd < m_maxH)
		{
			if (minD > curd) minD = curd;
			if (maxD < curd) maxD = curd;
		}
	}

	return 1;
}

int fst_bilaterla_filtering_single_direction_all_DispRang(
	BYTE *img, int width, int height, int color,
	int minD, int DispRange, float sigC,
	int dx, int dy, float *C, float *S
	)
{
	//Â·¾¶ËÑË÷·¶Î§ <= <=
	int startx, endx;
	int starty, endy;

	//ËÑË÷Æðµã·¶Î§ <= <
	int sPx, ePx;
	int sPy, ePy;
	int px, py; //Æðµã±éÀú²½¾à

	int x, y, d;
	if (dx == 1 && dy == 0) //left to right
	{
		startx = 0; endx = width - 1; starty = -1; endy = -1; sPx = -1; ePx = -1; sPy = 0; ePy = height; px = 0; py = 1;
	}
	else if (dx == -1 && dy == 0) // right to left
	{
		startx = width - 1; endx = 0; starty = -1; endy = -1; sPx = -1; ePx = -1; sPy = 0; ePy = height; px = 0; py = 1;
	}
	else if (dx == 0 && dy == 1) // top to bottom
	{
		startx = -1; endx = -1; starty = 0; endy = height - 1; sPx = 0; ePx = width; sPy = -1; ePy = -1; px = 1; py = 0;
	}
	else if (dx == 0 && dy == -1) //bottom to top
	{
		startx = -1; endx = -1; starty = height - 1; endy = 0; sPx = 0; ePx = width; sPy = -1; ePy = -1; px = 1; py = 0;
	}
	else if (dx == 1 && dy == 1) //diagonal direction 1
	{
		startx = 0; endx = width - 1; starty = 0; endy = height - 1; sPx = 0; ePx = width; sPy = 1; ePy = height; px = 1; py = 1;
	}
	else if (dx == 1 && dy == -1) //diagonal direction 2
	{
		startx = 0; endx = width - 1; starty = height - 1; endy = 0; sPx = 0; ePx = width; sPy = height - 2; ePy = -1; px = 1; py = -1;
	}
	else if (dx == -1 && dy == 1) //diagonal direction 3
	{
		startx = width - 1; endx = 0; starty = 0; endy = height - 1; sPx = width - 1; ePx = -1; sPy = 1; ePy = height; px = -1; py = 1;
	}
	else if (dx == -1 && dy == -1)  //diagonal direction 4
	{
		startx = width - 1; endx = 0; starty = height - 1; endy = 0; sPx = width - 1; ePx = -1; sPy = height - 2; ePy = -1; px = -1; py = -1;
	}
	else return 0;

	//build color table
	int k;
	float w_c[256];
	for (k = 0; k < 256; k++)
	{
		w_c[k] = (float)exp(-(k / sigC)*(k / sigC));
	}

	float *preLr;//store cost of previous pixel
	preLr = new float[DispRange];
	memset(preLr, 0, sizeof(float)*DispRange);
	float *Lr;
	Lr = new float[DispRange];
	memset(Lr, 0, sizeof(float)*DispRange);

	double deltaColor;
	int pos;
	float T = 0;
	int tx, ty;
	int sx, sy, ex, ey;
	int wh = width*height;
	int LineRange = width*DispRange;
	if (dy != 0)
	{
		sx = startx - dx; ex = endx + dx;
		sy = starty - dy; ey = endy + dy;

		for (x = sPx; x != ePx; x = x + px)
		{
			y = starty;

			pos = y*width + x;
			for (d = 0; d < DispRange; d++)//initialize the first pixel
			{
				preLr[d] = C[y*LineRange + x*DispRange + d];
			}

			tx = x + dx;
			ty = y + dy;
			while (tx != ex && ty != ey)
			{
				pos = ty*width + tx;
				deltaColor = 0;
				for (int n = 0; n < color; n++)
				{
					deltaColor += abs(img[n*width*height + ty*width + tx] - img[n*width*height + (ty - dy)*width + (tx - dx)]);
				}
				deltaColor /= color;
				k = (int)(deltaColor + 0.5);
				T = w_c[k];

				int cur_pos = ty*LineRange + tx*DispRange;
				for (d = 0; d < DispRange; d++)
				{
					Lr[d] = C[cur_pos + d] + T*preLr[d];
					S[cur_pos + d] += (Lr[d] - C[cur_pos + d]);
				}

				tx = tx + dx;
				ty = ty + dy;
				memcpy(preLr, Lr, sizeof(float)*DispRange);
			}
		}
	}

	if (dx != 0)
	{
		sx = startx - dx; ex = endx + dx;
		sy = starty - dy; ey = endy + dy;

		for (y = sPy; y != ePy; y = y + py)
		{
			x = startx;

			pos = y*width + x;
			for (d = 0; d < DispRange; d++)//initialize the first pixel
			{
				preLr[d] = C[y*LineRange + x*DispRange + d];
			}

			tx = x + dx;
			ty = y + dy;
			while (tx != ex && ty != ey)
			{
				pos = ty*width + tx;
				deltaColor = 0;
				for (int n = 0; n < color; n++)
				{
					deltaColor += abs(img[n*width*height + ty*width + tx] - img[n*width*height + (ty - dy)*width + (tx - dx)]);
				}
				deltaColor /= color;
				k = (int)(deltaColor + 0.5);
				T = w_c[k];

				int cur_pos = ty*LineRange + tx*DispRange;
				for (d = 0; d < DispRange; d++)
				{
					Lr[d] = C[cur_pos + d] + T*preLr[d];
					S[cur_pos + d] += (Lr[d] - C[cur_pos + d]);
				}

				tx = tx + dx;
				ty = ty + dy;
				memcpy(preLr, Lr, sizeof(float)*DispRange);
			}
		}
	}

	delete[]preLr; preLr = NULL;
	delete[]Lr; Lr = NULL;

	return 1;
}

int Tree_filtering_all_dispRange(BYTE *img, int color, float *cost, int width, int height, int dispRange,
	float sigC, int minD)
{
	//initialize
	float *S = new float[width*height*dispRange];
	memset(S, 0, sizeof(float)*width*height*dispRange);

	float *S_current = new float[width*height*dispRange];
	memcpy(S_current, cost, sizeof(float)*width*height*dispRange);

	float *Cost_backup = new float[width*height*dispRange];
	memcpy(Cost_backup, cost, sizeof(float)*width*height*dispRange);

	//1. firstly horizontal then vertical
	//horizontal
	memcpy(cost, Cost_backup, sizeof(float)*width*height*dispRange);
	memcpy(S_current, Cost_backup, sizeof(float)*width*height*dispRange);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 1, 0, cost, S_current);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, -1, 0, cost, S_current);

	//vertical
	memcpy(cost, S_current, sizeof(float)*width*height*dispRange);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 0, 1, cost, S_current);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 0, -1, cost, S_current);

	for (int i = 0; i < width*height*dispRange; i++)
	{
		S[i] += S_current[i];
	}

	//2. firstly vertical then horizontal
	memcpy(cost, Cost_backup, sizeof(float)*width*height*dispRange);
	memcpy(S_current, Cost_backup, sizeof(float)*width*height*dispRange);
	//vertical
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 0, 1, cost, S_current);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 0, -1, cost, S_current);

	//horizontal
	memcpy(cost, S_current, sizeof(float)*width*height*dispRange);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 1, 0, cost, S_current);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, -1, 0, cost, S_current);

	for (int i = 0; i < width*height*dispRange; i++)
	{
		S[i] += S_current[i];
	}

	//3. diagonal direction 1
	memcpy(cost, Cost_backup, sizeof(float)*width*height*dispRange);
	memcpy(S_current, Cost_backup, sizeof(float)*width*height*dispRange);
	//45 degree
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 1, -1, cost, S_current);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, -1, 1, cost, S_current);

	//135 degree
	memcpy(cost, S_current, sizeof(float)*width*height*dispRange);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 1, 1, cost, S_current);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, -1, -1, cost, S_current);

	for (int i = 0; i < width*height*dispRange; i++)
	{
		S[i] += S_current[i];
	}

	//4. diagonal direction 2
	memcpy(cost, Cost_backup, sizeof(float)*width*height*dispRange);
	memcpy(S_current, Cost_backup, sizeof(float)*width*height*dispRange);
	//135 degree
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 1, 1, cost, S_current);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, -1, -1, cost, S_current);

	//45 degree
	memcpy(cost, S_current, sizeof(float)*width*height*dispRange);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, 1, -1, cost, S_current);
	fst_bilaterla_filtering_single_direction_all_DispRang(img, width, height, color, minD, dispRange, sigC, -1, 1, cost, S_current);

	for (int i = 0; i < width*height*dispRange; i++)
	{
		S[i] += S_current[i];
	}

	memcpy(cost, S, sizeof(float)*width*height*dispRange);

	//free memory
	delete[]S; S = NULL;
	delete[]Cost_backup; Cost_backup = NULL;
	delete[]S_current; S_current = NULL;

	return 1;
}

int Tree_filtering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double sigma_C)
{
	//1. find the disparity range
	int minD, maxD;
	Get_DispRange(dsm_data, w, h, minD, maxD);
	int DispRange = maxD - minD + 1;

	//2. compute the cost volumes
	int max_deltaD = 5;
	int LineRange = w*DispRange;
	float *Cost = new float[w*h*DispRange];
	memset(Cost, 0, sizeof(float)*w*h*DispRange);
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			float curd = dsm_data[y*w + x];
			if (isnan(curd) == false && curd>m_minH && curd < m_maxH)
			{
				for (int d = 0; d < DispRange; d++)
				{
					float deltad = abs(d + minD - curd);
					Cost[y*LineRange + x*DispRange + d] = min(deltad, max_deltaD)/* / max_deltaD*/;
				}
			}
			else
			{
				for (int d = 0; d < DispRange; d++)
				{
					Cost[y*LineRange + x*DispRange + d] = 1;
				}
			}
		}

	//3. cost aggregations
	float *S = new float[w*h*DispRange];
	memcpy(S, Cost, sizeof(float)*w*h*DispRange);
	Tree_filtering_all_dispRange(ortho_img, bands, S, w, h, DispRange, sigma_C, minD);

	//4. WTA
	float S1, S2, S3;
	int Pos = 0;
	int Pos2 = 0;
	int dispHeight = h;
	int dispWidth = w;
	float *disp_data_refine = new float[w*h];
	for (int i = 0; i < w*h; i++)
		disp_data_refine[i] = NAN;

	for (int i = 0; i < dispHeight; i++)
		for (int j = 0; j < dispWidth; j++)
		{
			int sd, ed;
			int imgPos = i*w + j;
			sd = 0;
			ed = maxD - minD;

			Pos = i*LineRange + j*DispRange;
			float tmp = S[Pos + sd];
			int bestD = sd;
			for (int k = sd + 1; k <= ed; k++)
			{
				if (tmp > S[Pos + k])
				{
					tmp = S[Pos + k];
					bestD = k;
				}
			}

			float interD = (float)bestD;
			if (bestD > sd && bestD < ed)
			{
				S1 = S[Pos + (bestD - 1)];
				S2 = S[Pos + bestD];
				S3 = S[Pos + (bestD + 1)];

				if (S1 + S3 - 2.0f*S2 == 0)
					interD = (float)bestD;
				else
					interD = bestD + (S1 - S3) / (2.0f*(S1 + S3 - 2.0f*S2));
			}

			if (sd <= ed)
				disp_data_refine[imgPos] = interD + minD;
			else
				disp_data_refine[imgPos] = NAN;
			Pos2++;
		}

	//5. output
	memcpy(dsm_data, disp_data_refine, sizeof(float)*w*h);

	//free memory
	delete[]Cost; Cost = NULL;
	delete[]S; S = NULL;
	delete[]disp_data_refine; disp_data_refine = NULL;

	return 1;
}

int Compute_Local_Polynomial_Coeff(double *x, double *y, double *disp_win, double *intensity_win, double *weight_win, double wr, int validNum,
	double *a, double &gama, double &beta)
{
	//a[0]*x + a[1]*y + a[2]*x^2 + a[3]*x*y + a[4]*y^2
	//gama
	//beta
	CSimple_Matrix_Computation matc;
	double solution[7];
	double *A = new double[(2 * validNum) * 7];
	double *L = new double[2 * validNum];
	for (int i = 0; i < validNum; i++)
	{
		double weight = weight_win[i];
		weight = sqrt(weight);
		A[i * 7 + 0] = x[i] * weight;
		A[i * 7 + 1] = y[i] * weight;
		A[i * 7 + 2] = x[i] * x[i] * weight;
		A[i * 7 + 3] = x[i] * y[i] * weight;
		A[i * 7 + 4] = y[i] * y[i] * weight;
		A[i * 7 + 5] = 1* weight;
		A[i * 7 + 6] = intensity_win[i] * weight;

		L[i] = disp_win[i] * weight;
	}

	for (int i = validNum; i < 2 * validNum; i++)
	{
		double weight = weight_win[i]*wr;
		weight = sqrt(weight);
		A[i * 7 + 0] = 0;
		A[i * 7 + 1] = 0;
		A[i * 7 + 2] = 0;
		A[i * 7 + 3] = 0;
		A[i * 7 + 4] = 0;
		A[i * 7 + 5] = 0;
		A[i * 7 + 6] = 1 * weight;

		L[i] = 0;
	}

	//compute the solution
	double alpha = 0;
	matc.LeastSquares(A, L, solution, validNum * 2, 7, alpha);

	a[0] = solution[0];
	a[1] = solution[1];
	a[2] = solution[2];
	a[3] = solution[3];
	a[4] = solution[4];
	gama = solution[5];
	beta = solution[6];

	//free memory
	delete[]A; A = NULL;
	delete[]L; L = NULL;
	return 1;
}

int Compute_horizontal_vertical_weights(BYTE *img_data, int w, int h, int bands, int sx, int sy, int ex, int ey, float sigC, double &W)
{
	double Wh, Wv = 1;
	if (sx == ex && sy == ey)
	{
		W = 2;
	}

	//1. compute the Wh by firstly horizontally aggregating and then vertically aggregating.
	int dx, dy;
	if (sx == ex)
		dx = 0;
	else
		dx = (ex - sx) / abs(ex - sx);

	if (sy == ey)
		dy = 0;
	else
		dy = (ey - sy) / abs(ey - sy);

	Wh = 1;
	for (int x = sx; x != ex; x += dx) //horizontal aggregation
	{
		double deltaC = 0;
		double weight = 0;
		for (int b = 0; b < bands; b++)
		{
			deltaC += abs(img_data[b*w*h + sy*w + x + dx] - img_data[b*w*h + sy*w + x]);
		}
		deltaC /= bands;
		weight = exp(-deltaC / sigC);
		Wh = Wh*weight;
	}

	for (int y = sy; y != ey; y += dy) //vertical aggregation
	{
		double deltaC = 0;
		double weight = 0;
		for (int b = 0; b < bands; b++)
		{
			deltaC += abs(img_data[b*w*h + (y + dy)*w + ex] - img_data[b*w*h + y*w + ex]);
		}
		deltaC /= bands;
		weight = exp(-deltaC / sigC);
		Wh = Wh*weight;
	}

	//2. compute the Wv by firstly vertical aggregating and then horizontally aggregating.
	Wv = 1;
	for (int y = sy; y != ey; y += dy) //vertical aggregation
	{
		double deltaC = 0;
		double weight = 0;
		for (int b = 0; b < bands; b++)
		{
			deltaC += abs(img_data[b*w*h + (y + dy)*w + sx] - img_data[b*w*h + y*w + sx]);
		}
		deltaC /= bands;
		weight = exp(-deltaC / sigC);
		Wv = Wv*weight;
	}

	for (int x = sx; x != ex; x += dx) //horizontal aggregation
	{
		double deltaC = 0;
		double weight = 0;
		for (int b = 0; b < bands; b++)
		{
			deltaC += abs(img_data[b*w*h + ey*w + x + dx] - img_data[b*w*h + ey*w + x]);
		}
		deltaC /= bands;
		weight = exp(-deltaC / sigC);
		Wv = Wv*weight;
	}

	//3. combine the horizontal weight and the vertical weight
	W = Wh + Wv;

	return 1;
}

int MLPA_filtering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double &time, double er, double sigma_C)
{
	clock_t startTime, endTime;
	int halfsize = winsize / 2;
	double *weight_win = new double[winsize*winsize];
	double *disp_win = new double[winsize*winsize];
	double *intensity_win = new double[winsize*winsize];
	double *x_win = new double[winsize*winsize];
	double *y_win = new double[winsize*winsize];

	//step 1. locally correct each pixel
	startTime = clock();
	float *dsm_data_refine1 = new float[w*h];
	memcpy(dsm_data_refine1, dsm_data, sizeof(float)*w*h);
	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			int sx, ex, sy, ey;
			sx = x - halfsize; ex = x + halfsize;
			sy = y - halfsize; ey = y + halfsize;

			if (sx < 0) sx = 0;
			if (sy < 0) sy = 0;
			if (ex > w - 1) ex = w - 1;
			if (ey > h - 1) ey = h - 1;

			int I = 0;
			int validNum = 0;
			for (int ty = sy; ty <= ey; ty++)
				for (int tx = sx; tx <= ex; tx++)
				{
					float curd = dsm_data[ty*w + tx];
					if (isnan(curd) == true || curd < m_minH || curd>m_maxH) continue;

					double weight = 0;
					Compute_horizontal_vertical_weights(ortho_img, w, h, bands, tx, ty, x, y, sigma_C, weight);

					double cur_intensity = 0;
					for (int k = 0; k < bands; k++)
					{
						cur_intensity += ortho_img[k*w*h + ty*w + tx];
					}
					cur_intensity /= bands;

					weight_win[I] = max(weight, 0.01);
					disp_win[I] = curd;
					intensity_win[I] = cur_intensity;
					x_win[I] = tx;
					y_win[I] = ty;
					I++;
					validNum++;
				}

			//a[0]*x + a[1]*y + a[2]*x^2 + a[3]*x*y + a[4]*y^2
			if (validNum > 12)
			{
				double a[5] = { 0 }, gama = 0, beta = 0;
				double cur_intensity = 0;
				for (int k = 0; k < bands; k++)
				{
					cur_intensity += ortho_img[k*w*h + y*w + x];
				}
				cur_intensity /= bands;
				Compute_Local_Polynomial_Coeff(x_win, y_win, disp_win, intensity_win, weight_win, er, validNum, a, gama, beta);
				dsm_data_refine1[y*w + x] = a[0] * x + a[1] * y + a[2] * x*x + a[3] * x*y + a[4] * y*y + gama + beta*cur_intensity;
			}
		}
	}
	endTime = clock();

	time = (double)(endTime - startTime) / CLOCKS_PER_SEC;

	//2. refine the correction results by aggregating the surrounding results.
	float *dsm_data_refine2 = new float[w*h];
	memcpy(dsm_data_refine2, dsm_data_refine1, sizeof(float)*w*h);
	for (int y = 0; y < h; y++)
	{
		for (int x = 0; x < w; x++)
		{
			int sx, ex, sy, ey;
			sx = x - halfsize; ex = x + halfsize;
			sy = y - halfsize; ey = y + halfsize;

			if (sx < 0) sx = 0;
			if (sy < 0) sy = 0;
			if (ex > w - 1) ex = w - 1;
			if (ey > h - 1) ey = h - 1;

			double sumW = 0;
			double sumD = 0;
			for (int ty = sy; ty <= ey; ty++)
				for (int tx = sx; tx <= ex; tx++)
				{
					float curd = dsm_data_refine1[ty*w + tx];
					if (isnan(curd) == true || curd < m_minH || curd>m_maxH) continue;

					double weight = 0;
					Compute_horizontal_vertical_weights(ortho_img, w, h, bands, tx, ty, x, y, sigma_C, weight);
					weight = max(weight, 0.01);
					sumW += weight;
					sumD += weight*curd;
				}

			if (sumW != 0)
				sumD /= sumW;
			else
				sumD = dsm_data_refine1[y*w + x];

			dsm_data_refine2[y*w + x] = sumD;
		}
	}

	memcpy(dsm_data, dsm_data_refine2, sizeof(float)*w*h);

	//free memory
	delete[]weight_win; weight_win = NULL;
	delete[]dsm_data_refine1; dsm_data_refine1 = NULL;
	delete[]dsm_data_refine2; dsm_data_refine2 = NULL;
	delete[]disp_win; disp_win = NULL;
	delete[]intensity_win; intensity_win = NULL;
	delete[]x_win; x_win = NULL;
	delete[]y_win; y_win = NULL;
	return 1;
}

int GuidedMedian_Filtering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double sig_S, double sig_C)
{
	//printf("Bialtering filtering...\n");
	float *dsm_filter = new float[w*h];
	memcpy(dsm_filter, dsm_data, sizeof(float)*w*h);

	int halfsize = winsize / 2;

	int scale = 2;
	float *localDis = new float[winsize*winsize];
	float *weight_array = new float[winsize*winsize];
	float *array = new float[winsize*winsize*(scale + 1)];
	int localNum = 0;

	double deltaS, deltaC;
	double ws, wc;
	double weight;
	double sum_w;
	float midValue = 0;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			if (isnan(dsm_data[y*w + x]) == true)
			{
				continue;
			}
			int sx, ex, sy, ey;
			sx = x - halfsize;
			ex = x + halfsize;
			sy = y - halfsize;
			ey = y + halfsize;

			if (sx < 0) sx = 0;
			if (ex > w - 1) ex = w - 1;
			if (sy < 0) sy = 0;
			if (ey > h - 1) ey = h - 1;

			int curpos = y*w + x;
			int tx, ty;
			sum_w = 1;
			localNum = 0;
			for (ty = sy; ty <= ey; ty++)
				for (tx = sx; tx <= ex; tx++)
				{
					if (isnan(dsm_data[ty*w + tx]) == false)
					{
						//if (ty != y || tx != x)
						{
							deltaS = max(abs(ty - y), abs(tx - x)); //¿Õ¼ä¾àÀë
							deltaC = 0;
							if (bands == 1)
							{
								deltaC = abs(ortho_img[ty*w + tx] - ortho_img[y*w + x]);
							}
							else
							{
								double tmpC = (abs(ortho_img[0 * w*h + ty*w + tx] - ortho_img[0 * w*h + y*w + x])
									+ abs(ortho_img[1 * w*h + ty*w + tx] - ortho_img[1 * w*h + y*w + x])
									+ abs(ortho_img[2 * w*h + ty*w + tx] - ortho_img[2 * w*h + y*w + x])) / 3.0;

								if (tmpC > 254.5) tmpC = 254.5;
								deltaC = (int)(tmpC + 0.5);
							}

							ws = exp(-deltaS / sig_S);
							wc = exp(-deltaC / sig_C);

							weight = wc;
							sum_w += weight;

							weight_array[localNum] = weight;
							localDis[localNum] = dsm_data[ty*w + tx];
							//dsm_filter[curpos] += weight*dsm_data[ty*w + tx];

							localNum++;
						}
					}
				}

			WeightedSort_Med(localDis, weight_array, localNum, midValue, scale, array);

			dsm_filter[curpos] = midValue;
		}

	memcpy(dsm_data, dsm_filter, sizeof(float)*w*h);

	//free memory
	delete[]dsm_filter; dsm_filter = NULL;
	delete[]localDis; localDis = NULL;
	delete[]weight_array; weight_array = NULL;
	delete[]array; array = NULL;
	return 1;
}

void WeightedSort_Med(float *cost, float *w, int Num, float &med, int scale, float *array)
{
	int i;
	int Num2 = 0;
	for (i = 0; i < Num; i++)
	{
		Num2 += ((int)(scale*w[i] + 0.5));
	}

	int j, k = 0;
	for (i = 0; i < Num; i++)
		for (j = 0; j < (int)(scale*w[i] + 0.5); j++)
		{
			array[k] = cost[i];
			k++;
		}

	//ÅÅÐò
	BubbleSort(array, Num2);
	//quicksort(array,Num2);

	if ((Num2 % 2) == 1)
	{
		med = array[Num2 / 2];
	}
	else
	{
		med = (array[Num2 / 2] + array[Num2 / 2 - 1]) / 2.0f;
	}
}

void BubbleSort(float *arr, int size)
{
	int i, j;
	for (i = 0; i </*size-1*/size / 2 + 1; i++)
		for (j = size - 1; j > i; j--)
			if (arr[j] < arr[j - 1])
				Swap(arr[j], arr[j - 1]);
}

void Swap(float &a, float &b)
{
	float temp = a;
	a = b;
	b = temp;
}


int BialteringFiltering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double sig_S, double sig_C)
{
	//printf("Bialtering filtering...\n");
	float *dsm_filter = new float[w*h];
	memcpy(dsm_filter, dsm_data, sizeof(float)*w*h);

	int halfsize = winsize / 2;

	double deltaS, deltaC;
	double ws, wc;
	double weight;
	double sum_w;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			if (isnan(dsm_data[y*w + x]) == true)
			{
				continue;
			}
			int sx, ex, sy, ey;
			sx = x - halfsize;
			ex = x + halfsize;
			sy = y - halfsize;
			ey = y + halfsize;

			if (sx < 0) sx = 0;
			if (ex > w - 1) ex = w - 1;
			if (sy < 0) sy = 0;
			if (ey > h - 1) ey = h - 1;

			int curpos = y*w + x;
			int tx, ty;
			sum_w = 1;
			for (ty = sy; ty <= ey; ty++)
				for (tx = sx; tx <= ex; tx++)
				{
					if (isnan(dsm_data[ty*w + tx]) == false)
					{
						if (ty != y || tx != x)
						{
							deltaS = max(abs(ty - y), abs(tx - x)); //¿Õ¼ä¾àÀë
							deltaC = 0;
							if (bands == 1)
							{
								deltaC = abs(ortho_img[ty*w + tx] - ortho_img[y*w + x]);
							}
							else
							{
								double tmpC = (abs(ortho_img[0 * w*h + ty*w + tx] - ortho_img[0 * w*h + y*w + x])
											 + abs(ortho_img[1 * w*h + ty*w + tx] - ortho_img[1 * w*h + y*w + x])
											 + abs(ortho_img[2 * w*h + ty*w + tx] - ortho_img[2 * w*h + y*w + x])) / 3.0;

								if (tmpC > 254.5) tmpC = 254.5;
								deltaC = (int)(tmpC + 0.5);
							}

							ws = exp(-deltaS / sig_S);
							wc = exp(-deltaC / sig_C);

							weight = ws*wc;
							sum_w += weight;

							dsm_filter[curpos] += weight*dsm_data[ty*w + tx];
						}
					}
				}

			dsm_filter[curpos] /= sum_w;
		}

	memcpy(dsm_data, dsm_filter, sizeof(float)*w*h);

	//free memory
	delete[]dsm_filter; dsm_filter = NULL;

	return 1;
}

int LocalPatchFiltering_AdaptWin_slow(int blockID, int blockNum, float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int PatchNum,
	double P_coeff, double sig_C, double sig_S, double sig_D, int iteration_times, float w_color, float w_dis, float w_disp,
	double &running_time)
{
	//printf("Local patch filtering with slow version...\n");
	double disp_thresh = 2.0; //force the differences betweent the filtering results and the original DSM no higher than disp_thresh

	//step1. adapt win selection
	PixelInfo **adaptWin = new PixelInfo *[w*h];
	for (int i = 0; i < w*h; i++)
	{
		adaptWin[i] = new PixelInfo[PatchNum];
	}
	int *validNum = new int[w*h];
	memset(validNum, 0, sizeof(int)*w*h);
	PixelInfo *local_win = new PixelInfo[PatchNum];
	double *plane_paras = new double[w*h * 3]; //store in sequence of a, b, c
	PixelInfo *neighbors = new PixelInfo[PatchNum * 10];

	clock_t startTime, end_time_workflow;
	startTime = clock();
	printf("Local Patch filtering process...%d/%d block | define adaptive window for each pixel\r", blockID, blockNum);
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			DefineAdaptWin(x, y, PatchNum, dsm_data, ortho_img, w, h, bands, w_dis, w_color, w_disp,
				adaptWin[y*w + x], validNum[y*w + x], neighbors);
		}

	//iterative filtering
	int I = 0;
	int pos = 0;
	while (I < iteration_times)
	{
		printf("Local Patch filtering process...%d/%d block | The %d / %d th iteration                 \r", blockID, blockNum, I + 1, iteration_times);
		if (I > 0)
		{
			//update d;
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++)
				{
					pos = y*w + x;
					for (int i = 0; i < validNum[pos]; i++)
					{
						int tx, ty;
						tx = adaptWin[pos][i].x;
						ty = adaptWin[pos][i].y;

						adaptWin[pos][i].d = dsm_data[ty*w + tx];
					}
				}

			//update deltaD
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++)
				{
					pos = y*w + x;
					for (int i = 0; i < validNum[pos]; i++)
					{
						int tx, ty;
						tx = adaptWin[pos][i].x;
						ty = adaptWin[pos][i].y;

						adaptWin[pos][i].deltaD = fabs(dsm_data[ty*w + tx] - dsm_data[pos]);
					}
				}
		}

		//initialize
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
				for (int k = 0; k < 3; k++)
				{
					plane_paras[k*w*h + y*w + x] = NAN;
				}

		//compute a,b,c for each pixel
		//printf("Compute disparity plane parameters for each pixel\n");
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
			{
				pos = y*w + x;
				int localNum = validNum[pos];
				if (localNum<10) continue;

				//update surface orientation in disparity plane computation
				for (int i = 0; i < localNum; i++)
				{
					double tP = 0;
					double deltaC, deltaS, deltaD = 0;
					int tx, ty;
					tx = adaptWin[pos][i].x;
					ty = adaptWin[pos][i].y;
					float td = adaptWin[pos][i].d;

					deltaS = adaptWin[pos][i].deltaS;
					deltaC = adaptWin[pos][i].deltaC;
					deltaD = adaptWin[pos][i].deltaD;

					//double ws = exp(-deltaS / sig_S);
					//double wc = exp(-deltaC / sig_C);
					double wd = exp(-deltaD / sig_D);
					tP = /*wc**/wd; //to compute disparity plane, it seems that the spatial dis does not need.

					adaptWin[pos][i].P = tP;
				}

				if (localNum > 10)
				{
					//compute a b c for current pixel
					double ta = 0, tb = 0, tc = 0;

					compute_abc(adaptWin[pos], localNum, ta, tb, tc);

					double td = ta*x + tb*y + tc;
					double deltad = fabs(td - dsm_data[pos]);
					if (deltad <= disp_thresh)
					{
						plane_paras[0 * w*h + y*w + x] = ta;
						plane_paras[1 * w*h + y*w + x] = tb;
						plane_paras[2 * w*h + y*w + x] = tc;
					}
				}
			}

		//printf("Refine disparity for each pixel\n");
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
			{
				pos = y*w + x;
				int localNum = validNum[pos];
				double check_a = plane_paras[0 * w*h + pos];

				if (isnan(check_a) == true) continue;

				//update P (surface orientation) in the refinement
				for (int i = 0; i < localNum; i++)
				{
					double tP = 0;
					double deltaC, deltaS, deltaD = 0;
					int tx, ty;
					tx = adaptWin[pos][i].x;
					ty = adaptWin[pos][i].y;
					float td = adaptWin[pos][i].d;

					deltaS = adaptWin[pos][i].deltaS;
					deltaC = adaptWin[pos][i].deltaC;
					deltaD = adaptWin[pos][i].deltaD;

					double ws = exp(-deltaS / sig_S);
					//double wc = exp(-deltaC / sig_C);
					double wd = exp(-deltaD / sig_D);
					tP = ws*wd/**wc*/;

					if (tx == x && ty == y)
						adaptWin[pos][i].P = 1;
					else
						adaptWin[pos][i].P = P_coeff*tP;
				}

				int validPixelNum = localNum;
				for (int i = 0; i < localNum; i++)
				{
					local_win[i] = adaptWin[pos][i];
					int tx, ty;
					tx = adaptWin[pos][i].x;
					ty = adaptWin[pos][i].y;

					if (tx == x && ty == y)
					{
						continue;
					}

					//update d by plane parameters
					double na, nb, nc;
					na = plane_paras[0 * w*h + ty*w + tx];
					nb = plane_paras[1 * w*h + ty*w + tx];
					nc = plane_paras[2 * w*h + ty*w + tx];

					if (isnan(na) == false)
						local_win[i].d = na*tx + nb*ty + nc;
					else
					{
						local_win[i].P = 0;
						validPixelNum--;
					}
				}
				if (validPixelNum > 10)
				{
					//compute a b c for current pixel
					double ta = 0, tb = 0, tc = 0;

					compute_abc(local_win, localNum, ta, tb, tc);

					//refine disparity
					float refine_d = ta*x + tb*y + tc;

					float deltad = fabs(refine_d - dsm_data[pos]);
					if (deltad <= disp_thresh)
						dsm_data[pos] = refine_d;
				}
			}

		//end_time_iteration = clock();
		//printf("The running time of %d iteration is %lf second\n", I + 1, (double)(end_time_iteration - endTime_adapt) / CLOCKS_PER_SEC);
		//endTime_adapt = end_time_iteration;

		I++;

	}
	
	end_time_workflow = clock();
	//printf("\nThe running time of the filtering workflow is %lf second\n", (double)(end_time_workflow - startTime) / CLOCKS_PER_SEC);

	running_time = (double)(end_time_workflow - startTime) / CLOCKS_PER_SEC;

	//free memory
	for (int i = 0; i < w*h; i++)
	{
		delete adaptWin[i];
	}
	delete[]adaptWin; adaptWin = NULL;
	delete[]validNum; validNum = NULL;
	delete[]local_win; local_win = NULL;
	delete[]neighbors; neighbors = NULL;
	delete[]plane_paras; plane_paras = NULL;
	return 1;
}

int LocalPatchFiltering_AdaptWin(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int PatchNum, 
	double P_coeff, double sig_C, double sig_S, double sig_D, int iteration_times)
{
	printf("Local patch filtering...\n");
	double w_dis = 0.3, w_color = 0.3, w_disp = 0.6; // later, these parameters will be set as the input of functions. 
	double disp_thresh = 2;
	PixelInfo **adaptWin = new PixelInfo *[w*h];
	for (int i = 0; i < w*h;i++)
	{
		adaptWin[i] = new PixelInfo[PatchNum];
	}
	int *validNum = new int[w*h];
	PixelInfo *local_win = new PixelInfo[PatchNum];
	double *plane_paras = new double[w*h * 3]; //store in sequence of a, b, c
	memset(validNum, 0, sizeof(int)*w*h);

	clock_t startTime, endTime_adapt, end_time_iteration, end_time_workflow;
	startTime = clock();
	printf("define adaptive window for each pixel\n");
	DefineAdaptWin_fst(PatchNum, dsm_data, ortho_img, w, h, bands, w_dis, w_color, w_disp, adaptWin, validNum);
// 	for (int y = 0; y < h; y++)
// 	{
// 		for (int x = 0; x < w; x++)
// 		{
// 			DefineAdaptWin(x, y, PatchNum, dsm_data, ortho_img, w, h, bands, w_dis, w_color, w_disp,
// 				adaptWin[y*w + x], validNum[y*w + x]);
// 		}
// 		printf("current progress... %lf  %%", (double)(y + 1) / h * 100);
// 		printf("\r");
// 	}
// 	printf("\n");
	endTime_adapt = clock();
	printf("The running time of adaptive window define is %lf second\n", (double)(endTime_adapt - startTime) / CLOCKS_PER_SEC);

	int I = 0;
	while (I < iteration_times)
	{
		printf("The %d / %d th iteration\n", I + 1, iteration_times);

		if (I > 0)
		{
			//update d;
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++)
				{
					for (int i = 0; i < validNum[y*w + x];i++)
					{
						int tx, ty;
						tx = adaptWin[y*w + x][i].x;
						ty = adaptWin[y*w + x][i].y;

						adaptWin[y*w + x][i].d = dsm_data[ty*w + tx];
					}
				}

			//update deltaD
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++)
				{
					for (int i = 0; i < validNum[y*w + x]; i++)
					{
						int tx, ty;
						tx = adaptWin[y*w + x][i].x;
						ty = adaptWin[y*w + x][i].y;

						adaptWin[y*w + x][i].deltaD = fabs(dsm_data[ty*w + tx] - dsm_data[y*w + x]);
					}
				}
		}

		//initialize
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
				for (int k = 0; k < 3; k++)
				{
					plane_paras[k*w*h + y*w + x] = NAN;
				}

		//compute abc for each pixel
		printf("Compute disparity plane parameters for each pixel\n");
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
			{
				int localNum = validNum[y*w + x];
			
				//update P in disparity plane computation
				for (int i = 0; i < localNum;i++)
				{
					double tP = 0;
					double deltaC, deltaS, deltaD = 0;
					int tx, ty;
					tx = adaptWin[y*w + x][i].x;
					ty = adaptWin[y*w + x][i].y;
					float td = adaptWin[y*w + x][i].d;

					deltaS = adaptWin[y*w + x][i].deltaS;
					deltaC = adaptWin[y*w + x][i].deltaC;
					deltaD = adaptWin[y*w + x][i].deltaD;
//					deltaS = sqrt((tx - x)*(tx - x) + (ty - y)*(ty - y));
// 					if (bands == 1)
// 					{
// 						deltaC = abs(ortho_img[ty*w + tx] - ortho_img[y*w + x]);
// 					}
// 					else
// 					{
// 						double tmpC = (abs(ortho_img[0 * w*h + ty*w + tx] - ortho_img[0 * w*h + y*w + x])
// 							+ abs(ortho_img[1 * w*h + ty*w + tx] - ortho_img[1 * w*h + y*w + x])
// 							+ abs(ortho_img[2 * w*h + ty*w + tx] - ortho_img[2 * w*h + y*w + x])) / 3.0;
// 
// 						if (tmpC > 254.5) tmpC = 254.5;
// 						deltaC = (int)(tmpC + 0.5);
// 					}
// 					deltaD = fabs(td - dsm_data[y*w + x]);
					double ws = exp(-deltaS / sig_S);
					double wc = exp(-deltaC / sig_C);
					double wd = exp(-deltaD / sig_D);
					tP = /*wc**/wd; //to compute disparity plane, it seems that the spatial dis does not need.
					
					adaptWin[y*w + x][i].P = tP;
				}
				
				if (localNum > 10)
				{
					//compute a b c for current pixel
					double ta = 0, tb = 0, tc = 0;

					compute_abc(adaptWin[y*w+x], localNum, ta, tb, tc);

					double td = ta*x + tb*y + tc;
					double deltad = fabs(td - dsm_data[y*w + x]);
					if (deltad <= disp_thresh)
					{
						plane_paras[0 * w*h + y*w + x] = ta;
						plane_paras[1 * w*h + y*w + x] = tb;
						plane_paras[2 * w*h + y*w + x] = tc;
					}
				}
			}

		printf("Refine disparity for each pixel\n");
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
			{
				int localNum = validNum[y*w + x];
				double check_a = plane_paras[0 * w*h + y*w + x];

				if (isnan(check_a)==true) continue;
				//////////////////////////////////////////////////////////////////////////
// 				int maxd = adaptWin[y*w + x][0].deltaD;
// 				for (int i = 1; i < localNum; i++)
// 				{
// 					if (maxd < adaptWin[y*w + x][i].deltaD)
// 						maxd = adaptWin[y*w + x][i].deltaD;
// 				}
// 				if (maxd > 4) continue;
				//////////////////////////////////////////////////////////////////////////
// 				double a, b;
// 				a = plane_paras[0 * w*h + y*w + x];
// 				b = plane_paras[1 * w*h + y*w + x];
// 				double theta = fabs(atan2(b, a));
// 				if (theta<PI/6) continue;

				//update P in the refinement
				for (int i = 0; i < localNum; i++)
				{
					double tP = 0;
					double deltaC, deltaS, deltaD = 0;
					int tx, ty;
					tx = adaptWin[y*w + x][i].x;
					ty = adaptWin[y*w + x][i].y;
					float td = adaptWin[y*w + x][i].d;

					deltaS = adaptWin[y*w + x][i].deltaS;
					deltaC = adaptWin[y*w + x][i].deltaC;
					deltaD = adaptWin[y*w + x][i].deltaD;
// 					deltaS = sqrt((tx - x)*(tx - x) + (ty - y)*(ty - y));
// 					if (bands == 1)
// 					{
// 						deltaC = abs(ortho_img[ty*w + tx] - ortho_img[y*w + x]);
// 					}
// 					else
// 					{
// 						double tmpC = (abs(ortho_img[0 * w*h + ty*w + tx] - ortho_img[0 * w*h + y*w + x])
// 							+ abs(ortho_img[1 * w*h + ty*w + tx] - ortho_img[1 * w*h + y*w + x])
// 							+ abs(ortho_img[2 * w*h + ty*w + tx] - ortho_img[2 * w*h + y*w + x])) / 3.0;
// 
// 						if (tmpC > 254.5) tmpC = 254.5;
// 						deltaC = (int)(tmpC + 0.5);
// 					}
// 					deltaD = fabs(td - dsm_data[y*w + x]);
					double ws = exp(-deltaS / sig_S);
					double wc = exp(-deltaC / sig_C);
					double wd = exp(-deltaD / sig_D);
					tP = ws*wd; 

					if (tx == x && ty == y)
						adaptWin[y*w + x][i].P = 1;
					else
						adaptWin[y*w + x][i].P = P_coeff*tP;
				}

				int validPixelNum = localNum;
				for (int i = 0; i < localNum; i++)
				{
					local_win[i] = adaptWin[y*w + x][i];
					int tx, ty;
					tx = adaptWin[y*w + x][i].x;
					ty = adaptWin[y*w + x][i].y;

					if (tx == x && ty == y)
					{
						continue;
					}

					//update d by plane parameters
					double na, nb, nc;
					na = plane_paras[0 * w*h + ty*w + tx];
					nb = plane_paras[1 * w*h + ty*w + tx];
					nc = plane_paras[2 * w*h + ty*w + tx];

					if (isnan(na) == false)
						local_win[i].d = na*tx + nb*ty + nc;
					else
					{
						local_win[i].P = 0;
						validPixelNum--;
					}
				}
				if (validPixelNum > 10)
				{
					//compute a b c for current pixel
					double ta = 0, tb = 0, tc = 0;

					compute_abc(local_win, localNum, ta, tb, tc);

					//refine disparity
					float refine_d = ta*x + tb*y + tc;

					float deltad = fabs(refine_d - dsm_data[y*w + x]);
					if (deltad<=disp_thresh)
						dsm_data[y*w + x] = refine_d;
				}
			}

		end_time_iteration = clock();
		printf("The running time of %d iteration is %lf second\n", I + 1, (double)(end_time_iteration - endTime_adapt) / CLOCKS_PER_SEC);
		endTime_adapt = end_time_iteration;

		I++;
	}

	end_time_workflow = clock();
	printf("\nThe running time of the filtering workflow is %lf second\n", (double)(end_time_workflow - startTime) / CLOCKS_PER_SEC);

	//free memory
	for (int i = 0; i < w*h; i++)
	{
		delete adaptWin[i];
	}
	delete[]adaptWin; adaptWin = NULL;
	delete[]validNum; validNum = NULL;
	delete[]local_win; local_win = NULL;
	delete[]plane_paras; plane_paras = NULL;

	return 1;
}

//printf("ValidNum = %d\t", validNum[y*w + x]);
// if (x == 722 && y == 597)
// {
// 	for (int i = 0; i < validNum[y*w + x]; i++)
// 	{
// 		int tx, ty;
// 		tx = adaptWin[y*w + x][i].x;
// 		ty = adaptWin[y*w + x][i].y;
// 
// 		ortho_img[0 * w*h + ty*w + tx] = 255;
// 		ortho_img[1 * w*h + ty*w + tx] = 255;
// 		ortho_img[2 * w*h + ty*w + tx] = 255;
// 	}
// 	printf("valid num = %d", validNum[y*w + x]);
// 	SaveImageFile("D:\\out.tif", ortho_img, w, h, 3);
// 	return 0;
// }

int LocalPatchFiltering(float *dsm_data, BYTE *ortho_img, int w, int h, int bands, int winsize, double P_coeff, 
	double sig_C, double sig_S, double sig_D, int iteration_times)
{
	int I = 0;
	int halfsize = winsize / 2;
	PixelInfo * local_pt_array = new PixelInfo[winsize*winsize];
	int localNum = 0;
	float *dsm_base = new float[w*h];
	memcpy(dsm_base, dsm_data, sizeof(float)*w*h);
	while (I<iteration_times)
	{
		printf("%d / %d th iteration...\n", I + 1, iteration_times);
		double *coeff_matrix = new double[w*h * 3]; //store in sequence of a, b, c
		float *dsm_refine = new float[w*h];
		memcpy(dsm_refine, dsm_data, sizeof(float)*w*h);

		//initialize
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
				for (int k = 0; k < 3;k++)
				{
					coeff_matrix[k*w*h + y*w + x] = NAN;
				}

		//compute abc for each pixel
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
			{
				int sx, sy, ex, ey;
				sx = x - halfsize; 
				ex = x + halfsize; 
				sy = y - halfsize; 
				ey = y + halfsize;

				if (sx < 0) sx = 0;
				if (ex > w - 1) ex = w - 1;
				if (sy < 0) sy = 0;
				if (ey > h - 1) ey = h - 1;

				localNum = 0;
				for (int ty = sy; ty <= ey; ty++)
					for (int tx = sx; tx <= ex; tx++)
					{
						float td = dsm_data[ty*w + tx];
						if (isnan(td)==false)
						{
							//compute weight which is related to spatial distance, color difference;
							double tP = 0;
							double deltaC, deltaS, deltaD = 0;
							deltaS = sqrt((tx - x)*(tx - x) + (ty - y)*(ty - y));
							if (bands == 1)
							{
								deltaC = abs(ortho_img[ty*w + tx] - ortho_img[y*w + x]);
							}
							else
							{
								double tmpC = (abs(ortho_img[0 * w*h + ty*w + tx] - ortho_img[0 * w*h + y*w + x])
									+ abs(ortho_img[1 * w*h + ty*w + tx] - ortho_img[1 * w*h + y*w + x])
									+ abs(ortho_img[2 * w*h + ty*w + tx] - ortho_img[2 * w*h + y*w + x])) / 3.0;

								if (tmpC > 254.5) tmpC = 254.5;
								deltaC = (int)(tmpC + 0.5);
							}
							deltaD = fabs(td - dsm_data[y*w + x]);

							double ws = exp(-deltaS / sig_S);
							double wc = exp(-deltaC / sig_C);
							double wd = exp(-deltaD / sig_D);
							tP = wc*wd; //to compute disparity plane, it seems that the spatial dis does not need.

							local_pt_array[localNum].x = (double)tx;
							local_pt_array[localNum].y = (double)ty;
							local_pt_array[localNum].d = (double)td;
							local_pt_array[localNum].P = tP;
							localNum++;
						}
					}

				if (localNum > 3)
				{
					//compute a b c for current pixel
					double ta = 0, tb = 0, tc = 0;

					compute_abc(local_pt_array, localNum, ta, tb, tc);
					
					coeff_matrix[0 * w*h + y*w + x] = ta;
					coeff_matrix[1 * w*h + y*w + x] = tb;
					coeff_matrix[2 * w*h + y*w + x] = tc;
				}
			}

		//refine the whole dsm
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
			{
				int sx, sy, ex, ey;
				sx = x - halfsize;
				ex = x + halfsize;
				sy = y - halfsize;
				ey = y + halfsize;

				if (sx < 0) sx = 0;
				if (ex > w - 1) ex = w - 1;
				if (sy < 0) sy = 0;
				if (ey > h - 1) ey = h - 1;

				localNum = 0;
				for (int ty = sy; ty <= ey; ty++)
					for (int tx = sx; tx <= ex; tx++)
					{
						float td = dsm_data[ty*w + tx];
						if (isnan(td) == false)
						{
							//compute weight which is related to spatial distance, color difference;
							double tP = 0;
							double deltaC, deltaS, deltaD = 0;
							deltaS = sqrt((tx - x)*(tx - x) + (ty - y)*(ty - y));
							if (bands == 1)
							{
								deltaC = abs(ortho_img[ty*w + tx] - ortho_img[y*w + x]);
							}
							else
							{
								double tmpC = (abs(ortho_img[0 * w*h + ty*w + tx] - ortho_img[0 * w*h + y*w + x])
									+ abs(ortho_img[1 * w*h + ty*w + tx] - ortho_img[1 * w*h + y*w + x])
									+ abs(ortho_img[2 * w*h + ty*w + tx] - ortho_img[2 * w*h + y*w + x])) / 3.0;

								if (tmpC > 254.5) tmpC = 254.5;
								deltaC = (int)(tmpC + 0.5);
							}
							deltaD = fabs(dsm_data[y*w + x] - dsm_data[ty*w + tx]);
							double ws = exp(-deltaS / sig_S);
							double wc = exp(-deltaC / sig_C);
							double wd = exp(-deltaD / sig_D);
							tP = wc*ws*wd;

							//change td
							double ca, cb, cc;
							ca = coeff_matrix[0 * w*h + ty*w + tx];
							cb = coeff_matrix[1 * w*h + ty*w + tx];
							cc = coeff_matrix[2 * w*h + ty*w + tx];

							if (tx != x || ty != y)
								td = ca*tx + cb*ty + cc;

							local_pt_array[localNum].x = (double)tx;
							local_pt_array[localNum].y = (double)ty;
							local_pt_array[localNum].d = (double)td;
							local_pt_array[localNum].P = tP;
							localNum++;
						}
					}

				if (localNum > 3)
				{
					//compute a b c for current pixel
					double ta = 0, tb = 0, tc = 0;

					compute_abc(local_pt_array, localNum, ta, tb, tc);

					dsm_refine[y*w + x] = ta*x + tb*y + tc;
				}
			}

		memcpy(dsm_data, dsm_refine, sizeof(float)*w*h);
		I++;

		//free memory
		delete[]coeff_matrix; coeff_matrix = NULL;
		delete[]dsm_refine; dsm_refine = NULL;
	}

	

	//free memory
	delete[]local_pt_array; local_pt_array = NULL;
	delete[]dsm_base; dsm_base = NULL;
	return 1;
}

int compute_abc(PixelInfo *array, int ptnum, double &a, double &b, double &c)
{
	if (ptnum<3)
	{
		a = 0; b = 0; c = 0;
		return 0;
	}

	double design_mat[9];
	double const_vec[3];
	memset(design_mat, 0, sizeof(double) * 9);
	memset(const_vec, 0, sizeof(double) * 3);

	for (int i = 0; i < ptnum; i++)
	{
		double tx, ty, td, tP;
		tx = array[i].x;
		ty = array[i].y;
		td = array[i].d;
		tP = array[i].P;

		design_mat[0] += tP*tx*tx;
		design_mat[1] += tP*tx*ty;
		design_mat[2] += tP*tx;

		design_mat[3] += tP*tx*ty;
		design_mat[4] += tP*ty*ty;
		design_mat[5] += tP*ty;

		design_mat[6] += tP*tx;
		design_mat[7] += tP*ty;
		design_mat[8] += tP;

		const_vec[0] += tP*tx*td;
		const_vec[1] += tP*ty*td;
		const_vec[2] += tP*td;
	}

	a = (const_vec[0] * design_mat[4] * design_mat[8] - const_vec[0] * design_mat[5] * design_mat[7] - const_vec[1] * design_mat[1] * design_mat[8] 
		+ const_vec[1] * design_mat[2] * design_mat[7] + const_vec[2] * design_mat[1] * design_mat[5] - const_vec[2] * design_mat[2] * design_mat[4])
		/ (design_mat[0] * design_mat[4] * design_mat[8] - design_mat[0] * design_mat[5] * design_mat[7] - design_mat[1] * design_mat[3] * design_mat[8] 
		+ design_mat[1] * design_mat[5] * design_mat[6] + design_mat[2] * design_mat[3] * design_mat[7] - design_mat[2] * design_mat[4] * design_mat[6]);
	b = -(const_vec[0] * design_mat[3] * design_mat[8] - const_vec[0] * design_mat[5] * design_mat[6] - const_vec[1] * design_mat[0] * design_mat[8] 
		+ const_vec[1] * design_mat[2] * design_mat[6] + const_vec[2] * design_mat[0] * design_mat[5] - const_vec[2] * design_mat[2] * design_mat[3]) 
		/ (design_mat[0] * design_mat[4] * design_mat[8] - design_mat[0] * design_mat[5] * design_mat[7] - design_mat[1] * design_mat[3] * design_mat[8] 
		+ design_mat[1] * design_mat[5] * design_mat[6] + design_mat[2] * design_mat[3] * design_mat[7] - design_mat[2] * design_mat[4] * design_mat[6]);
	c = (const_vec[0] * design_mat[3] * design_mat[7] - const_vec[0] * design_mat[4] * design_mat[6] - const_vec[1] * design_mat[0] * design_mat[7] 
		+ const_vec[1] * design_mat[1] * design_mat[6] + const_vec[2] * design_mat[0] * design_mat[4] - const_vec[2] * design_mat[1] * design_mat[3]) 
		/ (design_mat[0] * design_mat[4] * design_mat[8] - design_mat[0] * design_mat[5] * design_mat[7] - design_mat[1] * design_mat[3] * design_mat[8]
		+ design_mat[1] * design_mat[5] * design_mat[6] + design_mat[2] * design_mat[3] * design_mat[7] - design_mat[2] * design_mat[4] * design_mat[6]);

	return 1;
}

int DefineAdaptWin_fst(int PatchNum, float *dsm_data, BYTE *img_data, int w, int h, int bands, double w_dis, double w_color, double w_disp,
	PixelInfo **adaptWin, int *validNum)
{
	if (w < PatchNum || h < PatchNum)
	{
		printf("Image size is too small\n");
		return 0;
	}
	double w_sum = w_dis + w_color + w_disp;
	w_dis /= w_sum;
	w_color /= w_sum;
	w_disp /= w_sum;

	//initialize
	memset(validNum, 0, sizeof(int)*w*h);
	int pos = 0;
	int indx = 0;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w;x++)
		{
			pos = y*w + x;
			indx = validNum[pos];
			adaptWin[pos][indx].y = y;
			adaptWin[pos][indx].x = x;
			adaptWin[pos][indx].P = -1;
			adaptWin[pos][indx].d = dsm_data[pos];
			adaptWin[pos][indx].deltaC = 0;
			adaptWin[pos][indx].deltaD = 0;
			adaptWin[pos][indx].deltaS = 0;
			validNum[pos]++;
		}

	//adaptive window by region grow
	int maxSize = 10000;
	PixelInfo *neighbors = new PixelInfo[maxSize];
	PixelInfo *adaptWin_new = new PixelInfo[maxSize];
	//grow direction
	int stepx[4] = { 1, -1, 0, 0 };
	int stepy[4] = { 0, 0, 1, -1 };
	int cur_iter = 0;
	int cur_num = 0;
	int PatchNum_new = 0;
	int neighborNum = 0; //new added neighbor pixel num
	int validNum_new = 0; //new added valid pixel num
	int ifrepeat_neigbor, ifrepeat_adaptwin;
	PixelInfo curPixel;
	int x_cur, y_cur;
	int cx, cy;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			pos = y*w + x;
			if (isnan(dsm_data[pos])==true)
				continue;

			cur_num = validNum[pos]; //number of neighbor pixels that we already have 
			PatchNum_new = PatchNum - cur_num; // the number of neighbor pixels that we need to newly add.

			cx = x; cy = y;
			cur_iter = 0;
			curPixel = adaptWin[pos][0];
			neighborNum = 0;
			while (cur_iter < PatchNum_new)
			{
				x_cur = curPixel.x;
				y_cur = curPixel.y;
				for (int j = 0; j < 4; j++)
				{
					int nx, ny;
					nx = x_cur + stepx[j];
					ny = y_cur + stepy[j];

					ifrepeat_neigbor = CheckIfRepeat(nx, ny, neighbors, neighborNum);
					ifrepeat_adaptwin = CheckIfRepeat(nx, ny, adaptWin[pos], cur_num);
					if (nx >= 0 && nx < w && ny >= 0 && ny < h && ifrepeat_neigbor == 0 && ifrepeat_adaptwin == 0 && isnan(dsm_data[ny*w + nx]) == false)
					{
						PixelInfo neighborPixel;
						neighborPixel.x = nx;
						neighborPixel.y = ny;
						neighborPixel.d = dsm_data[ny*w + nx];

						double deltaColor, deltaDis, deltaDisp;
						deltaDis = sqrt((nx - cx)*(nx - cx) + (ny - cy)*(ny - cy));
						deltaDisp = fabs(dsm_data[ny*w + nx] - dsm_data[cy*w + cx]);
						if (bands == 1)
						{
							deltaColor = abs(img_data[ny*w + nx] - img_data[cy*w + cx]);
						}
						else
						{
							deltaColor = (abs(img_data[0 * w*h + ny*w + nx] - img_data[0 * w*h + cy*w + cx])
								+ abs(img_data[1 * w*h + ny*w + nx] - img_data[1 * w*h + cy*w + cx])
								+ abs(img_data[2 * w*h + ny*w + nx] - img_data[2 * w*h + cy*w + cx])) / 3.0;
						}
						neighborPixel.deltaC = deltaColor;
						neighborPixel.deltaS = deltaDis;
						neighborPixel.deltaD = deltaDisp;
						neighborPixel.P = w_color*deltaColor + w_dis*deltaDis + w_disp*deltaDisp;

						neighbors[neighborNum] = neighborPixel;
						neighborNum++;
					}
				}

				//find lowest score in the neighbor pixels
				int bestID = -1;
				double bestScore = 999999;
				for (int i = 0; i < neighborNum; i++)
				{
					if (neighbors[i].P >= 0 && neighbors[i].P < bestScore)
					{
						bestScore = neighbors[i].P;
						bestID = i;
					}
				}

				if (bestID == -1)
				{
					break;
				}
				else
				{
					neighbors[bestID].P = -1;
				}
				adaptWin_new[cur_iter] = neighbors[bestID];
				cur_iter++;

				curPixel = adaptWin_new[cur_iter - 1];
			}

			validNum_new = cur_iter;
			for (int i = 0; i < validNum_new;i++)
			{
				adaptWin[pos][cur_num + i] = adaptWin_new[i];
			}
			validNum[pos] += validNum_new;

			//update adapt win of neighbor pixels
			for (int i = 0; i < validNum_new; i++)
			{
				x_cur = adaptWin_new[i].x;
				y_cur = adaptWin_new[i].y;

				if (validNum[y_cur*w + x_cur] < PatchNum)
				{
					adaptWin[y_cur*w + x_cur][validNum[y_cur*w + x_cur]] = adaptWin[pos][0];
					validNum[y_cur*w + x_cur]++;
				}
			}
		}

	//free memory
	delete[]neighbors; neighbors = NULL;
	delete[]adaptWin_new; adaptWin_new = NULL;

	return 1;
}

int DefineAdaptWin(int cx, int cy, int PatchNum, float *dsm_data, BYTE *img_data, int w, int h, int bands, double w_dis, double w_color, double w_disp,
	PixelInfo *adaptWin, int &validNum, PixelInfo *neighbors)
{
	//adaptWin = new PixelInfo[PatchNum];
	validNum = 0;

	int cur_iter = 0;

	if (cx<0 || cx>w - 1 || cy<0 || cy>h - 1)
		return 0;

	if (isnan(dsm_data[cy*w + cx]) == true)
		return 0;

	double w_sum = w_dis + w_color + w_disp;
	w_dis /= w_sum;
	w_color /= w_sum;
	w_disp /= w_sum;

	PixelInfo curPixel;
	curPixel.x = cx;
	curPixel.y = cy;
	curPixel.d = dsm_data[cy*w + cx];
	curPixel.P = -1;
	curPixel.deltaC = 0;
	curPixel.deltaD = 0;
	curPixel.deltaS = 0;

	adaptWin[cur_iter] = curPixel;
	cur_iter++;

	int stepx[4] = { 1, -1, 0, 0 };
	int stepy[4] = { 0, 0, 1, -1 };

// 	vector <PixelInfo> neighbors;
// 	neighbors.push_back(curPixel);
	int neighborNum = 0;
	neighbors[0] = curPixel;
	neighborNum++;
	while (cur_iter<PatchNum)
	{
		curPixel = adaptWin[cur_iter - 1];
		int x_cur, y_cur;
		x_cur = curPixel.x;
		y_cur = curPixel.y;
		for (int j = 0; j < 4; j++)
		{
			int nx, ny;
			nx = x_cur + stepx[j];
			ny = y_cur + stepy[j];

			int ifrepeat = CheckIfRepeat(nx, ny, neighbors, neighborNum);
			if (nx >= 0 && nx < w && ny >= 0 && ny < h && ifrepeat == 0 && isnan(dsm_data[ny*w + nx])==false)
			{
				PixelInfo neighborPixel;
				neighborPixel.x = nx;
				neighborPixel.y = ny;
				neighborPixel.d = dsm_data[ny*w + nx];

				double deltaColor, deltaDis, deltaDisp;
				deltaDis = sqrt((nx - cx)*(nx - cx) + (ny - cy)*(ny - cy));
				deltaDisp = fabs(dsm_data[ny*w + nx] - dsm_data[cy*w + cx]);
				if (bands==1)
				{
					deltaColor = abs(img_data[ny*w + nx] - img_data[cy*w + cx]);
				}
				else
				{
					deltaColor = (abs(img_data[0 * w*h + ny*w + nx] - img_data[0 * w*h + cy*w + cx])
								+ abs(img_data[1 * w*h + ny*w + nx] - img_data[1 * w*h + cy*w + cx])
								+ abs(img_data[2 * w*h + ny*w + nx] - img_data[2 * w*h + cy*w + cx])) / 3.0;
				}
				neighborPixel.deltaC = min(deltaColor, 20); //this will need further consideration
				neighborPixel.deltaS = deltaDis;
				neighborPixel.deltaD = deltaDisp;
				neighborPixel.P = w_color*deltaColor + w_dis*deltaDis + w_disp*deltaDisp;

// 				w_color = 1 - exp(-deltaColor / 10);
// 				w_dis = 1 - exp(-deltaDis / sqrt(PatchNum));
// 				w_disp = 1 - exp(deltaDisp / 2);
// 				neighborPixel.P = w_color + w_dis + w_disp;

				neighbors[neighborNum] = neighborPixel;
				neighborNum++;
			}
		}


		//find lowest score in the neighbor pixels
		int bestID = -1;
		double bestScore = 999999;
		for (int i = 0; i < neighborNum;i++)
		{
			if (neighbors[i].P>=0 && neighbors[i].P<bestScore)
			{
				bestScore = neighbors[i].P;
				bestID = i;
			}
		}

		if (bestID==-1)
		{
			break;
		}
		else
		{
			neighbors[bestID].P = -1;
		}
		adaptWin[cur_iter] = neighbors[bestID];
		cur_iter++;
	}

	validNum = cur_iter;

	return 1;
}

int CheckIfRepeat(int x, int y, PixelInfo *array, int ptNum)
{
	int cx, cy;
	for (int i = 0; i < ptNum; i++)
	{
		cx = array[i].x;
		cy = array[i].y;

		if (cx == x && cy == y)
		{
			return 1;
		}
	}

	return 0;
}

int CheckIfRepeat(int x, int y, vector<PixelInfo> array)
{
	int num = array.size();
	for (int i = 0; i < num;i++)
	{
		int cx, cy;
		cx = array[i].x;
		cy = array[i].y;

		if (cx==x && cy==y)
		{
			return 1;
		}
	}

	return 0;
}