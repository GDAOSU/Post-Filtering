#include "stdafx.h"
#include "FileOperation.h"
#include <stdio.h>
#include <io.h>
#include <fstream>
#include <string>
#include <algorithm>
#include "direct.h"
#include "BundleAdjust.h"


CFileOperation::CFileOperation()
{
}


CFileOperation::~CFileOperation()
{
}

int CFileOperation::GetAllFilePathsInDir(char *dir_path, Long_Strip_Img *& strips, int & stripNum)
{
	string path = dir_path;
	vector <string> files;

	//Read folders of each strip images
	getFiles(path, files);

	stripNum = files.size();

	strips = new Long_Strip_Img[stripNum];

	if (stripNum<2)
	{
		//free memory;
		delete[]strips; strips = NULL;
		stripNum = 0;
		files.clear();
		vector<string>().swap(files);

		printf("%s\n", "Number of valid satellite images or rpb files is too small!");

		return 0;
	}
	else
	{
		//check the number of folders
		int total_num = stripNum;
		stripNum = 0;
		for (int i = 0; i < total_num; i++)
		{
			const char *str = files[i].c_str();
			int len = strlen(str);
			bool ifdot = false;
			for (int j = len - 1; j >= 0;j--)
			{
				if (str[j]=='\\')
				{
					break;
				}
				if (str[j]=='.')
				{
					ifdot = true;
					break;
				}
			}

			if (ifdot==false) //it is folder instead of file
			{
				stripNum++;
			}
		}

		if (stripNum<2)
		{
			//free memory;
			delete[]strips; strips = NULL;
			stripNum = 0;
			files.clear();
			vector<string>().swap(files);

			printf("%s\n", "Number of valid satellite images or rpb files is too small!");

			return 0;
		}

		strips = new Long_Strip_Img[stripNum];
		int k = 0;
		for (int i = 0; i < total_num; i++)
		{
			const char *str = files[i].c_str();
			int len = strlen(str);
			bool ifdot = false;
			for (int j = len - 1; j >= 0; j--)
			{
				if (str[j] == '\\')
				{
					break;
				}
				if (str[j] == '.')
				{
					ifdot = true;
					break;
				}
			}

			if (ifdot == false) //it is folder instead of file
			{
				strcpy(strips[k].Root_folder_path, files[i].c_str());
				k++;
			}
		}

		for (int i = 0; i < stripNum;i++)
		{
			vector <string> cur_folder;
			getFiles(strips[i].Root_folder_path, cur_folder);

			int imgNum = 0;
			int localNum = cur_folder.size();
			for (int j = 0; j < localNum;j++)
			{
				const char *str = cur_folder[j].c_str();
				const char *ext = str + strlen(str) - 3;

				if (strcmp(ext, "TIF") == 0 || strcmp(ext, "tif") == 0)
				{
					imgNum++;
				}
			}

			if (imgNum==0)
			{
				printf("%s", "Found an empty Folder!");
				stripNum = 0;
				delete[]strips; strips = NULL;

				//free memory;
				files.clear();
				vector<string>().swap(files);

				return 0;
			}

			char **img_paths = new char *[imgNum];
			char **rpb_paths = new char *[imgNum];
			for (int j = 0; j < imgNum; j++)
			{
				img_paths[j] = new char[Str_Max_Len];
				rpb_paths[j] = new char[Str_Max_Len];
			}

			int k = 0;
			for (int j = 0; j < localNum; j++)
			{
				const char *str = cur_folder[j].c_str();
				const char *ext = str + strlen(str) - 3;

				if (strcmp(ext, "TIF") == 0 || strcmp(ext, "tif") == 0)
				{
					strcpy(img_paths[k], cur_folder[j].c_str());
					k++;
				}
			}

			for (int j = 0; j < imgNum; j++)
			{
				char imgName[Str_Max_Len];
				char rpbName[Str_Max_Len];

				GetFileName(img_paths[j], imgName);

				bool rpb_flag = false;
				for (int k = 0; k < localNum; k++)
				{
					const char *str = cur_folder[k].c_str();
					const char *ext = str + strlen(str) - 3;

					if (strcmp(ext, "RPB") == 0 || strcmp(ext, "rpb") == 0)
					{
						GetFileName(cur_folder[k].c_str(), rpbName);
					}

					if (strcmp(imgName, rpbName) == 0)
					{
						strcpy(rpb_paths[j], cur_folder[k].c_str());
						rpb_flag = true;
						break;
					}
				}

				if (rpb_flag == false)
				{
					printf("%s\n", "Cannot find the corresponding RPB files");
					cur_folder.clear();
					vector<string>().swap(cur_folder);

					files.clear();
					vector<string>().swap(files);

					strips = 0;
					delete[]strips; strips = NULL;

					for (int k = 0; k < imgNum; k++)
					{
						delete []img_paths[k];
						delete []rpb_paths[k];
					}
					delete[]img_paths; img_paths = NULL;
					delete[]rpb_paths; rpb_paths = NULL;

					return 0;
				}
			}

			strips[i].patchNum = imgNum;
			strips[i].img_paths = new char *[imgNum];
			strips[i].rpb_paths = new char *[imgNum];

			for (int j = 0; j < imgNum; j++)
			{
				strips[i].img_paths[j] = new char[Str_Max_Len];
				strips[i].rpb_paths[j] = new char[Str_Max_Len];

				strcpy(strips[i].img_paths[j], img_paths[j]);
				strcpy(strips[i].rpb_paths[j], rpb_paths[j]);
			}
			

			//free memory
			cur_folder.clear();
			vector<string>().swap(cur_folder);

			for (int j = 0; j < imgNum; j++)
			{
				delete[]img_paths[j];
				delete[]rpb_paths[j];
			}
			delete[]img_paths; img_paths = NULL;
			delete[]rpb_paths; rpb_paths = NULL;
		}
	}

	//free memory;
	files.clear();
	vector<string>().swap(files);

	return 1;
}

int CFileOperation::FindDirPath_prj(const char *prjPath, char *dirPath, bool ifContainPrjName)
{
	if (ifContainPrjName == true)
	{
		int len;
		len = strlen(prjPath);

		if (len <= 3)
			return 0;

		int i;
		for (i = len - 1; i >= 0; i--)
		{
			if (prjPath[i] == '\\')
				break;
		}

		if (i < 0) return 0;

		int j;
		for (j = 0; j <= i; j++)
		{
			dirPath[j] = prjPath[j];
		}
		dirPath[j] = 0;

		char FileName[255] = { 0 };
		for (j = i + 1; j < len; j++)
		{
			if (prjPath[j] == '.')
				break;
			else
				FileName[j - (i + 1)] = prjPath[j];
		}
		FileName[j] = 0;

		strcat_s(dirPath, 255, FileName);
	}
	else
	{
		int len;
		len = strlen(prjPath);

		if (len <= 3)
			return 0;

		int i;
		for (i = len - 1; i >= 0; i--)
		{
			if (prjPath[i] == '\\')
				break;
		}

		if (i < 0) return 0;

		int j;
		for (j = 0; j < i; j++)
		{
			dirPath[j] = prjPath[j];
		}
		dirPath[j] = 0;
	}
	return 1;
}

int CFileOperation::GetLineNuminFiles(char *filePath, int &Num)
{
	FILE *fp = fopen(filePath, "r");
	char str[255];
	Num = 0;
	while (!feof(fp))
	{
		fgets(str, 255, fp);
		Num++;
	}
	Num--;
	fclose(fp); fp = NULL;
	return 1;
}

int CFileOperation::ReadPrjFile(char *input_prj_path, PrjPara &para)
{
	FILE  *fp = fopen(input_prj_path, "r");
	strcpy(para.img_rpb_folder_path, "");
	strcpy(para.AOI_kml_path, "");
	para.AOI_cutting_extra_size = 0;
	para.Plane_rectification_overlap_size = 0;
	para.Plane_rectification_tile_size = 0;
	para.Pair_overlap_area = 0;
	para.mch_win_size = 0;
	para.mch_epi_stepH = 0;
	para.mch_epi_search_range = 0;

	char keywords[Str_Max_Len];
	while (!feof(fp))
	{
		fscanf(fp, "%s", keywords);
		if (strcmp(keywords, "END") == 0 || strcmp(keywords, "end") == 0 || strcmp(keywords, "End") == 0)
		{
			break;
		}
		else if (strcmp(keywords, "img&rpb") == 0 || strcmp(keywords, "IMG&RPB") == 0 || strcmp(keywords, "Img&Rpb") == 0)
		{
			char str[Str_Max_Len];
			fscanf(fp, "%s", str);
			strcpy(para.img_rpb_folder_path, str);
		}
		else if (strcmp(keywords, "kml") == 0 || strcmp(keywords, "KML") == 0 || strcmp(keywords, "Kml") == 0)
		{
			char str[Str_Max_Len];
			fscanf(fp, "%s", str);
			strcpy(para.AOI_kml_path, str);
		}
		else if (strcmp(keywords, "AOI_cutting_extra_size") == 0)
		{
			fscanf(fp, "%d", &para.AOI_cutting_extra_size);
		}
		else if (strcmp(keywords, "Plane_rectification_tile_size") == 0)
		{
			fscanf(fp, "%d", &para.Plane_rectification_tile_size);
		}
		else if (strcmp(keywords, "Plane_rectification_overlap_size") == 0)
		{
			fscanf(fp, "%d", &para.Plane_rectification_overlap_size);
		}
		else if (strcmp(keywords, "Pair_overlap_area") == 0)
		{
			fscanf(fp, "%lf", &para.Pair_overlap_area);
		}
		else if (strcmp(keywords, "Matching_window_size") == 0)
		{
			fscanf(fp, "%d", &para.mch_win_size);
		}
		else if (strcmp(keywords, "Matching_epi_stepH") == 0)
		{
			fscanf(fp, "%lf", &para.mch_epi_stepH);
		}
		else if (strcmp(keywords, "Matching_epi_search_range") == 0)
		{
			fscanf(fp, "%d", &para.mch_epi_search_range);
		}
	}

	fclose(fp); fp = NULL;
	return 1;
}

int CFileOperation::GetFileName(const char *path, char *name)
{
	char Drive[Str_Max_Len];
	char dir[Str_Max_Len];
	char Ext[10];
	_splitpath(path, Drive, dir, name, Ext);
	return 1;
}

void CFileOperation::getFiles(string path, vector <string> &files)
{
	//文件句柄  
	intptr_t   hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					//getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				{
					files.push_back(p.assign(path).append("\\").append(fileinfo.name));
				}
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		}while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

// int CFileOperation::ReadKML(char *kmlPath, double &minx, double &miny, double &maxx, double &maxy, double &avgHei, char *zoneID)
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
// 	double *Lat, *Lon, *Hei;
// 	Lat = new double[ptNum];
// 	Lon = new double[ptNum];
// 	Hei = new double[ptNum];
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
// 		sscanf(str_coordinate, data_type[i], &Lon[i], &Lat[i], &Hei[i]);
// 	}
// 
// 	//found bounding box of aoi
// 	avgHei = 0;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		avgHei += Hei[i];
// 	}
// 	avgHei /= ptNum;
// 
// 	LL2UTM(Lat[0], Lon[0], minx, miny, zoneID);
// 	maxx = minx; maxy = miny;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		double tX, tY;
// 		LL2UTM_fixedZone(Lat[i], Lon[i], zoneID, tX, tY);
// 
// 		if (tX < minx)
// 		{
// 			minx = tX;
// 		}
// 		if (tY < miny)
// 		{
// 			miny = tY;
// 		}
// 		if (tX > maxx)
// 		{
// 			maxx = tX;
// 		}
// 		if (tY > maxy)
// 		{
// 			maxy = tY;
// 		}
// 	}
// 
// 	//free memory
// 	fclose(fp); fp = NULL;
// 	delete[]Lat; Lat = NULL;
// 	delete[]Lon; Lon = NULL;
// 	delete[]Hei; Hei = NULL;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		delete[]data_type[i]; data_type[i] = NULL;
// 	}
// 	delete[]data_type; data_type = NULL;
// 	return 1;
// }

int CFileOperation::ReadKML(char *kmlPath, double *&lat, double *&lon, double *&hei, int &nodeNum)
{
	//Read kml
	FILE *fp = fopen(kmlPath, "r");
	char str[255]; //each line
	char str_content[255]; //contest of each line
	char str_coordinate[255] = ""; //coordinates that we need
	while (!feof(fp))
	{
		fgets(str, 255, fp);
		sscanf(str, "%s", str_content);
		if (strcmp(str_content, "<coordinates>") == 0)
		{
			fgets(str, 255, fp);
			sscanf(str, "%s", str_content);
			while (strcmp(str_content, "</coordinates>") != 0)
			{
				strcat(str_coordinate, str);
				strcat(str_coordinate, "\t");
				fgets(str, 255, fp);
				sscanf(str, "%s", str_content);
			}

			break;
		}
	}

	int ptNum = 0;
	for (int i = 0; i < strlen(str_coordinate); i++)
	{
		if (str_coordinate[i] == ',')
		{
			ptNum++;
		}
	}
	ptNum /= 2;

	//	double *Lat, *Lon, *Hei;
	nodeNum = ptNum;
	lat = new double[ptNum];
	lon = new double[ptNum];
	hei = new double[ptNum];

	char **data_type = new char *[ptNum];
	for (int i = 0; i < ptNum; i++)
	{
		data_type[i] = new char[255];
	}
	for (int i = 0; i < ptNum; i++)
	{
		strcpy(data_type[i], "");
	}
	for (int i = 0; i < ptNum; i++)
		for (int j = 0; j < ptNum; j++)
		{
			if (j == i)
			{
				strcat(data_type[i], "%lf,%lf,%lf ");
			}
			else
			{
				strcat(data_type[i], "%*lf,%*lf,%*lf ");

			}
		}


	for (int i = 0; i < ptNum; i++)
	{
		sscanf(str_coordinate, data_type[i], &lon[i], &lat[i], &hei[i]);
	}

	//found bounding box of aoi
	// 	avgHei = 0;
	// 	for (int i = 0; i < ptNum; i++)
	// 	{
	// 		avgHei += Hei[i];
	// 	}
	// 	avgHei /= ptNum;
	// 
	// 	LL2UTM(Lat[0], Lon[0], minx, miny, zoneID);
	// 	maxx = minx; maxy = miny;
	// 	for (int i = 0; i < ptNum; i++)
	// 	{
	// 		double tX, tY;
	// 		LL2UTM_fixedZone(Lat[i], Lon[i], zoneID, tX, tY);
	// 
	// 		if (tX < minx)
	// 		{
	// 			minx = tX;
	// 		}
	// 		if (tY < miny)
	// 		{
	// 			miny = tY;
	// 		}
	// 		if (tX > maxx)
	// 		{
	// 			maxx = tX;
	// 		}
	// 		if (tY > maxy)
	// 		{
	// 			maxy = tY;
	// 		}
	// 	}

	//free memory
	fclose(fp); fp = NULL;
	// 	delete[]Lat; Lat = NULL;
	// 	delete[]Lon; Lon = NULL;
	// 	delete[]Hei; Hei = NULL;
	for (int i = 0; i < ptNum; i++)
	{
		delete[]data_type[i]; data_type[i] = NULL;
	}
	delete[]data_type; data_type = NULL;

	return 1;
}

int CFileOperation::ReadKml4(char *kmlPath, double *lat, double *lon, double *hei)
{
	double *lat_all, *lon_all, *hei_all;
	int nodeNum = 0;
	ReadKML(kmlPath, lat_all, lon_all, hei_all, nodeNum);
	nodeNum--;

	if (nodeNum<3)
	{
		printf("The node in KML file is too small\n");
		//free memory
		delete[]lat_all; lat_all = NULL;
		delete[]lon_all; lon_all = NULL;
		delete[]hei_all; hei_all = NULL;

		return 0;
	}

	double lat_min = lat_all[0], lon_min = lon_all[0], lat_max = lat_all[0], lon_max = lon_all[0];
	double avgHei = 0;
	for (int i = 1; i < nodeNum;i++)
	{
		if (lat_min>lat_all[i])
			lat_min = lat_all[i];
		if (lat_max < lat_all[i])
			lat_max = lat_all[i];
		if (lon_min > lon_all[i])
			lon_min = lon_all[i];
		if (lon_max < lon_all[i])
			lon_max = lon_all[i];
	}

	for (int i = 0; i < nodeNum;i++)
	{
		avgHei += hei_all[i];
	}
	avgHei /= nodeNum;

	//compute the corners of bouding box
	for (int i = 0; i < 4;i++)
	{
		hei[i] = avgHei;
	}

	//clock wise
	lat[0] = lat_min; lon[0] = lon_min;
	lat[1] = lat_min; lon[1] = lon_max;
	lat[2] = lat_max; lon[2] = lon_max;
	lat[3] = lat_max; lon[3] = lon_min;

	//free memory
	delete[]lat_all; lat_all = NULL;
	delete[]lon_all; lon_all = NULL;
	delete[]hei_all; hei_all = NULL;

	return 1;
}

int CFileOperation::FindAllPath(char *prjPath, char *FileName, char *dstList, char *dstPath)
{
	char prjRootDir[255];
	FindDirPath_prj(prjPath, prjRootDir, false);
	char prjFilePath[255];
	FindDirPath_prj(prjPath, prjFilePath, true);

	if (FileName == "")
	{
		strcpy_s(dstList, 255, prjPath);
		strcpy_s(dstPath, 255, prjRootDir);
	}
	else if (strcmp(FileName, "Plane_Rectification") == 0)
	{
		strcpy_s(dstList, 255, prjFilePath);
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Plane_Rectification");
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Plane_Rectification_list.txt");

		FindDirPath_prj(dstList, dstPath, false);
	}
	else if (strcmp(FileName, "Feature_Matching") == 0)
	{
		strcpy_s(dstList, 255, prjFilePath);
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Feature_Matching");
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Correspondences_list.txt");

		FindDirPath_prj(dstList, dstPath, false);
	}
	else if (strcmp(FileName, "img&rpc") == 0)
	{
		strcpy_s(dstList, 255, prjFilePath);
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "img&rpc");
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "AOI_img&rpc_list.txt");

		FindDirPath_prj(dstList, dstPath, false);
	}
	else if (strcmp(FileName, "Pair_selection") == 0)
	{
		strcpy_s(dstList, 255, prjFilePath);
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Pair_selection");
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Pair_selection_list.txt");

		FindDirPath_prj(dstList, dstPath, false);
	}
	else if (strcmp(FileName, "Bundle_Adjustment") == 0)
	{
		strcpy_s(dstList, 255, prjFilePath);
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Bundle_Adjustment");
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Bundle_Adjustment_list.txt");

		FindDirPath_prj(dstList, dstPath, false);
	}
	else if (strcmp(FileName, "Coarse_Registration") == 0)
	{
		strcpy_s(dstList, 255, prjFilePath);
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Coarse_Registration");
		strcat_s(dstList, 255, "\\");
		strcat_s(dstList, 255, "Coarse_Registration_list.txt");

		FindDirPath_prj(dstList, dstPath, false);
	}
	// 	else if (strcmp(FileName, "PtFilter") == 0)
	// 	{
	// 		strcpy_s(dstList, 255, prjFilePath);
	// 		strcat_s(dstList, 255, "\\");
	// 		strcat_s(dstList, 255, "PtFilter");
	// 		strcat_s(dstList, 255, "\\");
	// 		strcat_s(dstList, 255, "PtFilter.txt");
	// 
	// 		FindRootDir(dstList, dstPath);
	// 	}
	// 	else if (strcmp(FileName, "DenseMatching") == 0)
	// 	{
	// 		strcpy_s(dstList, 255, prjFilePath);
	// 		strcat_s(dstList, 255, "\\");
	// 		strcat_s(dstList, 255, "DenseMatching");
	// 		strcat_s(dstList, 255, "\\");
	// 		strcat_s(dstList, 255, "DenseMatching.txt");
	// 
	// 		FindRootDir(dstList, dstPath);
	// 	}
	// 	else if (strcmp(FileName, "PtFusion") == 0)
	// 	{
	// 		strcpy_s(dstList, 255, prjFilePath);
	// 		strcat_s(dstList, 255, "\\");
	// 		strcat_s(dstList, 255, "PtFusion");
	// 		strcat_s(dstList, 255, "\\");
	// 		strcat_s(dstList, 255, "PtFusion.txt");
	// 
	// 		FindRootDir(dstList, dstPath);
	// 	}
	else
	{
		dstList = "";
		dstPath = "";
	}

	return 1;
}

int CFileOperation::AddCharacterToPath(char *path, char c)
{
	int len = strlen(path);
	if (path[len - 1]!=c)
	{
		char str[1] = { c };
		strcat(path, str);
	}
	return 1;
}