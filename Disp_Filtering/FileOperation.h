#pragma once
#include <vector>
#include <string>
using namespace std;

#define  Str_Max_Len 1024

typedef struct
{
	char Root_folder_path[Str_Max_Len];
	int patchNum;
	char ** img_paths;
	char ** rpb_paths;
	char ** refined_rpb_paths;
}Long_Strip_Img;


typedef struct
{
	char img_rpb_folder_path[Str_Max_Len];

	char AOI_kml_path[Str_Max_Len];

	int AOI_cutting_extra_size;

	int Plane_rectification_tile_size;

	int Plane_rectification_overlap_size;

	double Pair_overlap_area;

	int mch_win_size;

	double mch_epi_stepH;

	int mch_epi_search_range;

}PrjPara;

class CFileOperation
{
public:
	CFileOperation();
	~CFileOperation();

public:
	int GetAllFilePathsInDir(char *dir_path, Long_Strip_Img *& strips, int & stripNum);
	int GetFileName(const char *path, char *name);
	int ReadPrjFile(char *input_prj_path, PrjPara &para);
	int FindDirPath_prj(const char *prjPath, char *dirPath, bool ifContainPrjName);
	int FindAllPath(char *prjPath, char *FileName, char *dstList, char *dstPath);
	int ReadKML(char *kmlPath, double *&lat, double *&lon, double *&hei, int &nodeNum);
	int ReadKml4(char *kmlPath, double *lat, double *lon, double *hei);
	int ReadKML(char *kmlPath, double &minx, double &miny, double &maxx, double &maxy, double &avgHei, char *zoneID);
	int AddCharacterToPath(char *path, char c);
	int GetLineNuminFiles(char *filePath, int &Num);

public:
	void getFiles(string path, vector <string> &files); 
};

