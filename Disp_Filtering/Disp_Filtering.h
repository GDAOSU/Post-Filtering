#pragma once
#include "imagebase.h"
#include "FileOperation.h"
#include "RPC_Fun.h"
#include "LL_UTM.h"
#include "ImageFeatureMatching.h"
#include <vector>
using namespace std;

#define EPSION 1.0e-20

#define MaxLen 1024
// #define Overlap 3
// #define avgH 100
// #define PI 3.14159265359
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

// typedef struct
// {
// 	double gX, gY, gZ; //ground coordinates
// }Pt3D;

// typedef struct  
// {
// 	double Lat, Lon, Hei;
// 	int imgNum;
// 	int *ID;
// 	double *rows, *cols;
// }PtTrack;
// 
// typedef struct  
// {
// 	double a, b, c, d, e, f;
// 	double Lat, Lon, Hei;
// }Bundle_Para;

class CBundleAdjust
{
public:
	CBundleAdjust();
	~CBundleAdjust();

public:
	int BundleAdjustment_Sat(char *input_path, char *output_path);

private:
	//以后这个类只放框架函数，功能类函数全放到具体功能的类里面
	int Generate_AOI_Images(char *input_path);
	int Generate_Plane_Rectification(char *input_path);
	int Pair_selection(char *input_path);
	int Feature_matching(char *input_path);
	int BundleAdjustment_RPC(char *input_path);

public:
	char m_prj_path[MaxLen];
	double m_avgH;
	double m_rectification_GSD;
	int m_rect_tile_size;
	int m_rect_overlap_size;
	char m_zoneID[10];
	double m_pair_overlap_area;
	double m_epi_stepH;
	int m_Virtual_GCP_Plane_Size;
	int m_Virtual_GCP_Height_Size;

public:
	/////////////////////////////////////Reader/////////////////////////////////////
	int ReadStereoListInfor(char *Stereo_list_path, MchPtInfor *& mchPairs, int &pairNum);

	/*
	已加入FileOperation.h的函数，以后注意清理替换掉
	*/
	/*
	int GetFileName(char *path, char *name);
	int ReadPrjFile(char *input_path, int &satNum, char **& imgPath, char **&rpcPath); //Read prj file
	//these two functions can be merged into one: FindDirPath_prj
	int FindRootDir(char *FilePath, char *RootPath);
	int FindPrjDir(char *FilePath, char *RootPath);
	int FindAllPath(char *prjPath, char *FileName, char *dstList, char *dstPath);
	int ReadKML(char *kmlPath, double *&lat, double *&lon, double *&hei, int &nodeNum);
	int ReadKML(char *kmlPath, double &minx, double &miny, double &maxx, double &maxy, double &avgHei, char *zoneID);
	int GetLineNuminFiles(char *filePath, int &Num);
	*/
	/////////////////////////////////////END/////////////////////////////////////

	/////////////////////////////////////RPC operation/////////////////////////////////////
	//这三个貌似已经重新写新的了，可能不在需要了，以后需要检查
	int Compute_AOI_IMG_RPC(char **imgPaths, char **rpcPaths, int imgNum, char *kmlPath, double extraBuffer, 
		char *output_img_path, char *outout_rpc_path);
	int Compute_AOI_IMG_RPC_From_Two_Patches(char imgPath[2][255], char rpcPath[2][255],
		char *kmlPath, double extraBuffer,
		char *output_img_path, char *outout_rpc_path);
	int Check_AOI_Overlap(char *imgPaths, char *rpcPaths, double minX, double minY, double maxX, double maxY, char zoneID[10], double avgHei,
		double &min_px, double &min_py, double &max_px, double &max_py, bool &ifcover);
	//END

	int Merge_Img_Tiles(char **imgPaths, char **rpcPaths, int coverNum, int *sx, int *sy, int *imgw, int *imgh, int *imgID, int origin_ID, char *zoneID,
		char *merge_result_path, char *merge_rpc_path);
	int Merge_Img_Tiles(char **imgPaths, char **rpcPaths, int coverNum, int *sx, int *sy, int *imgw, int *imgh, int *imgID, char *zoneID, 
		char *output_folder);
	int Compute_AOI_images(char *input_path, char *kml_path, char *output_folder);
	int Merge_Img_Two_Tiles(char imgPaths[2][255], char rpcPaths[2][255], int *sx, int *sy,
		int *imgw, int *imgh, int origin_ID, char *zoneID,
		char *merge_result_path, char *merge_rpc_path);
	int Compute_RPC_Merge_Img_Tiles(char **imgPaths, char **rpcPaths, int coverNum, int *sx, int *sy, int *imgw, int *imgh, int *imgID, int origin_ID,
		char *merge_result_path, char *merge_rpc_path);

	//deleta = a1*sin(b1*x+c1)
	int ComputeSystemErrorBetweenTiles(int sx1, int sy1, int w1, int h1,
		int sx2, int sy2, int w2, int h2,
		RPC rpc1, RPC rpc2, double tx, double ty, double &ax1, double &bx1, double &cx1, double &ay1, double &by1, double &cy1);

	int ComputeCoordinateBeforeCorrection(double x_acc, double y_acc, double ax1, double bx1, double cx1,
		double ay1, double by1, double cy1, double &x, double &y, double &dx, double &dy);

	int ComputeRPCfromVirtualGCPs(double *ix, double *iy, double *lat, double *lon, double *hei, int ptNum,
		RPC &rpc, double &maxd);

	/*
	RPC 相关函数
	int ReadRpc_singleFile(char *rpcPath, RPC &rpcs);
	int ReadRpcInfo(char **rpcPath, int satNum, RPC *& rpcs);

	int Check_AOI_Overlap(char *imgPaths, char *rpcPaths, double *lat, double *lon, double *hei, int nodeNum, int extra_size,
	int &min_px, int &min_py, int &max_px, int &max_py, bool &ifcover);

	int RPC_Ground2Image(double Lat, double Lon, double Height, RPC rpc, double &row, double &column);
	int RPC_Ground2Image_normalize(double P, double L, double H, RPC rpc, double &r, double &c);
	int RPC_Image2Ground_normalize(double r, double c, double H, RPC rpc, double &P, double &L);
	int RPC_Image2Ground(double row, double column, double Height, RPC rpc, double &lat, double &lon);
	int RPC_coefficient(double P, double L, double H, int ID, double &result);
	int RPC_coefficient_derivative(double P, double L, double H, int ID, char var, double &result);
	int RPC_ForwardIntersection(RPC rpc1, RPC rpc2, double ix1, double iy1, double ix2, double iy2, double &Lat, double &Lon, double &Hei);

	int GetCornerPostions(int *imw, int *imh, double *height, RPC *rpcs, int satNum, double *&lats, double *&lons);

	int ReadTfwInforFromList(char *rectif_list_path, int ID, TfwPara &para);

	int ComputeTranslateBetweenTiles(int sx1, int sy1, int w1, int h1,
	int sx2, int sy2, int w2, int h2,
	RPC rpc1, RPC rpc2, double &tx, double &ty);
	*/

	/*
	经纬度UTM转换，加入CLL_UTM Class中
	*/
	/*
	int LL2UTM(double Lat, double Lon, double &x, double &y, char n[10]);
	int LL2UTM_fixedZone(double Lat, double Lon, char n[10], double &x, double &y);
	int UTM2LL(double x, double y, char n[10], double &Lat, double &Lon);
	*/
	/////////////////////////////////////END/////////////////////////////////////

	/////////////////////////////////////Stereo_matching_operation/////////////////////////////////////
	/*int ComputeEdgeWeightForStereo();*/

	/*
	需要放入CImageFeatureMatching类里面
	int PairMatching(int pyrlevelID, char *img_path1, char *img_path2, RPC rpc1, RPC rpc2, TfwPara tfw1, TfwPara tfw2,
	float *fx1, float *fy1, int feaNum1,
	float *fx2, float *fy2, int feaNum2,
	vector<double> &x1, vector<double> &y1, vector<double> &x2, vector<double> &y2);

	int HarrisFeatureDetection(char *Pyr_img_list, char *Feature_pt_list);

	int Feature_matching_TopLevel(char *Pyr_img_list, char *Feature_pt_list, char **rpc_path, int satNum, int **stereolist, int *stereoNum,
	char *rectif_list_path,
	char *matching_pt_path, char *stereo_list_path);
	int GetFeaturePoints(char *pt_path, float *&x, float *&y, int &ptNum);

	int FindCorrespondenceBasedonFeature(
	int pyrLevelID,
	TfwPara tfw1, TfwPara tfw2,
	float tx1, float ty1, RPC rpc1, RPC rpc2,
	unsigned short int *img1, int w1, int h1,
	unsigned short int *img2, int w2, int h2,
	unsigned short int *Win1, unsigned short int *Win2,
	int *labels2, float *feax2, float *feay2,
	float &bestx, float &besty, double &ratio);

	int GetLWinData(unsigned short int *img, int w, int h, float cxf, float cyf, unsigned short int *LWin, int size);
	int GetRWinSearchArea(int pyrLevelID, TfwPara tfw1, TfwPara tfw2, RPC lrpc, RPC rrpc,
	float lx, float ly, int rw, int rh, double minH, double maxH, double stepH, int *feaLabels, float *feax, float *feay,
	vector<float> & rx_candi, vector<float> &ry_candi);
	int CheckPtinBuffer(int cx, int cy, vector<int>vx, vector<int>vy, double searchRange);
	int ComputeScoreBetweenWins(unsigned short int *win1, unsigned short int *win2, int winsize, double &score);
	int ComputeZNCC_score(unsigned short int *win1, unsigned short int *win2, int winsize, double &score);//zncc
	int ComputeHOG_score(unsigned short int *win1, unsigned short int *win2, int winsize, double &score);//HOG

	int Get_OriginalCor_From_RecifyCor(double x_rectif, double y_rectif, TfwPara tfw, int pyrsize, int LevelID, RPC rpc,
	double &x_ori, double &y_ori);

	int Get_ReftifyCor_From_OriginalCor(double orix, double oriy, double h, TfwPara tfw, RPC rpc, int pyrsize, int LevelID,
	double &x_rectif, double &y_rectif);

	int FindCommonPts_Multi_view(char *rectif_list_path, char *stereo_pt_list, int levelID, char **rpc_path, int imgNum, char *output_list);

	int FindConnectPt(CorrespondencePts pt, MchPtInfor *mchPairs, int pairNum, bool **ifprocessed, vector<CorrespondencePts> &pt_array);

	int ChechRepeatofPairID(int ID1, int ID2, vector<CorrespondencePts> pts, bool &ifRepeat);

	int ComputeGroundPtsFromMultiMchPts(vector<CorrespondencePts> pt_array, RPC *rpcs, TfwPara *tfws, int pyrLevelID,
	vector<CorrespondencePts> &new_pt_array, double &Lat, double &Lon, double &hei,
	MchPtInfor *mchPair, int pairNum, bool **ifprocessed_backup);

	int JudgeOutliersinForwardIntersection(vector<CorrespondencePts> pt_array, double *gx, double *gy, double *hei, int num,
	bool *ifinliers);
	int ComputeWeightedCenter(double *gX, double *gY, double *gZ, int num, double &gx_cen, double &gy_cen, double &gz_cen);
	int MedianFilter(double *arr, int size, double &midValue);
	int OutputTrackingPtsResults(FILE *fp, vector<CorrespondencePts> Pts, double Lat, double Lon, double Hei);

	int Multi_view_ForwardIntersection(vector<CorrespondencePts> pt_array, RPC *rpcs, TfwPara *tfws, int pyrLevelID,
	double &Lat, double &Lon, double &hei);
	*/
	/*
	放到imageProcessing类里面
	int CreatePyramidImage(char *rectif_list_path, int pyrsize, int pyrlevel, char *Pyramid_list_path);

	int CreatePyramid_onelevel(unsigned short int *OriginImg, int w, int h, int size,
	unsigned short int *PyImg, int pw, int ph);

	int img_16_to_8bits(unsigned short int *img_16, BYTE *img_8, int w, int h);
	*/
	/////////////////////////////////////END/////////////////////////////////////

	/////////////////////////////////////BundleAdjustment/////////////////////////////////////

	/*
	put in the RPC_fun class

	int Derivate_RPC_var(RPC para, BiasPara bias_para, double row, double col, double Lat, double Lon, double Hei,
	char *fun_option, char * var_option, double &result);

	int Compute_Taylor_Constant_Term(double aff_para1, double aff_para2, double aff_para3, RPC rpc_para, char *option,
	double row, double col, double Lat, double Lon, double Hei,
	double &constant_result);

	int ComputeNormalEquations(PtTrack *pts, int ptNum, RPC *rpc_paras, BiasPara *bias_para0, int imgNum,
	double *Lat0, double *Lon0, double *Hei0,
	double *&N_bias, double *&L_bias);

	int GetImgNumFromPtList(PtTrack *pts, int ptNum, int &imgNum);


	int BundleAdjustment_RPC_Bias(PtTrack *pts, int ptNum, RPC *rpc_paras, double thresh, BiasPara *&bias_para, int &bias_num);
	*/
	/////////////////////////////////////END/////////////////////////////////////
 	
};

