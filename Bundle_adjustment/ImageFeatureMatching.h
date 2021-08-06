#pragma once
typedef struct  
{
	char leftID[100], rightID[100];

	char left_aoi_img[512], right_aoi_img[512];
	char left_aoi_rpb[512], right_aoi_rpb[512];

	char left_rect_img[512], right_rect_img[512];
	char left_rect_tfw[512], right_rect_tfw[512];
	char left_rect_rpb[512], right_rect_rpb[512];

	char left_rect_feat[512], right_rect_feat[512];
	char left_aoi_feat[512], right_aoi_feat[512];

	char left_right_corrpendence_path[512];
}PairInfor;

typedef struct
{
	float x1, y1, x2, y2;
	char ID1[20], ID2[20];

}CorrespondencePts;

typedef struct
{
	char ID1[100], ID2[100]; //IDs of matching images
	int ptNum;    //number of correspondences
	CorrespondencePts *corresList; //coordinates of correspondences
}MchPtInfor;

typedef struct  
{
	char imgID[20];
	double x, y;
	double deltax, deltay;
}ImgPtInfo;

typedef struct
{
	double Lat, Lon, Hei;
	int ptNum;
	ImgPtInfo *img_pts;
}Ground_Img_Pt;
 
#include "RPC_Fun.h"
#include <vector>
using namespace std;

class CImageFeatureMatching
{
public:
	CImageFeatureMatching();
	~CImageFeatureMatching();

public:
	/*
	Feature detection
	*/
	int HarrisFeatureDetection(char *input_img_path, char *output_pt_path, int &feaNum);

	/*
	Feature matching between pair
	*/
	int Pair_Matching_Epi(PairInfor cur_pair, FILE *fp_list);
	int GetFeaturePoints(char *pt_path, float *&x, float *&y, int &ptNum);
	int PairMatching_core(char *aoi_img_path_left, char *aoi_img_path_right, char *rect_path_left, char *rect_path_right,
		RPC rpc_left, RPC rpc_right, TfwPara tfw_left, TfwPara tfw_right,
		float *fx_left, float *fy_left, int feaNum_left,
		float *fx_right, float *fy_right, int feaNum_right,
		vector<float> &x_left, vector<float> &y_left, vector<float> &x_right, vector<float> &y_right);

	int FindCorrespondenceBasedonFeature(
		TfwPara tfw1, TfwPara tfw2,
		float tx1, float ty1, RPC rpc1, RPC rpc2,
		unsigned short int *img1, int w1, int h1,
		unsigned short int *img2, int w2, int h2,
		unsigned short int *Win1, unsigned short int *Win2,
		double minHeight, double maxHeight,
		int *labels2, float *feax2, float *feay2,
		float &bestx, float &besty, double &ratio);

	int GetWinData(unsigned short int *img, int w, int h, float cxf, float cyf, unsigned short int *LWin, int size);
	int CheckPtinBuffer(int cx, int cy, vector<int>vx, vector<int>vy, double searchRange);

	int GetWinSearchArea(TfwPara tfw1, TfwPara tfw2, RPC lrpc, RPC rrpc, float lx, float ly, int rw, int rh,
		double minH, double maxH, double stepH, int *feaLabels, float *feax, float *feay,
		vector<float> & rx_candi, vector<float> &ry_candi);

	int ComputeScoreBetweenWins(unsigned short int *win1, unsigned short int *win2, int winsize, double &score);

	int ComputeZNCC_score(unsigned short int *win1, unsigned short int *win2, int winsize, double &score);//zncc
	int ComputeImprovedCensus_score(unsigned short int *win1, unsigned short int *win2, int winsize, double &score);//zncc
	int ComputeZNCC_score_census(unsigned short int *win1_ori, unsigned short int *win2_ori, int winsize, double &score);//zncc

	int ComputeHOG_score(unsigned short int *win1, unsigned short int *win2, int winsize, double &score);//HOG

	int Get_OriginalCor_From_RecifyCor(float x_rectif, float y_rectif, TfwPara tfw, RPC rpc,
		double &x_ori, double &y_ori);
	int Get_ReftifyCor_From_OriginalCor(double orix, double oriy, double h, TfwPara tfw, RPC rpc, 
		double &x_rectif, double &y_rectif);

	int FindCommonPts_Multi_view(PairInfor *Pairs, int pairNum, Long_Strip_Img *rect_imgs, int stripNum, char *output_list);

	int  DetectRepeatFeatures(char *all_feat_pt_list, Long_Strip_Img *rect_strips, int stripNum);

	int FindConnectPt(CorrespondencePts pt, MchPtInfor *mchPairs, int pairNum, bool **ifprocessed, vector<CorrespondencePts> &pt_array);

	int ChechRepeatofPairID(char *ID1, char *ID2, vector<CorrespondencePts> pts, bool &ifRepeat);

	int ComputeGroundPtsFromMultiMchPts(vector<CorrespondencePts> pt_array, Long_Strip_Img *rect_imgs, int stripNum,
		vector<CorrespondencePts> &new_pt_array, double &Lat, double &Lon, double &hei,
		MchPtInfor *mchPair, int pairNum, bool **ifprocessed_backup);

	int GetStripPatchID(char *ID, int &stripID, int &patchID);

	int JudgeOutliersinForwardIntersection(vector<CorrespondencePts> pt_array, double *gx, double *gy, double *hei, int num,
		bool *ifinliers);
	int ComputeWeightedCenter(double *gX, double *gY, double *gZ, int num, double &gx_cen, double &gy_cen, double &gz_cen);
	int MedianFilter(double *arr, int size, double &midValue);
	int GetWeightedCenter(double *arr, int size, double sigma, double &center);
	int OutputTrackingPtsResults(FILE *fp, vector<CorrespondencePts> Pts, Long_Strip_Img *strips, int stripNum,
		double Lat, double Lon, double Hei);

	int Multi_view_ForwardIntersection(vector<CorrespondencePts> pt_array, Long_Strip_Img *strips, int StripNum,
		double &Lat, double &Lon, double &hei);

	int JudgeOutliersinForwardIntersection_LL(vector<CorrespondencePts> pt_array, 
		double *lat, double *lon, double *hei, int num, bool *ifinliers);

	int Coarse_Reg(PairInfor *Pairs, int pairNum, Long_Strip_Img *strips_aoi, int stripNum, char *output_list);

	int Coarse_Reg(char *pair_mch_list, char *Multi_view_track_list, Long_Strip_Img *strips_aoi, int stripNum, char *Bundle_adj_list);

	int FindBestPatchIDforReg(Ground_Img_Pt *ground_img_pts, int ptNum, int cur_stripID, int *stripIDs, Long_Strip_Img *strips, 
		int stripNum, int regNum,
		double *&lat, double *&lon, double *&hei, double *&ix, double *&iy, int &gcp_num, int &patchID);

	//I have a new fun to replace it
	int PointTracking_InStrip(char *mch_pair_list_path, Long_Strip_Img *strips, int stripNum);

	//This is original func, maybe I did not need it in the new class
	int Feature_matching_TopLevel(char *Pyr_img_list, char *Feature_pt_list, char **rpc_path, int satNum, int **stereolist, int *stereoNum,
		char *rectif_list_path,
		char *matching_pt_path, char *stereo_list_path);

public:
	int m_tile_size;
	int m_mch_win_size;
	double m_avg_height;
	char m_zone[10];
	double m_min_height;
	double m_max_height;
	double m_stepH;
	int m_epi_search_range;
	int m_boundary_thresh;
	int m_thresh_inliers;
	char m_basis_ID[100];
};

