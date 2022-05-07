#pragma once
typedef struct
{
	double A, B, C, D, E, F;
}TfwPara;

typedef struct
{
	double lineOffset, sampOffset;
	double latOffset, longOffset, heightOffset;
	double lineScale, sampScale;
	double latScale, longScale, heightScale;

	double lineNumCoef[20];
	double lineDenCoef[20];
	double sampNumCoef[20];
	double sampDenCoef[20];

	char satID[100];
	char bandID[100];
	char SpecId[100];
	char BEGIN_GROUP[100];
	char errBias[100];
	char errRand[100];
}RPC;

typedef struct
{
	double drow;
	double dcol;
}BiasPara_constant;

#include "FileOperation.h"
#include "ImageFeatureMatching.h"
/*
P, L, H, r, c: normalzied coordiantes
P: Lat
L: Lon
H: height
r: row
c: column
*/

class CRPC_Fun
{
public:
	CRPC_Fun();
	~CRPC_Fun();

public:
	/*
	RPC read/write fun
	*/
	int ReadRpc_singleFile(char *rpcPath, RPC &rpcs);
	int ReadRpc_txt(char *rpcPath, RPC &rpcs);
	int ReadRpcInfo(char **rpcPath, int satNum, RPC *& rpcs);
	int WriteRPBFile(RPC rpc, char *outputPath);

	/*
	AOI_cut
	*/
	int Check_AOI_Overlap(char *imgPaths, char *rpcPaths, double *lat, double *lon, double *hei, int extra_size,
		int &min_px, int &min_py, int &max_px, int &max_py, bool &ifcover);
	int Check_Patch_Coverage_In_SameStrip(char *imgPaths_cur, char *rpbPath_cur, char *imgPaths_nxt, char *rpbPath_nxt);

	/*
	RPC projection
	*/
	int RPC_Ground2Image(double Lat, double Lon, double Height, RPC rpc, double &row, double &column);
	int RPC_Ground2Image_normalize(double P, double L, double H, RPC rpc, double &r, double &c);
	int RPC_Image2Ground_normalize(double r, double c, double H, RPC rpc, double &P, double &L);
	int RPC_Image2Ground(double row, double column, double Height, RPC rpc, double &lat, double &lon);
	int RPC_coefficient(double P, double L, double H, int ID, double &result);
	int RPC_coefficient_derivative(double P, double L, double H, int ID, char var, double &result);
	int RPC_ForwardIntersection(RPC rpc1, RPC rpc2, double ix1, double iy1, double ix2, double iy2, double &Lat, double &Lon, double &Hei);

	/*
	RPC update
	*/
	int RPC_update_From_AOI(char *original_rpc_path, char *new_rpc_path, int min_px, int min_py);

	/*
	compute GSD and height plane from RPC, the ground corner of images
	*/
	int RPC_compute_GSD_and_Height(char *img_path, char *rpc_path, double &GSD, double &height, char *zoneID);

	int Get_Average_HeightFromRPCs(RPC *rpcs, int num, double &avgH);

	int GetCornerPostions(int *imw, int *imh, double *height, RPC *rpcs, int satNum, double *&lats, double *&lons);

	int ComputeImageProjectionArea(double ux[4], double uy[4], double &S);

	int ComputeTriArea(double x[3], double y[3], double &S);

	int ComputeOverlapArea(double *ux, double *uy, double *S, int ID1, int ID2, double &overlapS);
	int IsPointInBuffer(double vx[4], double vy[4], double x, double y);
	int GetCross(double x1, double y1, double x2, double y2, double x, double y, double &crossresult);

	/*
	TFW read
	*/
	int ReadTfwFile(char *tfw_path, double &A, double &B, double &C, double &D, double &E, double &F);
	int ReadTfwFile(char *tfw_path, TfwPara &para);

	/*
	Patches sorting
	*/
	int ComputeTranslateBetweenTiles(char *imgPath1, char *rpbPath1,
		char *imgPath2, char *rpbPath2, int &tx, int &ty);

	int SortPatches(Long_Strip_Img strip);

	/*
	Bundle adjustment related funs
	*/
	int Derivate_RPC_var(RPC para, BiasPara_constant bias_para, double row, double col, double Lat, double Lon, double Hei,
		char *fun_option, char * var_option, double &result);

	int Compute_Taylor_Constant_Term(double aff_para1, double aff_para2, double aff_para3, RPC rpc_para, char *option,
		double row, double col, double Lat, double Lon, double Hei,
		double &constant_result);

	int ComputeNormalEquations_constant(Ground_Img_Pt *pts, int ptNum, RPC *rpc_paras, BiasPara_constant *bias_para0, int imgNum, char **imgIDs,
		double RMSE, double *&N_bias, double *&L_bias);

	int GetImgNumFromPtList(Ground_Img_Pt *pts, int ptNum, Long_Strip_Img *strips, int StripNum, int &imgNum, RPC *& rpcs, char **&imgIDs);

	int GetImgID(char **IDs, int num, char *cur_ID);

	int RPC_BundleAdjustment(Ground_Img_Pt *pts, int ptNum, Long_Strip_Img *strips, int StripNum, char *Bundle_list_path);

	int RelativeOrientation_bias_constant(double *lx, double *ly, double *rx, double *ry, int ptNum, RPC &rpc_left_ori, RPC &rpc_right_ori,
		bool *inliers);

	int RelativeOrientation_bias_constant(double *lx, double *ly, double *rx, double *ry, int ptNum, RPC &rpc_left_ori, RPC &rpc_right_ori,
		double &drow, double &dcol, double &error_reprojection);

	int RefineRPC_bias_constant(RPC &rpc, double row_bias, double col_bias);

	int Multi_view_Forward_intersection(double *x, double *y, RPC *rpcs, int ptNum, double &lat, double &lon, double &hei);

	int RPC_ComputeConstantBiasUsingGCP(double *ix, double *iy, double *lat, double *lon, double *hei, int ptNum, RPC rpc,
		double &drow, double &dcol, double &error_reprojection);

	int ComputeRPCForPlaneRectification(Long_Strip_Img *strips, Long_Strip_Img *&rect_imgs, int stripNum,
		int Virtual_GCP_Plane_Size, int Virtual_GCP_Height_Size, char *zone, double avgHeight);

	int ComputeRPCfromVirtualGCPs(double *ix, double *iy, double *lat, double *lon, double *hei, int ptNum,
		RPC &rpc, double &maxd, double &avgd);

public:
	double m_reprojection_thresh;
	int m_virtual_GCP_plan_size;
	int m_virtual_GCP_height_size;
	char m_basis_ID[100];

private:
	int AOI_Intersection(double minLat1, double minLon1, double maxLat1, double maxLon1,
		double minLat2, double minLon2, double maxLat2, double maxLon2, double *lat, double *lon);
};

