#include "stdafx.h"
extern "C"

{

#include "fast.h"

}

#include "ImageFeatureMatching.h"
#include "BundleAdjust.h"
#include "FileOperation.h"
#include "WuHrsMch.h"
#include "ImageProcess.h"
#include "OtherFuns.h"
#include "LL_UTM.h"
#include "RPC_Fun.h"

#define PI 3.14159265359

CImageFeatureMatching::CImageFeatureMatching()
{
	m_tile_size = 10000;
	m_mch_win_size = 15;
	m_avg_height = 0;
	strcpy(m_zone, "");

	m_min_height = 400;
	m_max_height = 8000;
	m_stepH = 50;

	m_epi_search_range = 20;

	m_boundary_thresh = 50;

	m_thresh_inliers = 2.0;

	strcpy(m_basis_ID, "0_0");
}


CImageFeatureMatching::~CImageFeatureMatching()
{
}

int CImageFeatureMatching::HarrisFeatureDetection(char *input_img_path, char *output_pt_path, int &feaNum)
{
	//class object define
	CFileOperation file;
	CImageProcess ipro;
	
	CWuHrsMch harris;
	FILE *fp_single = fopen(output_pt_path, "w");

	CImageBase img_read;
	img_read.Open(input_img_path);
	int w, h;
	w = img_read.GetCols();
	h = img_read.GetRows();

	unsigned short int *img_data = new unsigned short int[m_tile_size*m_tile_size];
	BYTE *img_data_8 = new BYTE[m_tile_size*m_tile_size];
	int startx, starty, endx, endy;
	startx = 0; starty = 0;
	endx = startx + m_tile_size - 1;
	endy = starty + m_tile_size - 1;
	if (endx > w - 1) endx = w - 1;
	if (endy > h - 1) endy = h - 1;

	feaNum = 0;
	while (endy < h)
	{
		while (endx < w)
		{
			//read block
			img_read.Read(img_data, startx, starty, endx - startx + 1, endy - starty + 1);

			//harris
			//16bit to 8 bit
			ipro.img_16_to_8bits(img_data, img_data_8, endx - startx + 1, endy - starty + 1);

			//Harris
// 			const FPT4D* harris_features;
// 			long int ptNum = 0;
// 			harris_features = harris.Hrs_ExtrFeat(&ptNum, img_data_8, endx - startx + 1, endy - starty + 1, 25, 25);
// 
// 			int t;
// 			for (t = 0; t < ptNum; t++)
// 			{
// 				float x_rect, y_rect;
// 				x_rect = (harris_features[t].xl + startx);
// 				y_rect = (harris_features[t].yl + starty);
// 				/*fprintf(fp_single, "%f %f %d %f %f\n", x_rect, y_rect, 0, x_rect, y_rect);*/
// 				fprintf(fp_single, "%f %f\n", x_rect, y_rect);
// 			}
// 
// 			harris.FreePtsMem();

			//FAST
			xy* FAST_features;
			int ptNum = 0;
			int threshold_fast = 40/*10*/; //adjust it to get the optimal value in the future
			FAST_features = fast9_detect_nonmax(img_data_8, endx - startx + 1, endy - starty + 1, endx - startx + 1, threshold_fast, &ptNum);

			for (int t = 0; t < ptNum; t++)
			{
				float x_rect, y_rect;
				x_rect = (FAST_features[t].x + startx);
				y_rect = (FAST_features[t].y + starty);
				/*fprintf(fp_single, "%f %f %d %f %f\n", x_rect, y_rect, 0, x_rect, y_rect);*/
				fprintf(fp_single, "%f %f\n", x_rect, y_rect);
			}

			free(FAST_features);

			//count total feature number
			feaNum += ptNum;

			if (endx == w - 1)
			{
				break;
			}
			else
			{
				startx = endx + 1;
				endx = startx + m_tile_size - 1;
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
			endx = startx + m_tile_size - 1;
			if (endx > w - 1) endx = w - 1;
			starty = endy + 1;
			endy = starty + m_tile_size - 1;
			if (endy > h - 1) endy = h - 1;
		}
	}

	//free memory
	img_read.Close();
	delete[]img_data_8; img_data_8 = NULL;
	delete[]img_data; img_data = NULL;
	fclose(fp_single); fp_single = NULL;

	return 1;
}

int CImageFeatureMatching::Pair_Matching_Epi(PairInfor cur_pair, FILE *fp_list)
{
	//define class objects
	CImageProcess ipro;
	CFileOperation file;
	CRPC_Fun rpcf;
	CLL_UTM geo_conv;

	//read geo_referenced files
	RPC rpc_left, rpc_right;
	TfwPara tfw_left, tfw_right;
	rpcf.ReadRpc_singleFile(cur_pair.left_rect_rpb, rpc_left);
	rpcf.ReadRpc_singleFile(cur_pair.right_rect_rpb, rpc_right);
	rpcf.ReadTfwFile(cur_pair.left_rect_tfw, tfw_left);
	rpcf.ReadTfwFile(cur_pair.right_rect_tfw, tfw_right);

	//read feature points of left and right images
	float *feax_left, *feay_left;
	float *feax_right, *feay_right;
	int ptNum_left = 0, ptNum_right = 0;
	GetFeaturePoints(cur_pair.left_rect_feat, feax_left, feay_left, ptNum_left);
	GetFeaturePoints(cur_pair.right_rect_feat, feax_right, feay_right, ptNum_right);
	
	printf("\n%d feature points in left image and %d feature points in right image\n", ptNum_left, ptNum_right);

	vector<float> x_left, y_left, x_right, y_right;
	PairMatching_core(cur_pair.left_aoi_img, cur_pair.right_aoi_img, cur_pair.left_rect_img, cur_pair.right_rect_img,
		rpc_left, rpc_right, tfw_left, tfw_right,
		feax_left, feay_left, ptNum_left, feax_right, feay_right, ptNum_right,
		x_left, y_left, x_right, y_right);

	//Original: Transform rectified points to original image points
	//I have realized rpc for plane retification, so I can directly output correspondences for 
	int mchNum = x_left.size();
	FILE *fp = fopen(cur_pair.left_right_corrpendence_path, "w");
	fprintf(fp, "%s\t%s\n", cur_pair.leftID, cur_pair.rightID);
	printf("Found %d correspondences\n", mchNum);
	fprintf(fp, "%d\n", mchNum);
	for (int i = 0; i < mchNum;i++)
	{
		fprintf(fp, "%f %f %f %f\n", x_left[i], y_left[i], x_right[i], y_right[i]);
	}
	fclose(fp); fp = NULL;

	fprintf(fp_list, "%s\n", cur_pair.left_right_corrpendence_path);

	//detect and remove outliers using constant bias correction strategy
	//the following file can be removed
	fp = fopen(cur_pair.left_right_corrpendence_path, "r");
	char ID_left[100], ID_right[100];
	fscanf(fp, "%s %s", ID_left, ID_right);
	int Num_correspondence = 0;
	fscanf(fp, "%d", &Num_correspondence);
	double *ix_left, *iy_left, *ix_right, *iy_right;
	ix_left = new double[Num_correspondence];
	iy_left = new double[Num_correspondence];
	ix_right = new double[Num_correspondence];
	iy_right = new double[Num_correspondence];
	
	for (int i = 0; i < Num_correspondence; i++)
	{
		fscanf(fp, "%lf %lf %lf %lf", &ix_left[i], &iy_left[i], &ix_right[i], &iy_right[i]);
	}
	fclose(fp); fp = NULL;

	//Relative orientation
	bool *inliers = new bool[Num_correspondence];
	rpcf.RelativeOrientation_bias_constant(ix_left, iy_left, ix_right, iy_right, Num_correspondence, rpc_left, rpc_right, inliers);
	int outlier_num = 0;
	for (int i = 0; i < Num_correspondence;i++)
	{
		if (inliers[i]==false)
		{
			outlier_num++;
		}
	}
	printf("Detect %d ourliers\n", outlier_num);

	//rewrite the mch file
	fp = fopen(cur_pair.left_right_corrpendence_path, "w");
	fprintf(fp, "%s\t%s\n", ID_left, ID_right);
	fprintf(fp, "%d\n", Num_correspondence - outlier_num);

	for (int i = 0; i < Num_correspondence;i++)
	{
		if (inliers[i] == 1)
		{
			fprintf(fp, "%lf %lf %lf %lf\n", ix_left[i], iy_left[i], ix_right[i], iy_right[i]);
		}
	}

	fclose(fp); fp = NULL;
	delete[]inliers; inliers = NULL;

	delete[]ix_left; ix_left = NULL;
	delete[]iy_left; iy_left = NULL;
	delete[]ix_right; ix_right = NULL;
	delete[]iy_right; iy_right = NULL;

	////////////////////////////////////////////////////////////////////////// check the coordinates of matching points
// 	char left_pt_path[255], right_pt_path[255];
// 	file.FindDirPath_prj(cur_pair.left_right_corrpendence_path, left_pt_path, false);
// 	strcat(left_pt_path, "\\");
// 	strcpy(right_pt_path, left_pt_path);
// 	strcat(left_pt_path, "left_point.txt");
// 	strcat(right_pt_path, "right_point.txt");
// 	fp = fopen(left_pt_path, "w");
// 	int w_left, h_left;
// 	int w_right, h_right;
// 	ipro.GetImageSize(cur_pair.left_rect_img, w_left, h_left);
// 	ipro.GetImageSize(cur_pair.right_rect_img, w_right, h_right);
// 	for (int i = 0; i < mchNum;i++)
// 	{
// 		fprintf(fp, "%d %f %f %f %f %f\n", i + 1, x_left[i], h_left - 1 - y_left[i], 0, x_left[i], h_left - 1 - y_left[i]);
// 	}
// 	fclose(fp); fp = NULL;
// 
// 	fp = fopen(right_pt_path, "w");
// 	for (int i = 0; i < mchNum; i++)
// 	{
// 		fprintf(fp, "%d %f %f %f %f %f\n", i + 1, x_right[i], h_right - 1 - y_right[i], 0, x_right[i], h_right - 1 - y_right[i]);
// 	}
// 	fclose(fp); fp = NULL;
	//////////////////////////////////////////////////////////////////////////

	//free memory
	delete[]feax_left; feax_left = NULL;
	delete[]feay_left; feay_left = NULL;
	x_left.clear();
	y_left.clear();
	x_right.clear();
	y_right.clear();
	vector<float>().swap(x_left);
	vector<float>().swap(y_left);
	vector<float>().swap(x_right);
	vector<float>().swap(y_right);

	return 1;
}

int CImageFeatureMatching::PairMatching_core(char *aoi_img_path_left, char *aoi_img_path_right, char *rect_path_left, char *rect_path_right,
	RPC rpc_left, RPC rpc_right, TfwPara tfw_left, TfwPara tfw_right,
	float *fx_left, float *fy_left, int feaNum_left,
	float *fx_right, float *fy_right, int feaNum_right,
	vector<float> &x_left, vector<float> &y_left, vector<float> &x_right, vector<float> &y_right)
{
	//Define some class objests
	CImageProcess ipro;

	//Read image information
	int w_left, h_left, w_right, h_right;
	int bands_left, bands_right;
	unsigned short int *img_data_left, *img_data_right;
	int w_ori_left, h_ori_left;
	int w_ori_right, h_ori_right;

	ipro.GetImageSize(aoi_img_path_left, w_ori_left, h_ori_left);
	ipro.GetImageSize(aoi_img_path_right, w_ori_right, h_ori_right);

	ipro.ReadImageData16(rect_path_left, img_data_left, w_left, h_left, bands_left);
	ipro.ReadImageData16(rect_path_right, img_data_right, w_right, h_right, bands_right);

	//Acquire height range from RPC files
	double minH_left, maxH_left;
	double minH_right, maxH_right;
	minH_left = rpc_left.heightOffset - rpc_left.heightScale;
	maxH_left = rpc_left.heightOffset + rpc_left.heightScale;

	minH_right = rpc_right.heightOffset - rpc_right.heightScale;
	maxH_right = rpc_right.heightOffset + rpc_right.heightScale;

	m_min_height = min(minH_left, minH_right);
	m_max_height = max(maxH_left, maxH_right);

	printf("Height range for epipolar constraints is: [%lf, %lf]\n", m_min_height, m_max_height);

	//project feature points on images
	int *labels_left, *labels_right;
	labels_left = new int[w_left*h_left];
	labels_right = new int[w_right*h_right];

	for (int i = 0; i < w_left*h_left;i++) 
	{ 
		labels_left[i] = -1;
	}
	for (int i = 0; i < w_right*h_right; i++) 
	{ 
		labels_right[i] = -1;
	}

	for (int i = 0; i < feaNum_left; i++)
	{
		int tx, ty;
		tx = (int)(fx_left[i] + 0.5);
		ty = (int)(fy_left[i] + 0.5);
		if (tx >= 0 && tx < w_left && ty >= 0 && ty < h_left)
			labels_left[ty*w_left + tx] = i;
	}

	for (int i = 0; i < feaNum_right; i++)
	{
		int tx, ty;
		tx = (int)(fx_right[i] + 0.5);
		ty = (int)(fy_right[i] + 0.5);
		if (tx >= 0 && tx < w_right && ty >= 0 && ty < h_right)
			labels_right[ty*w_right + tx] = i;
	}

	unsigned short int *Win_left, *Win_right;
	Win_left = new unsigned short int[m_mch_win_size*m_mch_win_size];
	Win_right = new unsigned short int[m_mch_win_size*m_mch_win_size];
	int halfsize = m_mch_win_size / 2;

	for (int i = 0; i < feaNum_left;i++)
	{
		//Get the data of win1
		memset(Win_left, 0, sizeof(unsigned short int)*m_mch_win_size*m_mch_win_size);

		float x_cur, y_cur;
		x_cur = fx_left[i];
		y_cur = fy_left[i];
		double x_ori, y_ori;
		Get_OriginalCor_From_RecifyCor(x_cur, y_cur, tfw_left, rpc_left, x_ori, y_ori);

		if (x_ori<m_boundary_thresh || x_ori>w_ori_left - m_boundary_thresh || y_ori<m_boundary_thresh || y_ori>h_ori_left-m_boundary_thresh)
		{
			continue; //In or near the black invalid regions
		}

		//Find correspondence
		float bestx_right = 0, besty_right = 0;
		double ratio2 = 1/*0*/;
		int error1 = 0;
		error1 = FindCorrespondenceBasedonFeature(tfw_left, tfw_right, fx_left[i], fy_left[i], rpc_left, rpc_right,
			img_data_left, w_left, h_left,
			img_data_right, w_right, h_right,
			Win_left, Win_right, 
			m_min_height,m_max_height,
			labels_right, fx_right, fy_right, 
			bestx_right, besty_right, ratio2);

		int error2 = 0;
		float bestx_left = 0, besty_left = 0;
		double ratio1 = 1;
		if (error1!=0 && ratio2< 0.6/*0.4*/) //constraint 1: the ratio between the best score and the second best score
		{
			error2 = FindCorrespondenceBasedonFeature(tfw_right, tfw_left, bestx_right, besty_right, rpc_right, rpc_left,
				img_data_right, w_right, h_right, 
				img_data_left, w_left, h_left,
				Win_right, Win_left, 
				m_min_height, m_max_height,
				labels_left, fx_left, fy_left,
				bestx_left, besty_left, ratio1);
		}

		//constraint 2: Left-right consistency check
		if (error2!=0)
		{
			float deltax, deltay;
			deltax = fabs(fx_left[i] - bestx_left);
			deltay = fabs(fy_left[i] - besty_left);
			if (deltax < 1 && deltay < 1)
			{
				x_left.push_back(fx_left[i]);
				y_left.push_back(fy_left[i]);
				x_right.push_back(bestx_right);
				y_right.push_back(besty_right);
			}
		}
	}

	//free memory
	delete[]img_data_left; img_data_left = NULL;
	delete[]img_data_right; img_data_right = NULL;
	delete[]labels_left; labels_left = NULL;
	delete[]labels_right; labels_right = NULL;
	delete[]Win_left; Win_left = NULL;
	delete[]Win_right; Win_right = NULL;

	return 1;
}

int CImageFeatureMatching::GetFeaturePoints(char *pt_path, float *&x, float *&y, int &ptNum)
{
	FILE *fp = fopen(pt_path, "r");
	while (!feof(fp))
	{
		char str[MaxLen];
		fgets(str, MaxLen, fp);
		ptNum++;
	}
	ptNum--;
	x = new float[ptNum];
	y = new float[ptNum];
	rewind(fp);
	int i;
	for (i = 0; i < ptNum;i++)
	{
		fscanf(fp, "%f %f", &x[i], &y[i]);
	}
	fclose(fp); fp = NULL;

	return 1;
}

int CImageFeatureMatching::FindCorrespondenceBasedonFeature(
	TfwPara tfw1, TfwPara tfw2,
	float tx1, float ty1, RPC rpc1, RPC rpc2,
	unsigned short int *img1, int w1, int h1,
	unsigned short int *img2, int w2, int h2,
	unsigned short int *Win1, unsigned short int *Win2,
	double minHeight, double maxHeight,
	int *labels2, float *feax2, float *feay2,
	float &bestx, float &besty, double &ratio)
{
	//initialize
	ratio = 1;

	GetWinData(img1, w1, h1, tx1, ty1, Win1, m_mch_win_size);

	double minH = minHeight;
	double maxH = maxHeight;
	double stepH = m_stepH;

	vector <float> x2_candi, y2_candi;
	int num_candi;
	int error = 0;
	error = GetWinSearchArea(tfw1, tfw2, rpc1, rpc2, tx1, ty1, w2, h2, minH, maxH, stepH, labels2, feax2, feay2,
		x2_candi, y2_candi);
	num_candi = x2_candi.size();

	if (num_candi < 1 || error == 0)
	{
		x2_candi.clear();
		y2_candi.clear();
		vector<float>().swap(x2_candi);
		vector<float>().swap(y2_candi);
		return 0;
	}

	//According to the candidate potentials, compute the correspondences of (tx1, ty1)
//	int x, y;
	int i;
	double *Scores = new double[num_candi];
	for (i = 0; i < num_candi; i++)
	{
		float tx2, ty2;
		tx2 = x2_candi[i];
		ty2 = y2_candi[i];

		GetWinData(img2, w2, h2, tx2, ty2, Win2, m_mch_win_size);

		//compute the score between win1 and win2
		double score = 0;
		ComputeScoreBetweenWins(Win1, Win2, m_mch_win_size, score);
		Scores[i] = score;
	}

	if (num_candi==1)
	{
		if (Scores[0]<0.05)
		{
			bestx = x2_candi[0];
			besty = y2_candi[0];

			ratio = 0.0;
			x2_candi.clear();
			y2_candi.clear();
			vector<float>().swap(x2_candi);
			vector<float>().swap(y2_candi);
			delete[]Scores; Scores = NULL;

			return 1;
		}
		else
		{
			ratio = 1.0;
			x2_candi.clear();
			y2_candi.clear();
			vector<float>().swap(x2_candi);
			vector<float>().swap(y2_candi);
			delete[]Scores; Scores = NULL;

			return 0;
		}
	}

	//compute best score
	double bestScore = 100;
	double secondBestScore = 100;
	for (i = 0; i < num_candi; i++)
	{
		if (Scores[i] < bestScore)
		{
			float tx2, ty2;
			tx2 = x2_candi[i];
			ty2 = y2_candi[i];

			bestScore = Scores[i];
			bestx = tx2;
			besty = ty2;
		}
	}

	//compute second best score
	for (i = 0; i < num_candi; i++)
	{
		float tx2, ty2;
		tx2 = x2_candi[i];
		ty2 = y2_candi[i];
		if (Scores[i] < secondBestScore && tx2 != bestx && ty2 != besty)
		{
			secondBestScore = Scores[i];
		}
	}

//	if (bestScore <= 0.05 && secondBestScore>0.2)
	{
		ratio = (bestScore + 0.0000001) / (secondBestScore + 0.0000001);
	}
// 	else
// 	{
// 		ratio = 1.0;
// 		x2_candi.clear();
// 		y2_candi.clear();
// 		vector<float>().swap(x2_candi);
// 		vector<float>().swap(y2_candi);
// 		delete[]Scores; Scores = NULL;
// 
// 		return 0;
// 	}

	//free memory
	x2_candi.clear();
	y2_candi.clear();
	vector<float>().swap(x2_candi);
	vector<float>().swap(y2_candi);
 	delete[]Scores; Scores = NULL;

	return 1;
}

int CImageFeatureMatching::GetWinData(unsigned short int *img, int w, int h, float cxf, float cyf, unsigned short int *LWin, int size)
{
	int cx, cy;
	cx = (int)(cxf + 0.5f);
	cy = (int)(cyf + 0.5f);
	memset(LWin, 0, sizeof(unsigned short int)*size*size);
	int halfsize = size / 2;
	int sx, ex, sy, ey;
	sx = cx - halfsize; ex = cx + halfsize;
	sy = cy - halfsize; ey = cy + halfsize;

	int tx, ty;
	for (ty = sy; ty <= ey; ty++)
		for (tx = sx; tx <= ex; tx++)
		{
			if (tx >= 0 && ty >= 0 && tx < w && ty < h)
			{
				LWin[(ty - sy)*size + (tx - sx)] = img[ty*w + tx];
			}
			else
				LWin[(ty - sy)*size + (tx - sx)] = 0;
		}

	return 1;
}

int CImageFeatureMatching::GetWinSearchArea(TfwPara tfw1, TfwPara tfw2, RPC lrpc, RPC rrpc, float lx, float ly, int rw, int rh,
	double minH, double maxH, double stepH, int *feaLabels, float *feax, float *feay,
	vector<float> & rx_candi, vector<float> &ry_candi)
{
	CRPC_Fun rpcf;
	int minx, miny, w_search, h_search;

	//compute the original image coordinate
	double orix, oriy;
//	Get_OriginalCor_From_RecifyCor(lx, ly, tfw1, lrpc, orix, oriy);
	orix = lx; oriy = ly;

	//Get the projection points of epipolar line in the second image
	double height;
	vector<double> x_proj, y_proj;
	for (height = minH; height <= maxH; height += stepH)
	{
		//For img1, image to ground
		double lat, lon;
		double row, column;
		row = oriy; column = orix;
		rpcf.RPC_Image2Ground(row, column, height, lrpc, lat, lon);

		//For img2, ground to image
		double rx, ry;
		rpcf.RPC_Ground2Image(lat, lon, height, rrpc, ry, rx);
		double x_rect, y_rect;
		//Get_ReftifyCor_From_OriginalCor(rx, ry, m_avg_height, tfw2, rrpc, x_rect, y_rect);
		x_rect = rx; 
		y_rect = ry;

		if (x_rect >= 0 && x_rect < rw - 0.5f && y_rect >= 0 && y_rect < rh - 0.5f)
		{
			x_proj.push_back(x_rect);
			y_proj.push_back(y_rect);
		}
	}

	//compute the search area
	if (x_proj.size() < 2)
	{
		x_proj.clear();
		y_proj.clear();
		vector<double>().swap(x_proj);
		vector<double>().swap(y_proj);
		return 0;
	}

	int maxx, maxy;
	minx = (int)x_proj[0] - 1; miny = (int)y_proj[0] - 1;
	maxx = (int)x_proj[0] + 1; maxy = (int)y_proj[0] + 1;
	double searchRange = m_epi_search_range;
	int i;
	int num = x_proj.size();
	for (i = 1; i < num; i++)
	{
		if (minx > x_proj[i]) minx = (int)x_proj[i] - 1;
		if (maxx < x_proj[i]) maxx = (int)x_proj[i] + 1;

		if (miny > y_proj[i]) miny = (int)y_proj[i] - 1;
		if (maxy < y_proj[i]) maxy = (int)y_proj[i] + 1;
	}
	minx -= (int)(searchRange + 0.5); miny -= (int)(searchRange + 0.5);
	maxx += (int)(searchRange + 0.5); maxy += (int)(searchRange + 0.5);
	if (minx < 0) minx = 0;
	if (miny < 0) miny = 0;
	if (maxx > rw - 1) maxx = rw - 1;
	if (maxy > rh - 1) maxy = rh - 1;
	w_search = maxx - minx + 1;
	h_search = maxy - miny + 1;

	int x, y;
	for (y = 0; y < h_search; y++)
		for (x = 0; x < w_search; x++)
		{
			int tx, ty;
			tx = x + minx;
			ty = y + miny;

			if (feaLabels[ty*rw + tx] >-1) //contains a feature point
			{
				int ID = feaLabels[ty*rw + tx];

				rx_candi.push_back(feax[ID]);
				ry_candi.push_back(feay[ID]);
			}
		}

	//free memory
	x_proj.clear();
	y_proj.clear();
	vector<double>().swap(x_proj);
 	vector<double>().swap(y_proj);

	return 1;
}

int CImageFeatureMatching::ComputeScoreBetweenWins(unsigned short int *win1, unsigned short int *win2, int winsize, double &score)
{
	int x, y;
	int cx, cy;
	cx = winsize / 2; cy = winsize / 2;

	//improved Census
// 	double score_ipCensus;
// 	ComputeImprovedCensus_score(win1, win2, winsize, score_ipCensus);

	//compute Census, Zncc, HOG
	//Census
// 	double score_census = 0;
// 	int *win_census1, *win_census2;
// 	win_census1 = new int[winsize*winsize];
// 	win_census2 = new int[winsize*winsize];
// 	memset(win_census1, 0, sizeof(int)*winsize*winsize);
// 	memset(win_census2, 0, sizeof(int)*winsize*winsize);
// 
// 	for (y = 0; y < winsize; y++)
// 		for (x = 0; x < winsize;x++)
// 		{
// 			if (win1[y*winsize+x]>win1[cy*winsize+cx])
// 			{
// 				win_census1[y*winsize + x] = 1;
// 			}
// 			if (win2[y*winsize + x]>win2[cy*winsize + cx])
// 			{
// 				win_census2[y*winsize + x] = 1;
// 			}
// 		}
// 
// 	for (y = 0; y < winsize; y++)
// 		for (x = 0; x < winsize;x++)
// 		{
// 			score_census += abs(win_census1[y*winsize + x] - win_census2[y*winsize + x]);
// 		}
// 
// 	double MaxValue_census = /*0.5**/winsize*winsize;
// // 	if (score_census>0.5*MaxValue_census)
// // 	{
// // 		score_census = MaxValue_census;
// // 	}
// 	score_census /= MaxValue_census;

	//ZNCC
	double score_zncc = 0;
	ComputeZNCC_score(win1, win2, winsize, score_zncc);
	//ComputeZNCC_score_census(win1, win2, winsize, score_zncc);
	score_zncc = 1 - score_zncc;
	 
	//HOG
// 	double score_hog = 0;
// 	ComputeHOG_score(win1, win2, winsize, score_hog);

	//compute the final score
	score = score_zncc/*score_ipCensus*/;

	//free memory
// 	delete[]win_census1; win_census1 = NULL;
// 	delete[]win_census2; win_census2 = NULL;
	return 1;
}

int CImageFeatureMatching::ComputeHOG_score(unsigned short int *win1, unsigned short int *win2, int winsize, double &score)
{
	//sobel operatior
	float Template_Gx[9] = { -0.25f, 0, 0.25f,
		-0.5f, 0, 0.5f,
		-0.25f, 0, 0.25f };//sobel 算子模板
	float Template_Gy[9] = { -0.25f, -0.5f, -0.25f,
		0, 0, 0,
		0.25f, 0.5f, 0.25f };//sobel 算子模板

	//Gradient patch for win1 and win2
	double *Gx1, *Gy1;
	Gx1 = new double[winsize*winsize];
	Gy1 = new double[winsize*winsize];
	memset(Gx1, 0, sizeof(double)*winsize*winsize);
	memset(Gy1, 0, sizeof(double)*winsize*winsize);

	double *Gx2, *Gy2;
	Gx2 = new double[winsize*winsize];
	Gy2 = new double[winsize*winsize];
	memset(Gx2, 0, sizeof(double)*winsize*winsize);
	memset(Gy2, 0, sizeof(double)*winsize*winsize);

	double Array1[9], Array2[9];
	int x, y;
	int sx, sy, ex, ey;
	int tx, ty;
	int I1 = 0;
	for (y = 1; y < winsize - 1; y++)
		for (x = 1; x < winsize -1;x++)
		{
			sx = x - 1;
			ex = x + 1;
			sy = y - 1;
			ey = y + 1;

			I1 = 0; // a pixel in local 3x3 win
			for (ty = sy; ty <= ey; ty++)
				for (tx = sx; tx <= ex; tx++)
				{
					Array1[I1] = win1[ty*winsize + tx];
					Array2[I1] = win2[ty*winsize + tx];
					I1++;
				}

			I1 = 0; // a pixel in local 3x3 win
			for (I1 = 0; I1 < 9; I1++)
			{
				Gx1[y*winsize + x] += Template_Gx[I1] * Array1[I1];
				Gy1[y*winsize + x] += Template_Gy[I1] * Array1[I1];

				Gx2[y*winsize + x] += Template_Gx[I1] * Array2[I1];
				Gy2[y*winsize + x] += Template_Gy[I1] * Array2[I1];
			}
		}

	//2. gradient orientation patch
	double *GD1, *GD2;
	GD1 = new double[winsize*winsize];
	GD2 = new double[winsize*winsize];
	memset(GD1, 0, sizeof(double)*winsize*winsize);
	memset(GD2, 0, sizeof(double)*winsize*winsize);
	int I2 = 0; //a pixel in window
	for (I2 = 0; I2 < winsize*winsize; I2++)
	{
		float tGx, tGy;

		//win1
		tGx = (float)Gx1[I2];
		tGy = (float)Gy1[I2];
		if (tGx == 0 && tGy == 0)
			GD1[I2] = 0;
		else if (tGy == 0 && tGx > 0)
			GD1[I2] = 0;
		else if (tGy == 0 && tGx < 0)
			GD1[I2] = -180;
		else
			GD1[I2] = (float)(atan2(tGy, tGx) / PI * 180);

		GD1[I2] += 180;

		if (GD1[I2] >= 360) GD1[I2] = 0;
		if (GD1[I2] < 0)    GD1[I2] = 359;

		//win2
		tGx = (float)Gx2[I2];
		tGy = (float)Gy2[I2];
		if (tGx == 0 && tGy == 0)
			GD2[I2] = 0;
		else if (tGy == 0 && tGx > 0)
			GD2[I2] = 0;
		else if (tGy == 0 && tGx < 0)
			GD2[I2] = -180;
		else
			GD2[I2] = (float)(atan2(tGy, tGx) / PI * 180);

		GD2[I2] += 180;

		if (GD2[I2] >= 360) GD2[I2] = 0;
		if (GD2[I2] < 0)    GD2[I2] = 359;
	}

	//HOG generation
	int Bin1[12], Bin2[12];
	int binID1, binID2;
	memset(Bin1, 0, sizeof(int) * 12);
	memset(Bin2, 0, sizeof(int) * 12);
	for (y = 1; y < winsize - 1; y++)
		for (x = 1; x < winsize - 1; x++)
		{
			binID1 = (int)(GD1[y*winsize + x] / 30);
			Bin1[binID1] ++;
			binID2 = (int)(GD2[y*winsize + x] / 30);
			Bin2[binID2] ++;
		}

	score = 0;
	int i = 0;
	for (i = 0; i < 12;i++)
	{
		score += abs(Bin1[i] - Bin2[i]);
	}

	score /= (winsize - 2)*(winsize - 2);

	if (score>1)
	{
		score = 1;
	}

	//free memory
	delete[]Gx1; Gx1 = NULL;
	delete[]Gy1; Gy1 = NULL;
	delete[]Gx2; Gx2 = NULL;
	delete[]Gy2; Gy2 = NULL;
	delete[]GD1; GD1 = NULL;
	delete[]GD2; GD2 = NULL;
	return 1;
}

int CImageFeatureMatching::ComputeZNCC_score_census(unsigned short int *win1_ori, unsigned short int *win2_ori, int winsize, double &score)
{
	unsigned short int *win1 = new unsigned short int[winsize*winsize];
	unsigned short int *win2 = new unsigned short int[winsize*winsize];
	memset(win1, 0, sizeof(unsigned short int)*winsize*winsize);
	memset(win2, 0, sizeof(unsigned short int)*winsize*winsize);

	int cy, cx;
	cy = winsize / 2; cx = winsize / 2;
	for (int y = 0; y < winsize; y++)
		for (int x = 0; x < winsize;x++)
		{
			if (win1_ori[y*winsize+x]<win1_ori[cy*winsize+cx])
			{
				win1[y*winsize + x] = 1;
			}
		}

	for (int y = 0; y < winsize; y++)
		for (int x = 0; x < winsize; x++)
		{
			if (win2_ori[y*winsize + x] < win2_ori[cy*winsize + cx])
			{
				win2[y*winsize + x] = 1;
			}
		}

	score = 0;
	for (int y = 0; y < winsize; y++)
		for (int x = 0; x < winsize; x++)
		{
			score += abs(win1[y*winsize + x] - win2[y*winsize + x]);
		}

	score /= (winsize*winsize);
	score = 1 - score;

// 	double coef;
// 	double avg_base = 0, avg_ref = 0;
// 	int i;
// 	unsigned short int *baseTemp = win1;
// 	unsigned short int *refTemp = win2;
// 	for (i = 0; i < winsize*winsize; i++)
// 	{
// 		avg_base += baseTemp[i];
// 		avg_ref += refTemp[i];
// 	}
// 
// 	avg_base /= (winsize*winsize);
// 	avg_ref /= (winsize*winsize);
// 
// 	double cor = 0;
// 	for (i = 0; i < winsize*winsize; i++)
// 	{
// 		cor += (baseTemp[i] - avg_base)*(refTemp[i] - avg_ref);
// 	}
// 
// 	double cor_base = 0, cor_ref = 0;
// 	for (i = 0; i < winsize*winsize; i++)
// 	{
// 		cor_base += (baseTemp[i] - avg_base)*(baseTemp[i] - avg_base);
// 		cor_ref += (refTemp[i] - avg_ref)*(refTemp[i] - avg_ref);
// 	}
// 
// 	if (cor_base == 0 && cor_ref == 0)
// 		coef = 1;
// 	else if (cor_base == 0 && cor_ref != 0)
// 		coef = 0;
// 	else if (cor_base != 0 && cor_ref == 0)
// 		coef = 0;
// 	else
// 	{
// 		cor_base = sqrt(cor_base);
// 		cor_ref = sqrt(cor_ref);
// 
// 		coef = cor / (cor_ref*cor_base);
// 	}
// 
// 	coef = fabs(coef);
// 
// 	score = coef;

	delete[]win1; win1 = NULL;
	delete[]win2; win2 = NULL;

	return 1;
}

int CImageFeatureMatching::ComputeImprovedCensus_score(unsigned short int *win1, unsigned short int *win2, int winsize, double &score)
{
	double Gaussian_Template[5 * 5] = {
	1,  4,  7,  4, 1,
	4, 16, 26, 16, 4,
	7, 26, 41, 26, 7,
	4, 16, 26, 16, 4,
	1, 4,   7,  4, 1
	};
	for (int i = 0; i < 25;i++)
	{
		Gaussian_Template[i] /= 273;
	}

	unsigned short int *win_left, *win_right;
	win_left = new unsigned short int[winsize*winsize];
	win_right = new unsigned short int[winsize*winsize];
	memcpy(win_left, win1, sizeof(unsigned short int)*winsize*winsize);
	memcpy(win_right, win2, sizeof(unsigned short int)*winsize*winsize);

	int Gaussian_filter_szie = 5;
	int halfSize = Gaussian_filter_szie / 2;
	unsigned short int local_win[5 * 5];

	//Gaussian filtering for left images
	for (int y = halfSize; y < winsize - halfSize; y++)
		for (int x = halfSize; x < winsize - halfSize;x++)
		{
			int cx = x, cy = y;
			int sx, sy, ex, ey;
			sx = x - halfSize; ex = x + halfSize;
			sy = y - halfSize; ey = y + halfSize;

			for (int ty = sy; ty <= ey; ty++)
				for (int tx = sx; tx <= ex; tx++)
				{
					local_win[(ty - sy) * 5 + (tx - sx)] = win1[ty*winsize + tx];
				}

			double filtering_result = 0;
			for (int ty = 0; ty < 5; ty++)
				for (int tx = 0; tx < 5;tx++)
				{
					filtering_result += local_win[ty * 5 + tx] * Gaussian_Template[ty * 5 + tx];
				}

			win_left[y*winsize + x] = (unsigned short int)(filtering_result + 0.5);
		}

	//Gaussian filtering for right images
	for (int y = halfSize; y < winsize - halfSize; y++)
		for (int x = halfSize; x < winsize - halfSize; x++)
		{
			int cx = x, cy = y;
			int sx, sy, ex, ey;
			sx = x - halfSize; ex = x + halfSize;
			sy = y - halfSize; ey = y + halfSize;

			for (int ty = sy; ty <= ey; ty++)
				for (int tx = sx; tx <= ex; tx++)
				{
					local_win[(ty - sy) * 5 + (tx - sx)] = win2[ty*winsize + tx];
				}

			double filtering_result = 0;
			for (int ty = 0; ty < 5; ty++)
				for (int tx = 0; tx < 5; tx++)
				{
					filtering_result += local_win[ty * 5 + tx] * Gaussian_Template[ty * 5 + tx];
				}

			win_right[y*winsize + x] = (unsigned short int)(filtering_result + 0.5);
		}

	//Block processing of Census // Later, I will fix the stepx and stepy as 3
	double diff = 0;
	int block_interval = winsize / 3;
	int stepx = winsize / block_interval;
	int stepy = winsize / block_interval;

	for (int i = 0; i < block_interval; i++)
		for (int j = 0; j < block_interval; j++)
		{
			int sx, sy, ex, ey;
			sx = j*stepx; 
			sy = i*stepy;
			if (i == block_interval - 1) {
				ey = winsize;
			}
			else
				ey = (i + 1)*stepy;

			if (j == block_interval - 1) {
				ex = winsize;
			}
			else
				ex = (j + 1)*stepx;

			//compute the score for the sub-block
			int cx, cy;
			int center_intensity_left, center_intensity_right;
			cx = (sx + ex - 1) / 2;
			cy = (sy + ey - 1) / 2;

			center_intensity_left = win_left[cy*winsize + cx];
			center_intensity_right = win_right[cy*winsize + cx];

			for (int tx = sx; tx < ex; tx++)
				for (int ty = sy; ty < ey;ty++)
				{
					int bit_left, bit_right;
					if (win_left[ty*winsize + tx]>center_intensity_left)
						bit_left = 0;
					else
						bit_left = 1;

					if (win_right[ty*winsize + tx]>center_intensity_right)
						bit_right = 0;
					else
						bit_right = 1;

					diff += abs(bit_left - bit_right);
				}
		}

	score = diff / (winsize*winsize);

	//free memory
	delete[]win_left; win_left = NULL;
	delete[]win_right; win_right = NULL;
	return 1;
}

int CImageFeatureMatching::ComputeZNCC_score(unsigned short int *win1, unsigned short int *win2, int winsize, double &score)
{
	double coef;
	double avg_base = 0, avg_ref = 0;
	int i;
	unsigned short int *baseTemp = win1;
	unsigned short int *refTemp = win2;
	for (i = 0; i < winsize*winsize; i++)
	{
		avg_base += baseTemp[i];
		avg_ref += refTemp[i];
	}

	avg_base /= (winsize*winsize);
	avg_ref /= (winsize*winsize);

	double cor = 0;
	for (i = 0; i < winsize*winsize; i++)
	{
		cor += (baseTemp[i] - avg_base)*(refTemp[i] - avg_ref);
	}

	double cor_base = 0, cor_ref = 0;
	for (i = 0; i < winsize*winsize; i++)
	{
		cor_base += (baseTemp[i] - avg_base)*(baseTemp[i] - avg_base);
		cor_ref += (refTemp[i] - avg_ref)*(refTemp[i] - avg_ref);
	}

	if (cor_base == 0 && cor_ref == 0)
		coef = 1;
	else if (cor_base == 0 && cor_ref != 0)
		coef = 0;
	else if (cor_base != 0 && cor_ref == 0)
		coef = 0;
	else
	{
		cor_base = sqrt(cor_base);
		cor_ref = sqrt(cor_ref);

		coef = cor / (cor_ref*cor_base);
	}

	coef = fabs(coef);

	score = coef;
	return 1;
}

int CImageFeatureMatching::CheckPtinBuffer(int cx, int cy, vector<int>vx, vector<int>vy, double searchRange)
{
	CRPC_Fun rpcf;
	int num = vx.size();
	int i = 0;
	double dis1, dis2;
	dis1 = sqrt((cx - vx[i])*(cx - vx[i]) + (cy - vy[i])*(cy - vy[i]));
	dis2 = sqrt((cx - vx[i + 1])*(cx - vx[i + 1]) + (cy - vy[i + 1])*(cy - vy[i + 1]));
	double mind = (dis1 + dis2) / 2;
	int bestID = 0;
	for (i = 1; i < num - 1; i++)
	{
		dis1 = sqrt((cx - vx[i])*(cx - vx[i]) + (cy - vy[i])*(cy - vy[i]));
		dis2 = sqrt((cx - vx[i + 1])*(cx - vx[i + 1]) + (cy - vy[i + 1])*(cy - vy[i + 1]));
		double avgd = (dis1 + dis2) / 2;
		if (mind>avgd)
		{
			mind = avgd;
			bestID = i;
		}
	}

	//compute dis between point and line
	double x[3], y[3];
	x[0] = cx; y[0] = cy;
	x[1] = vx[bestID]; y[1] = vy[bestID];
	x[2] = vx[bestID + 1]; y[2] = vy[bestID + 1];

	double S;
	rpcf.ComputeTriArea(x, y, S);

	double dis_best;
	dis_best = sqrt((vx[bestID] - vx[bestID + 1])*(vx[bestID] - vx[bestID + 1]) + (vy[bestID] - vy[bestID + 1])*(vy[bestID] - vy[bestID + 1]));

	double dis;
	dis = 2 * S / dis_best;

	if (dis <= searchRange)
	{
		return 1;
	}
	else
		return 0;

	return 1;
}

int CImageFeatureMatching::Get_OriginalCor_From_RecifyCor(float x_rectif, float y_rectif, TfwPara tfw,RPC rpc,
	double &x_ori, double &y_ori)
{
	CLL_UTM geo_conv;
	CRPC_Fun rpcf;

	double gX, gY, gZ;
	gZ = m_avg_height;
	gX = tfw.A*x_rectif + tfw.B*y_rectif + tfw.C;
	gY = tfw.D*x_rectif + tfw.E*y_rectif + tfw.F;

	double lat, lon;
	geo_conv.UTM2LL(gX, gY, m_zone, lat, lon);

	double row, column;
	rpcf.RPC_Ground2Image(lat, lon, gZ, rpc, row, column);

	y_ori = row;
	x_ori = column;

	return 1;
}

int CImageFeatureMatching::Get_ReftifyCor_From_OriginalCor(double orix, double oriy, double h, TfwPara tfw, RPC rpc, 
	double &x_rectif, double &y_rectif)
{
	CLL_UTM geo_conv;
	CRPC_Fun rpcf;

	double Lat, Lon;
	rpcf.RPC_Image2Ground(oriy, orix, h, rpc, Lat, Lon);

	double gX, gY;
	geo_conv.LL2UTM_fixedZone(Lat, Lon, m_zone, gX, gY);

	double x_r, y_r;
	x_r = (gX - tfw.C) / tfw.A;
	y_r = (gY - tfw.F) / tfw.E;

	x_rectif = x_r;
	y_rectif = y_r;

	return 1;
}

int  CImageFeatureMatching::DetectRepeatFeatures(char *all_feat_pt_list, Long_Strip_Img *rect_strips, int stripNum)
{
	CFileOperation file;
	CRPC_Fun rpcf;
	CImageProcess ipro;

	FILE *fp = fopen(all_feat_pt_list,"r");
	int ImgNum = 0;
	fscanf(fp, "%d", &ImgNum);

	char cur_ID[100];
	char cur_fea_path[MaxLen];
	fscanf(fp, "%s %s", cur_ID, cur_fea_path);
	int repeatNum = 0;
	for (int i = 1; i < ImgNum;i++)
	{
		char nxt_ID[100];
		char nxt_fea_path[MaxLen];
		fscanf(fp, "%s %s", nxt_ID, nxt_fea_path);

		int cur_stripID, cur_patchID;
		int nxt_stripID, nxt_patchID;
		sscanf(cur_ID, "%d_%d", &cur_stripID, &cur_patchID);
		sscanf(nxt_ID, "%d_%d", &nxt_stripID, &nxt_patchID);

		if (cur_stripID==nxt_stripID)
		{
			TfwPara cur_para, nxt_para;
			rpcf.ReadTfwFile(rect_strips[cur_stripID].rpb_paths[cur_patchID], cur_para);
			rpcf.ReadTfwFile(rect_strips[nxt_stripID].rpb_paths[nxt_patchID], nxt_para);

			float *fx_cur, *fy_cur;
			float *fx_nxt, *fy_nxt;
			int feaNum_cur = 0, feaNum_nxt = 0;
			file.GetLineNuminFiles(cur_fea_path, feaNum_cur);
			file.GetLineNuminFiles(nxt_fea_path, feaNum_nxt);

			fx_cur = new float[feaNum_cur];
			fy_cur = new float[feaNum_cur];
			fx_nxt = new float[feaNum_nxt];
			fy_nxt = new float[feaNum_nxt];

			FILE *fp_cur = fopen(cur_fea_path, "r");
			for (int j = 0; j < feaNum_cur;j++)
			{
				fscanf(fp_cur, "%f %f", &fx_cur[j], &fy_cur[j]);
			}
			fclose(fp_cur); fp_cur = NULL;

			FILE *fp_nxt = fopen(nxt_fea_path, "r");
			for (int j = 0; j < feaNum_nxt; j++)
			{
				fscanf(fp_nxt, "%f %f", &fx_nxt[j], &fy_nxt[j]);
			}
			fclose(fp_nxt); fp_nxt = NULL;

			fp_cur = fopen(cur_fea_path, "w");
			int w_nxt, h_nxt;
			ipro.GetImageSize(rect_strips[nxt_stripID].img_paths[nxt_patchID], w_nxt, h_nxt);

			for (int j = 0; j < feaNum_cur; j++)
			{
				double gX, gY, gZ;
				gX = cur_para.A*fx_cur[j] + cur_para.B*fy_cur[j] + cur_para.C;
				gY = cur_para.D*fx_cur[j] + cur_para.E*fy_cur[j] + cur_para.F;
				gZ = m_avg_height;

				double x_nxt, y_nxt;
				x_nxt = (nxt_para.B*nxt_para.F - nxt_para.B*gY - nxt_para.C*nxt_para.E + nxt_para.E*gX) / (nxt_para.A*nxt_para.E - nxt_para.B*nxt_para.D);
				y_nxt = -(nxt_para.A*nxt_para.F - nxt_para.A*gY - nxt_para.C*nxt_para.D + nxt_para.D*gX) / (nxt_para.A*nxt_para.E - nxt_para.B*nxt_para.D);

				bool ifrepeat = false;
				if (x_nxt>=0 && x_nxt<w_nxt - 1 && y_nxt>=0 && y_nxt<h_nxt - 1)
				{
					for (int k = 0; k < feaNum_nxt;k++)
					{
						if (fabs(x_nxt - fx_nxt[k])<1 && fabs(y_nxt - fy_nxt[k])<1)
						{
							ifrepeat = true;
							break;
						}
					}
				}

				if (ifrepeat == false)
				{
					fprintf(fp_cur, "%f %f", fx_cur[j], fy_cur[j]);
				}
				else
					repeatNum++;
			}
			fclose(fp_cur); fp_cur = NULL;

			//free memory
			delete[]fx_cur; fx_cur = NULL;
			delete[]fy_cur; fy_cur = NULL;
			delete[]fx_nxt; fx_nxt = NULL;
			delete[]fy_nxt; fy_nxt = NULL;
		}

		strcpy(cur_ID, nxt_ID);
		strcpy(cur_fea_path, nxt_fea_path);
	}

	fclose(fp); fp = NULL;

	printf("Totally find %d repeated points\n", repeatNum);

	return 1;
}

int CImageFeatureMatching::PointTracking_InStrip(char *mch_pair_list_path, Long_Strip_Img *strips, int stripNum)
{
	//define obj class
	CRPC_Fun rpcf;
	CImageProcess ipro;
	CLL_UTM geo_conv;

	FILE *fp = fopen(mch_pair_list_path,"r");
	int pairNum = 0;
	fscanf(fp, "%d\n", &pairNum);
	char **pair_correspondence_path;
	pair_correspondence_path = new char *[pairNum];
	for (int i = 0; i < pairNum;i++)
	{
		pair_correspondence_path[i] = new char[MaxLen];
	}

	for (int i = 0; i < pairNum; i++)
	{
		fscanf(fp, "%s", pair_correspondence_path[i]);
	}
	fclose(fp); fp = NULL;

// 	printf("%d\n", pairNum);
// 	for (int i = 0; i < pairNum; i++)
// 	{
// 		printf("%s\n", pair_correspondence_path[i]);
// 	}

 	MchPtInfor *pairs = new MchPtInfor[pairNum];
	for (int i = 0; i < pairNum; i++)
	{
		fp = fopen(pair_correspondence_path[i], "r");

		fscanf(fp, "%s %s", pairs[i].ID1, pairs[i].ID2);
		fscanf(fp, "%d", &pairs[i].ptNum);
		
		pairs[i].corresList = new CorrespondencePts[pairs[i].ptNum];
		for (int j = 0; j < pairs[i].ptNum;j++)
		{
			fscanf(fp, "%f %f %f %f", &pairs[i].corresList[j].x1, &pairs[i].corresList[j].y1, &pairs[i].corresList[j].x2, &pairs[i].corresList[j].y2);
		}

		fclose(fp); fp = NULL;
	}

	bool **ifrepeat = new bool *[pairNum];
	for (int i = 0; i < pairNum;i++)
	{
		ifrepeat[i] = new bool[pairs[i].ptNum];
		memset(ifrepeat[i], 0, sizeof(bool)*pairs[i].ptNum);
	}

	for (int i = 0; i < pairNum - 1;i++)
	{
		for (int j = i + 1; j < pairNum;j++)
		{
			int stripID_i_1, stripID_i_2, stripID_j_1, stripID_j_2;
			int patchID_i_1, patchID_i_2, patchID_j_1, patchID_j_2;
			sscanf(pairs[i].ID1, "%d_%d", &stripID_i_1, &patchID_i_1);
			sscanf(pairs[i].ID2, "%d_%d", &stripID_i_2, &patchID_i_2);
			sscanf(pairs[j].ID1, "%d_%d", &stripID_j_1, &patchID_j_1);
			sscanf(pairs[j].ID2, "%d_%d", &stripID_j_2, &patchID_j_2);

			if (stripID_i_1 == stripID_j_1 && stripID_i_2 == stripID_j_2) //Same strip matching
			{
				RPC rpc_i_1, rpc_i_2, rpc_j_1, rpc_j_2;
				rpcf.ReadRpc_singleFile(strips[stripID_i_1].rpb_paths[patchID_i_1], rpc_i_1);
				rpcf.ReadRpc_singleFile(strips[stripID_i_2].rpb_paths[patchID_i_2], rpc_i_2);
				rpcf.ReadRpc_singleFile(strips[stripID_j_1].rpb_paths[patchID_j_1], rpc_j_1);
				rpcf.ReadRpc_singleFile(strips[stripID_j_2].rpb_paths[patchID_j_2], rpc_j_2);

				int w_i_1, h_i_1, w_i_2, h_i_2;
				int w_j_1, h_j_1, w_j_2, h_j_2;
				ipro.GetImageSize(strips[stripID_i_1].img_paths[patchID_i_1], w_i_1, h_i_1);
				ipro.GetImageSize(strips[stripID_i_2].img_paths[patchID_i_2], w_i_2, h_i_2);
				ipro.GetImageSize(strips[stripID_j_1].img_paths[patchID_j_1], w_j_1, h_j_1);
				ipro.GetImageSize(strips[stripID_j_2].img_paths[patchID_j_2], w_j_2, h_j_2);

				double ix1, iy1, ix2, iy2;
				double jx1, jy1, jx2, jy2;
				for (int k = 0; k < pairs[i].ptNum;k++)
				{
					if (ifrepeat[i][k]==false)
					{
						ix1 = pairs[i].corresList[k].x1;
						iy1 = pairs[i].corresList[k].y1;

						ix2 = pairs[i].corresList[k].x2;
						iy2 = pairs[i].corresList[k].y2;

						//transform i1 i2 to j1 j2
						double lat1, lon1, lat2, lon2;
						rpcf.RPC_Image2Ground(iy1, ix1, m_avg_height, rpc_i_1, lat1, lon1);
						rpcf.RPC_Image2Ground(iy2, ix2, m_avg_height, rpc_i_2, lat2, lon2);

						rpcf.RPC_Ground2Image(lat1, lon1, m_avg_height, rpc_j_1, jy1, jx1);
						rpcf.RPC_Ground2Image(lat2, lon2, m_avg_height, rpc_j_2, jy2, jx2);

// 						if (jx1>=0 )
// 						{
// 						}
					}
				}
			}
		}
	}

	//free memory
	for (int i = 0; i < pairNum;i++)
	{
		delete[]pair_correspondence_path[i]; pair_correspondence_path[i] = NULL;
	}
 	delete[]pair_correspondence_path; pair_correspondence_path = NULL;
	for (int i = 0; i < pairNum;i++)
	{
		delete[]pairs[i].corresList; pairs[i].corresList = NULL;
	}
	delete[]pairs; pairs = NULL;

	for (int i = 0; i < pairNum;i++)
	{
		delete[]ifrepeat[i]; ifrepeat[i] = NULL;
	}
	delete[]ifrepeat; ifrepeat = NULL;

	return 1;
}

int CImageFeatureMatching::FindBestPatchIDforReg(Ground_Img_Pt *ground_img_pts, int ptNum, int cur_stripID, int *stripIDs,
	Long_Strip_Img *strips, int stripNum, int regNum, 
	double *&lat, double *&lon, double *&hei, double *&ix, double *&iy, int &gcp_num, int &patchID)
{
	CRPC_Fun rpcf;
	vector <double> vLat, vLon, vHei, vix, viy;
	patchID = 0;
	
	if (strips[cur_stripID].patchNum==1)
	{
		patchID = 0;
	}
	else
	{
		int *corresp_num_array = new int[strips[cur_stripID].patchNum];
		memset(corresp_num_array, 0, sizeof(int)*strips[cur_stripID].patchNum);

		for (int j = 0; j < ptNum; j++)
		{
			bool ifcontain_cur_strip = false;
			int contain_reg_strip = 0;
			int trackNum = ground_img_pts[j].ptNum;
			for (int k = 0; k < trackNum; k++)
			{
				char *imgID = ground_img_pts[j].img_pts[k].imgID;
				int tmp_stripID, tmp_patchID;
				sscanf(imgID, "%d_%d", &tmp_stripID, &tmp_patchID);
				if (tmp_stripID == cur_stripID)
				{
					ifcontain_cur_strip = true;
				}
				for (int t = 0; t < regNum; t++)
				{
					if (tmp_stripID == stripIDs[t])
					{
						contain_reg_strip++;
					}
				}
			}

			if (ifcontain_cur_strip == true && contain_reg_strip >= 2)
			{
				for (int k = 0; k < trackNum; k++)
				{
					char *imgID = ground_img_pts[j].img_pts[k].imgID;
					int tmp_stripID, tmp_patchID;
					sscanf(imgID, "%d_%d", &tmp_stripID, &tmp_patchID);
					if (tmp_stripID == cur_stripID)
					{
						corresp_num_array[tmp_patchID]++;
						break;
					}
				}
			}
		}

		int num_most = 0;
		patchID = 0;

		for (int i = 0; i < strips[cur_stripID].patchNum;i++)
		{
			if (corresp_num_array[i]>num_most)
			{
				patchID = i;
				num_most = corresp_num_array[i];
			}
		}

		delete[]corresp_num_array; corresp_num_array = NULL;
	}

	//According to the patchID, find the corresponding gcp and image points
	for (int i = 0; i < ptNum;i++)
	{
		int trackNum = ground_img_pts[i].ptNum;
		bool ifcontain_cur_strip = false;
		int contain_reg_strip = 0;
		for (int j = 0; j < trackNum;j++)
		{
			char *imgID = ground_img_pts[i].img_pts[j].imgID;
			int tmp_stripID, tmp_patchID;
			sscanf(imgID, "%d_%d", &tmp_stripID, &tmp_patchID);
			if (tmp_stripID == cur_stripID && patchID ==  tmp_patchID)
			{
				ifcontain_cur_strip = true;
			}
			for (int t = 0; t < regNum; t++)
			{
				if (tmp_stripID == stripIDs[t])
				{
					contain_reg_strip++;
				}
			}
		}

		if (ifcontain_cur_strip == true && contain_reg_strip>=2)
		{
			double tLat = 0, tLon = 0, tHei = 0;
			double tix = 0, tiy = 0;

			//record image points
			for (int j = 0; j < trackNum; j++)
			{
				char *imgID = ground_img_pts[i].img_pts[j].imgID;
				int tmp_stripID, tmp_patchID;
				sscanf(imgID, "%d_%d", &tmp_stripID, &tmp_patchID);

				if (tmp_stripID == cur_stripID && patchID == tmp_patchID)
				{
					tix = ground_img_pts[i].img_pts[j].x;
					tiy = ground_img_pts[i].img_pts[j].y;

					break;
				}
			}

			//record lat, lon, hei
			int *reg_stripID = new int[contain_reg_strip];
			int *reg_patchID = new int[contain_reg_strip];
			double *reg_x = new double[contain_reg_strip];
			double *reg_y = new double[contain_reg_strip];
			RPC *reg_rpc = new RPC[contain_reg_strip];
			int iter = 0;
			for (int j = 0; j < trackNum; j++)
			{
				char *imgID = ground_img_pts[i].img_pts[j].imgID;
				int tmp_stripID, tmp_patchID;
				sscanf(imgID, "%d_%d", &tmp_stripID, &tmp_patchID);

				for (int t = 0; t < regNum; t++)
				{
					if (tmp_stripID == stripIDs[t])
					{
						reg_stripID[iter] = tmp_stripID;
						reg_patchID[iter] = tmp_patchID;
						reg_x[iter] = ground_img_pts[i].img_pts[j].x;
						reg_y[iter] = ground_img_pts[i].img_pts[j].y;
						rpcf.ReadRpc_singleFile(strips[tmp_stripID].refined_rpb_paths[tmp_patchID], reg_rpc[iter]);
						iter++;
						break;
					}
				}
			}

			//multi-view forward intersection
			rpcf.Multi_view_Forward_intersection(reg_x, reg_y, reg_rpc, contain_reg_strip, tLat, tLon, tHei);

			vLat.push_back(tLat);
			vLon.push_back(tLon);
			vHei.push_back(tHei);
			vix.push_back(tix);
			viy.push_back(tiy);

			delete[]reg_stripID; reg_stripID = NULL;
			delete[]reg_patchID; reg_patchID = NULL;
			delete[]reg_x; reg_x = NULL;
			delete[]reg_y; reg_y = NULL;
			delete[]reg_rpc; reg_rpc = NULL;
		}
	}

	gcp_num = vLat.size();
	lat = new double[gcp_num];
	lon = new double[gcp_num];
	hei = new double[gcp_num];
	ix = new double[gcp_num];
	iy = new double[gcp_num];

	for (int i = 0; i < gcp_num;i++)
	{
		lat[i] = vLat[i];
		lon[i] = vLon[i];
		hei[i] = vHei[i];
		ix[i] = vix[i];
		iy[i] = viy[i];
	}

	//free memory
	vLat.clear();
	vLon.clear();
	vHei.clear();
	vix.clear();
	viy.clear();
	vector<double>().swap(vLat);
	vector<double>().swap(vLon);
	vector<double>().swap(vHei);
	vector<double>().swap(vix);
	vector<double>().swap(viy);

	return 1;
}

int CImageFeatureMatching::Coarse_Reg(char *pair_mch_list, char *Multi_view_track_list, Long_Strip_Img *strips_aoi, int stripNum, char *Bundle_adj_list)
{
	CRPC_Fun rpcf;

	//Read tracking point result
	FILE *fp = fopen(Multi_view_track_list, "r");
	char str[MaxLen];
	fgets(str, MaxLen, fp);
	int ptNum_ground = 0;
	int ptNum_img = 0;
	double Deltax_total = 0, Deltay_total = 0;
	while (strcmp(str, "END\n"))
	{
		int single_num = 0;
		sscanf(str, "%*lf %*lf %*lf %d", &single_num);
		for (int i = 0; i < single_num; i++)
		{
			fgets(str, MaxLen, fp);
			double deltax_single, deltay_single;
			sscanf(str, "%*s %*lf %*lf %lf %lf", &deltax_single, &deltay_single);

			Deltax_total += fabs(deltax_single);
			Deltay_total += fabs(deltay_single);
			ptNum_img++;
		}
		ptNum_ground++;
		fgets(str, MaxLen, fp);
	}
	fclose(fp); fp = NULL;

	Deltax_total /= ptNum_img;
	Deltay_total /= ptNum_img;

	Ground_Img_Pt *ground_img_pts = new Ground_Img_Pt[ptNum_ground];
	fp = fopen(Multi_view_track_list, "r");
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

	//1. find the pair with the most correspondences as the basis
	int pairNum = 0;
	fp = fopen(pair_mch_list, "r");
	fscanf(fp, "%d", &pairNum);
	PairInfor *Pairs = new PairInfor[pairNum];
	for (int i = 0; i < pairNum;i++)
	{
		fscanf(fp, "%s", Pairs[i].left_right_corrpendence_path);
	}
	fclose(fp); fp = NULL;

	int num_most = 0;
	char ID1_best[100], ID2_best[100];
	int pairID_best = 0;
	int stripID1_best = 0, stripID2_best = 0;
	int patchID1_best = 0, patchID2_best = 0;
	for (int i = 0; i < pairNum; i++)
	{
		char *mch_corresp_path = Pairs[i].left_right_corrpendence_path;
		FILE *fp_mch = fopen(mch_corresp_path, "r");
		char cur_ID1[100], cur_ID2[100];
		fscanf(fp_mch, "%s %s", cur_ID1, cur_ID2);
		int cur_num = 0;
		fscanf(fp_mch, "%d", &cur_num);
		fclose(fp_mch); fp_mch = NULL;

		int cur_stripID1, cur_stripID2;
		int cur_patchID1, cur_patchID2;
		sscanf(cur_ID1, "%d_%d", &cur_stripID1, &cur_patchID1);
		sscanf(cur_ID2, "%d_%d", &cur_stripID2, &cur_patchID2);
		if (cur_num > num_most)
		{
			num_most = cur_num;
			strcpy(ID1_best, cur_ID1);
			strcpy(ID2_best, cur_ID2);
			pairID_best = i;
			stripID1_best = cur_stripID1;
			stripID2_best = cur_stripID2;
			patchID1_best = cur_patchID1;
			patchID2_best = cur_patchID2;
		}
	}

	//Read the matching points of the best pair
	double *x1, *y1, *x2, *y2;
	x1 = new double[num_most];
	y1 = new double[num_most];
	x2 = new double[num_most];
	y2 = new double[num_most];

	FILE *fp_read = fopen(Pairs[pairID_best].left_right_corrpendence_path, "r");
	fscanf(fp_read, "%*s %*s");
	fscanf(fp_read, "%d", &num_most);
	for (int i = 0; i < num_most; i++)
	{
		fscanf(fp_read, "%lf %lf %lf %lf", &x1[i], &y1[i], &x2[i], &y2[i]);
	}
	fclose(fp_read); fp_read = NULL;

	//refine the strip of ID2_best, the strip of ID1_best is basis
	double drow_basis, dcol_basis;
	RPC rpc1_basis, rpc2_basis;
	rpcf.ReadRpc_singleFile(strips_aoi[stripID1_best].rpb_paths[patchID1_best], rpc1_basis); 
	rpcf.ReadRpc_singleFile(strips_aoi[stripID2_best].rpb_paths[patchID2_best], rpc2_basis);
	double error_reprojection;

	printf("Select ID = %d strip as basis\n", stripID1_best);
	printf("Starting relatvie orientation of ID = %d strip\n", stripID2_best);
	rpcf.RelativeOrientation_bias_constant(x1, y1, x2, y2, num_most, rpc1_basis, rpc2_basis, drow_basis, dcol_basis, error_reprojection);

	//the basis can not be changed
	for (int i = 0; i<strips_aoi[stripID1_best].patchNum; i++)
	{
		RPC rpc_tmp;
		rpcf.ReadRpc_singleFile(strips_aoi[stripID1_best].rpb_paths[i], rpc_tmp);
		rpcf.WriteRPBFile(rpc_tmp, strips_aoi[stripID1_best].refined_rpb_paths[i]);
	}

	for (int i = 0; i < strips_aoi[stripID2_best].patchNum; i++)
	{
		RPC rpc_tmp;
		rpcf.ReadRpc_singleFile(strips_aoi[stripID2_best].rpb_paths[i], rpc_tmp);
		rpcf.RefineRPC_bias_constant(rpc_tmp, drow_basis, dcol_basis);
		rpcf.WriteRPBFile(rpc_tmp, strips_aoi[stripID2_best].refined_rpb_paths[i]);
	}

	printf("Refine ID  =  %d's RPC\n", stripID2_best);
	printf("drow = %lf    dcol = %lf\n", drow_basis, dcol_basis);
	printf("Reprojection error is: %lf\n", error_reprojection);

	//refine other stips one by one
	int *stripIDs = new int[stripNum];
	stripIDs[0] = stripID1_best;
	stripIDs[1] = stripID2_best;
	int exist_ID_num = 2;
	iter = 2;
	for (int iter = 2; iter < stripNum;iter++)
	{
		int *strips_Num = new int[stripNum];
		memset(strips_Num, 0, sizeof(int)*stripNum);

		for (int i = 0; i < ptNum_ground;i++)
		{
			int same_strip_num = 0;
			for (int k = 0; k < exist_ID_num;k++)
			{
				for (int j = 0; j < ground_img_pts[i].ptNum; j++)
				{
					char *pt_ID = ground_img_pts[i].img_pts[j].imgID;
					int pt_strip_ID, pt_patch_ID;
					sscanf(pt_ID, "%d_%d", &pt_strip_ID, &pt_patch_ID);

					if (pt_strip_ID == stripIDs[k])
					{
						same_strip_num++;
						break;
					}
				}
			}

			if (same_strip_num>=2) // it is the GCP
			{
				for (int j = 0; j < ground_img_pts[i].ptNum; j++)
				{
					char *pt_ID = ground_img_pts[i].img_pts[j].imgID;
					int pt_strip_ID, pt_patch_ID;
					sscanf(pt_ID, "%d_%d", &pt_strip_ID, &pt_patch_ID);

					bool ifrepeat = false;
					for (int t = 0; t < exist_ID_num;t++)
					{
						if (pt_strip_ID == stripIDs[t])
						{
							ifrepeat = true;
							break;
						}
					}

					if (ifrepeat == false)
					{
						strips_Num[pt_strip_ID]++;
					}
				}
			}
		}

		int most_ID = 0;
		int most_num = 0;
		for (int i = 0; i < stripNum;i++)
		{
			if (strips_Num[i]>most_num)
			{
				most_num = strips_Num[i];
				most_ID = i;
			}
		}
		stripIDs[iter] = most_ID;
		exist_ID_num++;
		delete[]strips_Num; strips_Num = NULL;
	}

	for (int i = 2; i < stripNum; i++)
	{
		int cur_stripID = stripIDs[i];
		double *lat, *lon, *hei;
		double *ix, *iy;
		int gcpNum = 0;
		int patchID_cur;
		FindBestPatchIDforReg(ground_img_pts, ptNum_ground, cur_stripID, stripIDs, strips_aoi, stripNum, i,
			lat, lon, hei, ix, iy, gcpNum, patchID_cur);

		double drow_cur = 0, dcol_cur = 0;
		RPC cur_rpc;
		double error_reproj_cur = 0;
		rpcf.ReadRpc_singleFile(strips_aoi[cur_stripID].rpb_paths[patchID_cur], cur_rpc);
		rpcf.RPC_ComputeConstantBiasUsingGCP(ix, iy, lat, lon, hei, gcpNum, cur_rpc, drow_cur, dcol_cur, error_reproj_cur);

		for (int j = 0; j < strips_aoi[cur_stripID].patchNum; j++)
		{
			RPC rpc_tmp;
			rpcf.ReadRpc_singleFile(strips_aoi[cur_stripID].rpb_paths[j], rpc_tmp);
			rpcf.RefineRPC_bias_constant(rpc_tmp, drow_cur, dcol_cur);
			rpcf.WriteRPBFile(rpc_tmp, strips_aoi[cur_stripID].refined_rpb_paths[j]);
		}

		printf("Refine ID  =  %d RPC\n", cur_stripID);
		printf("GCP Num is: %d \n", gcpNum);
		printf("drow = %lf    dcol = %lf\n", drow_cur, dcol_cur);
		printf("Reprojection error is: %lf\n", error_reproj_cur);

		//free memory;
		delete[]lat; lat = NULL;
		delete[]lon; lon = NULL;
		delete[]hei; hei = NULL;
		delete[]ix; ix = NULL;
		delete[]iy; iy == NULL;
	}

	//output new tracking points
	fp = fopen(Bundle_adj_list, "w");

	for (int i = 0; i < ptNum_ground;i++)
	{
		double *ix, *iy;
		RPC *rpcs;
		int cur_num = ground_img_pts[i].ptNum;
		ix = new double[cur_num];
		iy = new double[cur_num];
		rpcs = new RPC[cur_num];
		double tLat, tLon, tHei;
		for (int j = 0; j < cur_num;j++)
		{
			char *pt_id;
			double deltax, deltay;
			int pt_strip_id, pt_patch_id;
			ix[j] = ground_img_pts[i].img_pts[j].x;
			iy[j] = ground_img_pts[i].img_pts[j].y;
			pt_id = ground_img_pts[i].img_pts[j].imgID;
			sscanf(pt_id, "%d_%d", &pt_strip_id, &pt_patch_id);
			rpcf.ReadRpc_singleFile(strips_aoi[pt_strip_id].refined_rpb_paths[pt_patch_id], rpcs[j]);
		}

		rpcf.Multi_view_Forward_intersection(ix, iy, rpcs, cur_num, tLat, tLon, tHei);
		ground_img_pts[i].Lat = tLat;
		ground_img_pts[i].Lon = tLon;
		ground_img_pts[i].Hei = tHei;

		int realnum = 0;
		for (int j = 0; j < cur_num; j++)
		{
			double reproj_x, reproj_y;
			rpcf.RPC_Ground2Image(tLat, tLon, tHei, rpcs[j], reproj_y, reproj_x);

			double deltax, deltay;
			deltax = reproj_x - ix[j];
			deltay = reproj_y - iy[j];
			
			if (fabs(deltax)<3 && fabs(deltay)<3)
			{
				realnum++;
			}
		}

		if (realnum>=2)
		{
			fprintf(fp, "%lf %lf %lf %d\n", tLat, tLon, tHei, realnum/*cur_num*/);
			for (int j = 0; j < cur_num; j++)
			{
				double reproj_x, reproj_y;
				rpcf.RPC_Ground2Image(tLat, tLon, tHei, rpcs[j], reproj_y, reproj_x);

				double deltax, deltay;
				deltax = reproj_x - ix[j];
				deltay = reproj_y - iy[j];
				ground_img_pts[i].img_pts[j].deltax = deltax;
				ground_img_pts[i].img_pts[j].deltay = deltay;

				if (fabs(deltax) < 3 && fabs(deltay) < 3)
				{
					fprintf(fp, "%s %lf %lf %lf %lf\n", ground_img_pts[i].img_pts[j].imgID, ix[j], iy[j], deltax, deltay);
				}

			}
		}
		else
		{
			for (int j = 0; j < cur_num; j++)
			{
				double reproj_x, reproj_y;
				rpcf.RPC_Ground2Image(tLat, tLon, tHei, rpcs[j], reproj_y, reproj_x);

				double deltax, deltay;
				deltax = reproj_x - ix[j];
				deltay = reproj_y - iy[j];
				ground_img_pts[i].img_pts[j].deltax = deltax;
				ground_img_pts[i].img_pts[j].deltay = deltay;
			}
		}
		

		//free memory
		delete[]ix; ix = NULL;
		delete[]iy; iy = NULL;
		delete[]rpcs; rpcs = NULL;
	}
	fprintf(fp, "END\n");
	fclose(fp); fp = NULL;

	double re_projection_error = 0;
	double Dx = 0, Dy = 0;
	for (int i = 0; i < ptNum_ground;i++)
	{
		int patchNum = ground_img_pts[i].ptNum;
		for (int j = 0; j < patchNum;j++)
		{
			Dx += fabs(ground_img_pts[i].img_pts[j].deltax);
			Dy += fabs(ground_img_pts[i].img_pts[j].deltay);

			re_projection_error += ground_img_pts[i].img_pts[j].deltax*ground_img_pts[i].img_pts[j].deltax + ground_img_pts[i].img_pts[j].deltay*ground_img_pts[i].img_pts[j].deltay;
		}
	}
	Dx /= ptNum_img;
	Dy /= ptNum_img;
	re_projection_error = sqrt(re_projection_error/ ptNum_img);

	printf("The geo accuracy after coarse registration is (pixels): avg_dx = %lf    avg_dy = %lf    RMSE_XY = %lf\n", Dx, Dy, re_projection_error);

	//free memory
	for (int i = 0; i < ptNum_ground; i++)
	{
		delete[]ground_img_pts[i].img_pts; ground_img_pts[i].img_pts = NULL;
	}
	delete[]ground_img_pts; ground_img_pts = NULL;

	delete[]x1; x1 = NULL;
	delete[]x2; x2 = NULL;
	delete[]y1; y1 = NULL;
	delete[]y2; y2 = NULL;
	delete[]stripIDs; stripIDs = NULL;
	delete[]Pairs; Pairs = NULL;

	return 1;
}

int CImageFeatureMatching::Coarse_Reg(PairInfor *Pairs, int pairNum, Long_Strip_Img *strips_aoi, int stripNum, char *output_list)
{
	CRPC_Fun rpcf;

	//read correspondences of each pair
// 	MchPtInfor *mchPairs = new MchPtInfor[pairNum];
// 	for (int i = 0; i < pairNum; i++)
// 	{
// 		char pair_matching_result_path[MaxLen];
// 		strcpy(pair_matching_result_path, Pairs[i].left_right_corrpendence_path);
// 		FILE *fp_pair = fopen(pair_matching_result_path, "r");
// 
// 		fscanf(fp_pair, "%s %s", mchPairs[i].ID1, mchPairs[i].ID2);
// 		fscanf(fp_pair, "%d", &mchPairs[i].ptNum);
// 
// 		mchPairs[i].corresList = new CorrespondencePts[mchPairs[i].ptNum];
// 
// 		for (int j = 0; j < mchPairs[i].ptNum; j++)
// 		{
// 			fscanf(fp_pair, "%f %f %f %f", &mchPairs[i].corresList[j].x1, &mchPairs[i].corresList[j].y1,
// 				&mchPairs[i].corresList[j].x2, &mchPairs[i].corresList[j].y2);
// 		}
// 
// 		fclose(fp_pair); fp_pair = NULL;
// 	}
// 
// 	//
// 	bool **ifprocessed;
// 	ifprocessed = new bool *[pairNum];
// 	for (int i = 0; i < pairNum; i++)
// 	{
// 		ifprocessed[i] = new bool[mchPairs[i].ptNum];
// 		memset(ifprocessed[i], 0, sizeof(bool)*mchPairs[i].ptNum);
// 	}
// 
// 	bool **ifprocessed_backup;
// 	ifprocessed_backup = new bool *[pairNum];
// 	for (int i = 0; i < pairNum; i++)
// 	{
// 		ifprocessed_backup[i] = new bool[mchPairs[i].ptNum];
// 		memset(ifprocessed_backup[i], 0, sizeof(bool)*mchPairs[i].ptNum);
// 	}
// 
// 	FILE *fp_output = fopen(output_list, "w");
// 	for (int i = 0; i < pairNum; i++)
// 	{
// 		int cur_ptNum = mchPairs[i].ptNum;
// 		char *ID1, *ID2;
// 		ID1 = mchPairs[i].ID1;
// 		ID2 = mchPairs[i].ID2;
// 		for (int j = 0; j < cur_ptNum; j++)
// 		{
// 			if (ifprocessed[i][j] == false) //not processed before
// 			{
// 				//backup for the ifprocessed
// 				for (int k = 0; k < pairNum; k++)
// 				{
// 					memcpy(ifprocessed_backup[k], ifprocessed[k], sizeof(bool)*mchPairs[k].ptNum);
// 				}
// 
// 				CorrespondencePts cur_pt;
// 				strcpy(cur_pt.ID1, ID1);
// 				strcpy(cur_pt.ID2, ID2);
// 				cur_pt.x1 = mchPairs[i].corresList[j].x1;
// 				cur_pt.y1 = mchPairs[i].corresList[j].y1;
// 				cur_pt.x2 = mchPairs[i].corresList[j].x2;
// 				cur_pt.y2 = mchPairs[i].corresList[j].y2;
// 
// 				vector <CorrespondencePts> pt_array;
// 				pt_array.push_back(cur_pt);
// 				ifprocessed[i][j] = true;
// 				int iter = 0;
// 				while (pt_array.size() > iter)
// 				{
// 					cur_pt = pt_array[iter];
// 
// 					FindConnectPt(cur_pt, mchPairs, pairNum, ifprocessed, pt_array);
// 
// 					iter++;
// 				}
// 
// 				//remove outliers and compute the corresponding ground point
// 				vector<CorrespondencePts> new_pt_array;
// 				double Lat, Lon, Hei;
// 				ComputeGroundPtsFromMultiMchPts(pt_array, strips_aoi, stripNum,
// 					new_pt_array, Lat, Lon, Hei, mchPairs, pairNum, ifprocessed_backup);
// 
// // 				if (pt_array.size() == new_pt_array.size()) //totally correct matches
// // 				{
// 					//output result
// 					//int size = new_pt_array.size();
// 					/*OutputTrackingPtsResults(fp_output, new_pt_array, strips_aoi, stripNum, Lat, Lon, Hei);*/
// 					OutputTrackingPtsResults(fp_output, pt_array, strips_aoi, stripNum, Lat, Lon, Hei);
// 
// 					//free memory
// 					pt_array.clear();
// 					vector<CorrespondencePts>().swap(pt_array);
// 					new_pt_array.clear();
// 					vector<CorrespondencePts>().swap(new_pt_array);
// /*				}*/
// 			}
// 		}
// 	}
// 
// 	fprintf(fp_output, "%s\n", "END");
//	fclose(fp_output); fp_output = NULL;

	////////////////////////////////////////////////////////////////////////////Starting coarse registration
	//Read tracking point result
	FILE *fp = fopen(output_list, "r");
	char str[MaxLen];
	fgets(str, MaxLen, fp);
	int ptNum_ground = 0;
	int ptNum_img = 0;
	double Deltax_total = 0, Deltay_total = 0;
	while (strcmp(str, "END\n"))
	{
		int single_num = 0;
		sscanf(str, "%*lf %*lf %*lf %d", &single_num);
		for (int i = 0; i < single_num; i++)
		{
			fgets(str, MaxLen, fp);
			double deltax_single, deltay_single;
			sscanf(str, "%*s %*lf %*lf %lf %lf", &deltax_single, &deltay_single);

			Deltax_total += fabs(deltax_single);
			Deltay_total += fabs(deltay_single);
			ptNum_img++;
		}
		ptNum_ground++;
		fgets(str, MaxLen, fp);
	}
	fclose(fp); fp = NULL;

	Deltax_total /= ptNum_img;
	Deltay_total /= ptNum_img;

	printf("Totally %d ground points\n", ptNum_ground);
	printf("Totally %d image points\n", ptNum_img);
	printf("Average re-projection errors before coarse registration are: avgx = %lf avgy = %lf\n", Deltax_total, Deltay_total);

	Ground_Img_Pt *ground_img_pts = new Ground_Img_Pt[ptNum_ground];
	fp = fopen(output_list, "r");
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

	//1. find the pair with the most correspondences as the basis
	int num_most = 0;
	char ID1_best[100], ID2_best[100];
	int pairID_best = 0;
	int stripID1_best = 0, stripID2_best = 0;
	for (int i = 0; i < pairNum;i++)
	{
		char *mch_corresp_path = Pairs[i].left_right_corrpendence_path;
		FILE *fp_mch = fopen(mch_corresp_path, "r");
		char cur_ID1[100], cur_ID2[100];
		fscanf(fp_mch, "%s %s", cur_ID1, cur_ID2);
		int cur_num = 0;
		fscanf(fp_mch, "%d", &cur_num);
		fclose(fp_mch); fp_mch = NULL;

		int cur_stripID1, cur_stripID2;
		int cur_patchID1, cur_patchID2;
		sscanf(cur_ID1, "%d_%d", &cur_stripID1, &cur_patchID1);
		sscanf(cur_ID2, "%d_%d", &cur_stripID2, &cur_patchID2);
		if (cur_num>num_most)
		{
			num_most = cur_num;
			strcpy(ID1_best, cur_ID1);
			strcpy(ID2_best, cur_ID2);
			pairID_best = i;
			stripID1_best = cur_stripID1;
			stripID2_best = cur_stripID2;
		}
	}

	//Read the matching points of the best pair
	double *x1, *y1, *x2, *y2;
	x1 = new double[num_most];
	y1 = new double[num_most];
	x2 = new double[num_most];
	y2 = new double[num_most];

	FILE *fp_read = fopen(Pairs[pairID_best].left_right_corrpendence_path, "r");
	fscanf(fp_read, "%*s %*s");
	fscanf(fp_read, "%d", &num_most);
	for (int i = 0; i < num_most;i++)
	{
		fscanf(fp_read, "%lf %lf %lf %lf", &x1[i], &y1[i], &x2[i], &y2[i]);
	}
	fclose(fp_read); fp_read = NULL;

	//refine the strip of ID2_best, the strip of ID1_best is basis
	double drow_basis, dcol_basis;
	RPC rpc1_basis, rpc2_basis;
	rpcf.ReadRpc_singleFile(Pairs[pairID_best].left_aoi_rpb, rpc1_basis);
	rpcf.ReadRpc_singleFile(Pairs[pairID_best].right_aoi_rpb, rpc2_basis);
	double error_reprojection;
	rpcf.RelativeOrientation_bias_constant(x1, y1, x2, y2, num_most, rpc1_basis, rpc2_basis, drow_basis, dcol_basis, error_reprojection);
	
	for (int i = 0; i < strips_aoi[stripID2_best].patchNum;i++)
	{
		RPC rpc_tmp;
		rpcf.ReadRpc_singleFile(strips_aoi[stripID2_best].rpb_paths[i], rpc_tmp);
		rpcf.RefineRPC_bias_constant(rpc_tmp, drow_basis, dcol_basis);
		rpcf.WriteRPBFile(rpc_tmp, strips_aoi[stripID2_best].rpb_paths[i]);
	}

	printf("Select ID = %d strip as basis\n", stripID1_best);
	printf("Refine ID  =  %d RPC\n", stripID2_best);
	printf("drow = %lf    dcol = %lf\n", drow_basis, dcol_basis);
	printf("Reprojection error is: %lf\n", error_reprojection);

	//refine other stips one by one
	int *stripIDs = new int[stripNum];
	stripIDs[0] = stripID1_best;
	stripIDs[1] = stripID2_best;
	iter = 2;
	for (int i = 0; i < stripNum;i++)
	{
		if (i!= stripID1_best && i!= stripID2_best)
		{
			stripIDs[iter] = i;
			iter++;
		}
	}

	for (int i = 2; i < stripNum;i++)
	{
		int cur_stripID = stripIDs[i];
		double *lat, *lon, *hei;
		double *ix, *iy;
		int gcpNum = 0;
		int patchID_cur;
		FindBestPatchIDforReg(ground_img_pts, ptNum_ground, cur_stripID, stripIDs, strips_aoi, stripNum, i,
			lat, lon, hei, ix, iy, gcpNum, patchID_cur);

		double drow_cur = 0, dcol_cur = 0;
		RPC cur_rpc;
		double error_reproj_cur = 0;
		rpcf.ReadRpc_singleFile(strips_aoi[cur_stripID].rpb_paths[patchID_cur], cur_rpc);
		rpcf.RPC_ComputeConstantBiasUsingGCP(ix, iy, lat, lon, hei, gcpNum, cur_rpc, drow_cur, dcol_cur, error_reproj_cur);

		for (int j = 0; j < strips_aoi[cur_stripID].patchNum; j++)
		{
			RPC rpc_tmp;
			rpcf.ReadRpc_singleFile(strips_aoi[cur_stripID].rpb_paths[j], rpc_tmp);
			rpcf.RefineRPC_bias_constant(rpc_tmp, drow_cur, dcol_cur);
			rpcf.WriteRPBFile(rpc_tmp, strips_aoi[cur_stripID].rpb_paths[j]);
		}

		printf("Refine ID  =  %d RPC\n", cur_stripID);
		printf("GCP Num is: %d \n", gcpNum);
		printf("drow = %lf    dcol = %lf\n", drow_cur, dcol_cur);
		printf("Reprojection error is: %lf\n", error_reproj_cur);

		//free memory;
		delete[]lat; lat = NULL;
		delete[]lon; lon = NULL;
		delete[]hei; hei = NULL;
		delete[]ix; ix = NULL;
		delete[]iy; iy == NULL;
	}

	//free memory
// 	for (int i = 0; i < pairNum; i++)
// 	{
// 		delete[]mchPairs[i].corresList; mchPairs[i].corresList = NULL;
// 		delete[]ifprocessed[i]; ifprocessed[i] = NULL;
// 		delete[]ifprocessed_backup[i]; ifprocessed_backup[i] = NULL;
// 	}
// 	delete[]mchPairs; mchPairs = NULL;
// 	delete[]ifprocessed; ifprocessed = NULL;
// 	delete[]ifprocessed_backup; ifprocessed_backup = NULL;

	for (int i = 0; i < ptNum_ground;i++)
	{
		delete[]ground_img_pts[i].img_pts; ground_img_pts[i].img_pts = NULL;
	}
	delete[]ground_img_pts; ground_img_pts = NULL;

	delete[]x1; x1 = NULL;
	delete[]x2; x2 = NULL;
	delete[]y1; y1 = NULL;
	delete[]y2; y2 = NULL;
	delete[]stripIDs; stripIDs = NULL;

	return 1;
}

int CImageFeatureMatching::FindCommonPts_Multi_view(PairInfor *Pairs, int pairNum, Long_Strip_Img *rect_imgs, int stripNum, char *output_list)
{
	//read correspondences of each pair
	MchPtInfor *mchPairs = new MchPtInfor[pairNum];
	for (int i = 0; i < pairNum;i++)
	{
		char pair_matching_result_path[MaxLen];
		strcpy(pair_matching_result_path, Pairs[i].left_right_corrpendence_path);
		FILE *fp_pair = fopen(pair_matching_result_path,"r");

		fscanf(fp_pair, "%s %s", mchPairs[i].ID1, mchPairs[i].ID2);
		fscanf(fp_pair, "%d", &mchPairs[i].ptNum);
		
		mchPairs[i].corresList = new CorrespondencePts[mchPairs[i].ptNum];

		for (int j = 0; j < mchPairs[i].ptNum; j++)
		{
			fscanf(fp_pair, "%f %f %f %f", &mchPairs[i].corresList[j].x1, &mchPairs[i].corresList[j].y1,
				&mchPairs[i].corresList[j].x2, &mchPairs[i].corresList[j].y2);
		}

		fclose(fp_pair); fp_pair = NULL;
	}

	bool **ifprocessed;
	ifprocessed = new bool *[pairNum];
	for (int i = 0; i < pairNum;i++)
	{
		ifprocessed[i] = new bool[mchPairs[i].ptNum];
		memset(ifprocessed[i], 0, sizeof(bool)*mchPairs[i].ptNum);
	}

	bool **ifprocessed_backup;
	ifprocessed_backup = new bool *[pairNum];
	for (int i = 0; i < pairNum; i++)
	{
		ifprocessed_backup[i] = new bool[mchPairs[i].ptNum];
		memset(ifprocessed_backup[i], 0, sizeof(bool)*mchPairs[i].ptNum);
	}

	FILE *fp_output = fopen(output_list, "w");
	for (int i = 0; i < pairNum;i++)
	{
		int cur_ptNum = mchPairs[i].ptNum;
		char *ID1, *ID2;
		ID1 = mchPairs[i].ID1;
		ID2 = mchPairs[i].ID2;
		for (int j = 0; j < cur_ptNum;j++)
		{
			if (ifprocessed[i][j]==false) //not processed before
			{
				//backup for the ifprocessed
				for (int k = 0; k < pairNum; k++)
				{
					memcpy(ifprocessed_backup[k], ifprocessed[k], sizeof(bool)*mchPairs[k].ptNum);
				}

				CorrespondencePts cur_pt;
				strcpy(cur_pt.ID1, ID1);
				strcpy(cur_pt.ID2, ID2);
				cur_pt.x1 = mchPairs[i].corresList[j].x1;
				cur_pt.y1 = mchPairs[i].corresList[j].y1;
				cur_pt.x2 = mchPairs[i].corresList[j].x2;
				cur_pt.y2 = mchPairs[i].corresList[j].y2;

				vector <CorrespondencePts> pt_array;
				pt_array.push_back(cur_pt);
				ifprocessed[i][j] = true;
				int iter = 0;
				while (pt_array.size()>iter)
				{
					cur_pt = pt_array[iter];

					FindConnectPt(cur_pt, mchPairs, pairNum, ifprocessed, pt_array);

					iter++;
				}

				//remove outliers and compute the corresponding ground point
				vector<CorrespondencePts> new_pt_array;
				double Lat, Lon, Hei;
				ComputeGroundPtsFromMultiMchPts(pt_array, rect_imgs, stripNum,
					new_pt_array, Lat, Lon, Hei, mchPairs, pairNum, ifprocessed_backup);
				
				if (pt_array.size()==new_pt_array.size()) //totally correct matches
				{
					//output result
					int size = new_pt_array.size();
					OutputTrackingPtsResults(fp_output, new_pt_array, rect_imgs, stripNum, Lat, Lon, Hei);

					//free memory
					pt_array.clear();
					vector<CorrespondencePts>().swap(pt_array);
					new_pt_array.clear();
					vector<CorrespondencePts>().swap(new_pt_array);
				}
// 				else //exist outliers
// 				{
// 					for (int k = 0; k < pairNum; k++)
// 					{
// 						memcpy(ifprocessed[k], ifprocessed_backup[k], sizeof(bool)*mchPairs[k].ptNum);
// 					}
// 
// 					//free memory
// 					pt_array.clear();
// 					vector<CorrespondencePts>().swap(pt_array);
// 					new_pt_array.clear();
// 					vector<CorrespondencePts>().swap(new_pt_array);
// 
// 					//check the valid of the first pt
// 					if (ifprocessed[i][j] == true)
// 					{
// 						continue;
// 					}
// 
// 					cur_pt;
// 					strcpy(cur_pt.ID1, ID1);
// 					strcpy(cur_pt.ID2, ID2);
// 					cur_pt.x1 = mchPairs[i].corresList[j].x1;
// 					cur_pt.y1 = mchPairs[i].corresList[j].y1;
// 					cur_pt.x2 = mchPairs[i].corresList[j].x2;
// 					cur_pt.y2 = mchPairs[i].corresList[j].y2;
// 
// 					pt_array.push_back(cur_pt);
// 					ifprocessed[i][j] = true;
// 					int iter = 0;
// 					while (pt_array.size() > iter)
// 					{
// 						cur_pt = pt_array[iter];
// 
// 						FindConnectPt(cur_pt, mchPairs, pairNum, ifprocessed, pt_array);
// 
// 						iter++;
// 					}
// 
// 					//This function should be improved as the purely multi-view forward intersection
// // 					ComputeGroundPtsFromMultiMchPts(pt_array, rpcs, tfws, levelID,
// // 						new_pt_array, Lat, Lon, Hei, mchPairs, pairNum, ifprocessed_backup);
// 					Multi_view_ForwardIntersection(pt_array, strips_aoi, stripNum,
// 						Lat, Lon, Hei);
// 
// 					OutputTrackingPtsResults(fp_output, pt_array, strips_aoi, stripNum, Lat, Lon, Hei);
// 
// 					//free memory
// 					pt_array.clear();
// 					vector<CorrespondencePts>().swap(pt_array);
// 				}
			}
		}
	}

	fprintf(fp_output, "%s\n", "END");
	fclose(fp_output); fp_output = NULL;

	//free memory
	for (int i = 0; i < pairNum; i++)
	{
		delete[]mchPairs[i].corresList; mchPairs[i].corresList = NULL;
		delete[]ifprocessed[i]; ifprocessed[i] = NULL;
		delete[]ifprocessed_backup[i]; ifprocessed_backup[i] = NULL;
	}
	delete[]mchPairs; mchPairs = NULL;
	delete[]ifprocessed; ifprocessed = NULL;
	delete[]ifprocessed_backup; ifprocessed_backup = NULL;

	return 1;
}

int CImageFeatureMatching::GetStripPatchID(char *ID, int &stripID, int &patchID)
{
	sscanf(ID, "%d_%d", &stripID, &patchID);

	return 1;
}

int CImageFeatureMatching::ComputeGroundPtsFromMultiMchPts(vector<CorrespondencePts> pt_array, 
	Long_Strip_Img *rect_imgs, int stripNum,
	vector<CorrespondencePts> &new_pt_array, double &Lat, double &Lon, double &hei,
	MchPtInfor *mchPair, int pairNum, bool **ifprocessed_backup)
{
	CRPC_Fun rpcf;
	CLL_UTM geo_conv;

	int totalNum = pt_array.size();
	int i;
	double *lat_array, *lon_array, *hei_array;
	double *dis_array;
	lat_array = new double[totalNum];
	lon_array = new double[totalNum];
	hei_array = new double[totalNum];
	dis_array = new double[totalNum];
	for (i = 0; i < totalNum;i++)
	{
		char *ID1, *ID2;
		double x1, y1, x2, y2;
		ID1 = pt_array[i].ID1;
		x1 = pt_array[i].x1;  y1 = pt_array[i].y1;
		ID2 = pt_array[i].ID2;
		x2 = pt_array[i].x2; y2 = pt_array[i].y2;

		int stripID1, stripID2;
		int patchID1, patchID2;
		GetStripPatchID(ID1, stripID1, patchID1);
		GetStripPatchID(ID2, stripID2, patchID2);

		RPC rpc1, rpc2;
		rpcf.ReadRpc_singleFile(rect_imgs[stripID1].refined_rpb_paths[patchID1], rpc1);
		rpcf.ReadRpc_singleFile(rect_imgs[stripID2].refined_rpb_paths[patchID2], rpc2);

		//compute ground points
		double tLat, tLon, tHei;
		rpcf.RPC_ForwardIntersection(rpc1, rpc2, x1, y1, x2, y2, tLat, tLon, tHei);

		lat_array[i] = tLat;
		lon_array[i] = tLon;
		hei_array[i] = tHei;
	}

	//LL2UTM
	double *gx_array, *gy_array;
	gx_array = new double[totalNum];
	gy_array = new double[totalNum];
	for (i = 0; i < totalNum;i++)
	{
		geo_conv.LL2UTM_fixedZone(lat_array[i], lon_array[i], m_zone, gx_array[i], gy_array[i]);
	}

	//Group and detect outliers
	bool *ifinlisers = new bool[totalNum];
	JudgeOutliersinForwardIntersection_LL(pt_array, 
		lat_array, lon_array, hei_array, totalNum, ifinlisers);

	//record the pairID of outliers
	for (i = 0; i < totalNum;i++)
	{
		if (ifinlisers[i]==0)
		{
			for (int k = 0; k < pairNum;k++)
			{
				if (strcmp(pt_array[i].ID1, mchPair[k].ID1)==0 && strcmp(pt_array[i].ID2, mchPair[k].ID2)==0)
				{
					int j = 0;
					for (j = 0; j < mchPair[k].ptNum;j++)
					{
						if (pt_array[i].x1 == mchPair[k].corresList[j].x1 && pt_array[i].y1 == mchPair[k].corresList[j].y1
							&& pt_array[i].x2 == mchPair[k].corresList[j].x2 && pt_array[i].y2 == mchPair[k].corresList[j].y2)
						{
							ifprocessed_backup[k][j] = true;
							break;
						}
					}
					break;
				}
			}
		}
	}

	//output results
	Lat = 0;
	Lon = 0;
	hei = 0;
	int k = 0;
	for (i = 0; i < totalNum; i++)
	{
		if (ifinlisers[i]==1)
		{
			new_pt_array.push_back(pt_array[i]);
			Lat += lat_array[i];
			Lon += lon_array[i];
			hei += hei_array[i];
		}
	}

	int size = new_pt_array.size();
	Lat /= size;
	Lon /= size;
	hei /= size;

	//free memory
	delete[]ifinlisers; ifinlisers = NULL;
	delete[]lat_array; lat_array = NULL;
	delete[]lon_array; lon_array = NULL;
	delete[]hei_array; hei_array = NULL;
	delete[]dis_array; dis_array = NULL;
	delete[]gx_array; gx_array = NULL;
	delete[]gy_array; gy_array = NULL;

	return 1;
}

int CImageFeatureMatching::JudgeOutliersinForwardIntersection_LL(vector<CorrespondencePts> pt_array, 
	double *lat, double *lon, double *hei, int num, bool *ifinliers)
{
	CRPC_Fun rpcf;
	for (int i = 0; i < num;i++)
	{
		ifinliers[i] = 1;
	}
	for (int i = 0; i < num;i++)
	{
		char *imgID1_i, *imgID2_i;
		int stripID1_i, stripID2_i;
		int patchID1_i, patchID2_i;
		float x1_i, y1_i, x2_i, y2_i;

		imgID1_i = pt_array[i].ID1;
		imgID2_i = pt_array[i].ID2;
		sscanf(imgID1_i, "%d_%d", &stripID1_i, &patchID1_i);
		sscanf(imgID2_i, "%d_%d", &stripID2_i, &patchID2_i);
		x1_i = pt_array[i].x1;
		y1_i = pt_array[i].y1;
		x2_i = pt_array[i].x2;
		y2_i = pt_array[i].y2;

		for (int j = 0; j < num;j++)
		{
			if (i==j)
			{
				continue;
			}

			char *imgID1_j, *imgID2_j;
			int stripID1_j, stripID2_j;
			int patchID1_j, patchID2_j;
			float x1_j, y1_j, x2_j, y2_j;

			imgID1_j = pt_array[j].ID1;
			imgID2_j = pt_array[j].ID2;
			sscanf(imgID1_j, "%d_%d", &stripID1_j, &patchID1_j);
			sscanf(imgID2_j, "%d_%d", &stripID2_j, &patchID2_j);
			x1_j = pt_array[j].x1;
			y1_j = pt_array[j].y1;
			x2_j = pt_array[j].x2;
			y2_j = pt_array[j].y2;

			if (stripID1_i == stripID1_j)
			{
				if (fabs(x1_i - x1_j)>0.1 || fabs(y1_i - y1_j)>0.1)
				{
					ifinliers[i] = false;
					break;
				}
			}
			if (stripID1_i == stripID2_j)
			{
				if (fabs(x1_i - x2_j) > 0.1 || fabs(y1_i - y2_j) > 0.1)
				{
					ifinliers[i] = false;
					break;
				}
			}
			if (stripID2_i == stripID1_j)
			{
				if (fabs(x2_i - x1_j) > 0.1 || fabs(y2_i - y1_j) > 0.1)
				{
					ifinliers[i] = false;
					break;
				}
			}
			if (stripID2_i == stripID2_j)
			{
				if (fabs(x2_i - x2_j) > 0.1 || fabs(y2_i - y2_j) > 0.1)
				{
					ifinliers[i] = false;
					break;
				}
			}
		}
	}

// 	double lat_cen = 0, lon_cen = 0, hei_cen = 0;
// 	double *dis = new double[num];
// 	for (i = 0; i < num;i++)
// 	{
// 		lat_cen += lat[i];
// 		lon_cen += lon[i];
// 		hei_cen += hei[i];
// 	}
// 	lat_cen /= num;
// 	lon_cen /= num;
// 	hei_cen /= num;
// 
// 	double rmse = 0;
// 	for (i = 0; i < num; i++)
// 	{
// 		char *ID1, *ID2;
// 		double x1, y1, x2, y2;
// 		ID1 = pt_array[i].ID1;
// 		x1 = pt_array[i].x1;  y1 = pt_array[i].y1;
// 		ID2 = pt_array[i].ID2;
// 		x2 = pt_array[i].x2; y2 = pt_array[i].y2;
// 
// 		int stripID1, stripID2;
// 		int patchID1, patchID2;
// 		GetStripPatchID(ID1, stripID1, patchID1);
// 		GetStripPatchID(ID2, stripID2, patchID2);
// 
// 		RPC rpc1, rpc2;
// 		rpcf.ReadRpc_singleFile(strips_aoi[stripID1].rpb_paths[patchID1], rpc1);
// 		rpcf.ReadRpc_singleFile(strips_aoi[stripID2].rpb_paths[patchID2], rpc2);
// 
// 		double lx, ly, rx, ry;
// 		rpcf.RPC_Ground2Image(lat_cen, lon_cen, hei_cen, rpc1, ly, lx);
// 		rpcf.RPC_Ground2Image(lat_cen, lon_cen, hei_cen, rpc2, ry, rx);
// 
// 		double lx_ori, ly_ori, rx_ori, ry_ori;
// 		ly_ori = pt_array[i].y1;
// 		lx_ori = pt_array[i].x1;
// 
// 		ry_ori = pt_array[i].y2;
// 		rx_ori = pt_array[i].x2;
// 
// 		double disL, disR;
// 		disL = sqrt((ly - ly_ori)*(ly - ly_ori) + (lx - lx_ori)*(lx - lx_ori));
// 		disR = sqrt((ry - ry_ori)*(ry - ry_ori) + (rx - rx_ori)*(rx - rx_ori));
// 		dis[i] = (disL+disR)/2;
// 	}
// 	//I think the median filter is much better for rmse computation
// 	MedianFilter(dis, num, rmse);
// 
// 	for (i = 0; i < num; i++)
// 	{
// 		if (dis[i] <=/*3*/3 * rmse || dis[i] <= m_thresh_inliers)
// 		{
// 			ifinliers[i] = 1;
// 		}
// 	}
// 
// 	//each image mostly provide only one image
// 
// 
// 	//free memory
// 	delete[]dis; dis = NULL;

	return 1;
}

int  CImageFeatureMatching::JudgeOutliersinForwardIntersection(vector<CorrespondencePts> pt_array, 
	double *gx, double *gy, double *hei, int num, bool *ifinliers)
{
	int i;
	memset(ifinliers, 0, sizeof(bool)*num);

	double gx_cen = 0, gy_cen = 0, hei_cen = 0;
	double *dis = new double[num];
	ComputeWeightedCenter(gx, gy, hei, num, gx_cen, gy_cen, hei_cen);

	double rmse = 0;
	for (i = 0; i < num;i++)
	{
		dis[i] = sqrt((gx[i] - gx_cen)*(gx[i] - gx_cen) + (gy[i] - gy_cen)*(gy[i] - gy_cen) + (hei[i] - hei_cen)*(hei[i] - hei_cen));
		//rmse += dis[i];rmse /= num;
	}
	//I think the median filter is much better for rmse computation
	MedianFilter(dis, num, rmse);

	for (i = 0; i < num;i++)
	{
		if (dis[i] <=/*3*/4 * rmse || dis[i] <= m_thresh_inliers)
		{
			ifinliers[i] = 1;
		}
		else
		{
			int aaa = 1;
		}
	}

	//each image mostly provide only one image
	

	//free memory
	delete[]dis; dis = NULL;
	return 1; 
}

int CImageFeatureMatching::ComputeWeightedCenter(double *gX, double *gY, double *gZ, int num, 
	double &gx_cen, double &gy_cen, double &gz_cen)
{ 
	gx_cen = 0;
	gy_cen = 0;
	gz_cen = 0;

	double gx_bef, gy_bef, gz_bef;
	gx_bef = gx_cen; 
	gy_bef = gy_cen;
	gz_bef = gz_cen;
	int i, j;
	for (i = 0; i < num;i++)
	{
		gx_cen += gX[i];
		gy_cen += gY[i];
		gz_cen += gZ[i];
	}
	gx_cen /= num;
	gy_cen /= num;
	gz_cen /= num;

	double dis;
	dis = sqrt((gx_cen - gx_bef)*(gx_cen - gx_bef) + (gy_cen - gy_bef)*(gy_cen - gy_bef) + (gz_cen - gz_bef)*(gz_cen - gz_bef));
	double *weight = new double[num];
	double *dis_array = new double[num];
	while (dis>1)
	{
		for (i = 0; i < num;i++)
		{
			dis_array[i] = 
				sqrt((gx_cen - gX[i])*(gx_cen - gX[i]) + (gy_cen - gY[i])*(gy_cen - gY[i]) + (gz_cen - gZ[i])*(gz_cen - gZ[i]))
				+0.000001;
			weight[i] = 1 / dis_array[i];
		}
		double total_weight = 0;
		for (i = 0; i < num;i++)
		{
			total_weight += weight[i];
		}
		for (i = 0; i < num;i++)
		{
			weight[i] /= total_weight;
		}
// 		double totaldis = 0;
// 		for (i = 0; i < num; i++)
// 			for (j = 0; j < num;j++)
// 			{
// 				if (i!=j)
// 				{
// 					weight[i] *= dis_array[j];
// 				}
// 			}
// 		for (i = 0; i < num;i++)
// 		{
// 			totaldis += weight[i];
// 		}
// 
// 		for (i = 0; i < num;i++)
// 		{
// 			weight[i] /= totaldis;
// 		}

		//update center coordinates
		gx_bef = gx_cen; gy_bef = gy_cen; gz_bef = gz_cen;
		gx_cen = 0; gy_cen = 0; gz_cen = 0;
		for (i = 0; i < num;i++)
		{
			gx_cen += weight[i] * gX[i];
			gy_cen += weight[i] * gY[i];
			gz_cen += weight[i] * gZ[i];
		}

		dis = sqrt((gx_cen - gx_bef)*(gx_cen - gx_bef) + (gy_cen - gy_bef)*(gy_cen - gy_bef) + (gz_cen - gz_bef)*(gz_cen - gz_bef));
	}

	//free memory
	delete[]weight; weight = NULL;
	delete[]dis_array; dis_array = NULL;
	return 1;
}

int CImageFeatureMatching::GetWeightedCenter(double *arr, int size, double sigma, double &center)
{
	center = 0;
	double *weight = new double[size];
	memset(weight, 0, sizeof(double)*size);
	double *dis = new double[size];
	memset(dis, 0, sizeof(double)*size);

	for (int i = 0; i < size;i++)
	{
		center += arr[i];
	}
	center /= size;

	for (int i = 0; i < size;i++)
	{
		dis[i] = fabs(arr[i] - center);
	}
	double RMSE = 0;
	for (int i = 0; i < size;i++)
	{ 
		RMSE += dis[i]*dis[i];
	}
	RMSE = sqrt(RMSE / size);

	for (int i = 0; i < size;i++)
	{
		weight[i] = exp(-dis[i] / (3*RMSE)/*sigma*/);
	}

	double center_before = center + 10;
	double deltaCenter = fabs(center_before - center);
	while (deltaCenter>0.1)
	{
		center_before = center;
		center = 0;
		double weight_sum = 0;

		for (int i = 0; i < size; i++)
		{
			center += weight[i] * arr[i];
			weight_sum += weight[i];
		}
		center /= weight_sum;

		for (int i = 0; i < size; i++)
		{
			dis[i] = fabs(arr[i] - center);
		}
		RMSE = 0;
		for (int i = 0; i < size; i++)
		{
			RMSE += dis[i] * dis[i];
		}
		RMSE = sqrt(RMSE / size);

		for (int i = 0; i < size; i++)
		{
			weight[i] = exp(-dis[i] / (3*RMSE)/*sigma*/);
		}
		deltaCenter = fabs(center - center_before);
	}

	delete[]weight; weight = NULL;
	delete[]dis; dis = NULL;
	return 1;
}

int CImageFeatureMatching::MedianFilter(double *arr, int size, double &midValue)
{
	double *arr_tmp = new double[size];
	memcpy(arr_tmp, arr, sizeof(double)*size);

	//bull sort
	if (size == 1)
	{
		midValue = arr_tmp[0];
	}
	else if (size == 2)
	{
		midValue = (arr_tmp[0] + arr_tmp[1]) / 2;
	}
	else
	{
		int i, j;
		for (i = 0; i < size / 2 + 1; i++)
		{
			for (j = 0; j < size - i - 1; j++)
			{
				if (arr_tmp[j] > arr_tmp[j + 1])
				{
					double tmp = arr_tmp[j];
					arr_tmp[j] = arr_tmp[j + 1];
					arr_tmp[j + 1] = tmp;
				}
			}
		}

		if (size%2 == 1)
		{
			midValue = arr_tmp[size / 2];
		}
		else
		{
			midValue = (arr_tmp[size / 2] + arr_tmp[size / 2 - 1]) / 2;
		}
	}

	//free memory
	delete[]arr_tmp; arr_tmp = NULL;

	return 1;
}

int CImageFeatureMatching::OutputTrackingPtsResults(FILE *fp, vector<CorrespondencePts> Pts, 
	Long_Strip_Img *strips, int stripNum,
	double Lat, double Lon, double Hei)
{
	CRPC_Fun rpcf;

	int size = Pts.size();
	int ptNum = 0;
	for (int i = 0; i < stripNum;i++)
	{
		for (int j = 0; j < size; j++)
		{
			char *imgID1 = Pts[j].ID1;
			char *imgID2 = Pts[j].ID2;

			int stripID1, stripID2;
			int patchID1, patchID2;
			sscanf(imgID1, "%d_%d", &stripID1, &patchID1);
			sscanf(imgID2, "%d_%d", &stripID2, &patchID2);

			if (stripID1 == i || stripID2 == i)
			{
				ptNum++;
				break;
			}
		}
	}
	fprintf(fp, "%lf %lf %lf %d\n", Lat, Lon, Hei, ptNum);
	for (int i = 0; i < stripNum; i++)
	{
		for (int j = 0; j < size; j++)
		{
			char *imgID1 = Pts[j].ID1;
			char *imgID2 = Pts[j].ID2;

			int stripID1, stripID2;
			int patchID1, patchID2;
			sscanf(imgID1, "%d_%d", &stripID1, &patchID1);
			sscanf(imgID2, "%d_%d", &stripID2, &patchID2);

			double x1, y1, x2, y2;
			x1 = Pts[j].x1; y1 = Pts[j].y1;
			x2 = Pts[j].x2; y2 = Pts[j].y2;
			if (stripID1 == i)
			{
				RPC rpc;
				rpcf.ReadRpc_singleFile(strips[stripID1].refined_rpb_paths[patchID1], rpc);
				double deltax, deltay;
				double cx, cy;
				rpcf.RPC_Ground2Image(Lat, Lon, Hei, rpc, cy, cx);
				deltay = cy - y1;
				deltax = cx - x1;
				fprintf(fp, "%s %lf %lf %lf %lf\n", imgID1, x1, y1, deltax, deltay);

				break;
			}
			else if (stripID2==i)
			{
				RPC rpc;
				rpcf.ReadRpc_singleFile(strips[stripID2].refined_rpb_paths[patchID2], rpc);
				double deltax, deltay;
				double cx, cy;
				rpcf.RPC_Ground2Image(Lat, Lon, Hei, rpc, cy, cx);
				deltay = cy - y2;
				deltax = cx - x2;
				fprintf(fp, "%s %lf %lf %lf %lf\n", imgID2, x2, y2, deltax, deltay);

				break;
			}
		}
	}

// 	for (int k = 0; k < size; k++)
// 	{
// 		CorrespondencePts pt_tmp = Pts[k];
// 		bool ifrepeated_1 = false;
// 		for (int i = 0; i < k; i++)
// 		{
// 			if (strcmp(pt_tmp.ID1, Pts[i].ID1) == 0 || strcmp(pt_tmp.ID1, Pts[i].ID2) == 0)
// 			{
// 				ifrepeated_1 = true;
// 			}
// 		}
// 
// 		if (ifrepeated_1 == false)
// 		{
// 			ptNum++;
// 		}
// 
// 		bool ifrepeated_2 = false;
// 		for (int i = 0; i < k; i++)
// 		{
// 			if (strcmp(pt_tmp.ID2, Pts[i].ID1)==0 || strcmp(pt_tmp.ID2, Pts[i].ID2)==0)
// 			{
// 				ifrepeated_2 = true;
// 			}
// 		}
// 
// 		if (ifrepeated_2 == false)
// 		{
// 			ptNum++;
// 		}
// 	}
//
// 	fprintf(fp, "%lf %lf %lf %d\n", Lat, Lon, Hei, ptNum);
// 	for (int k = 0; k < size; k++)
// 	{
// 		CorrespondencePts pt_tmp = Pts[k];
// 		bool ifrepeated_1 = false;
// 		for (int i = 0; i < k;i++)
// 		{
// 			if (strcmp(pt_tmp.ID1,Pts[i].ID1)==0 || strcmp(pt_tmp.ID1, Pts[i].ID2)==0)
// 			{
// 				ifrepeated_1 = true;
// 			}
// 		}
// 
// 		if (ifrepeated_1 == false)
// 		{
// 			int stripID, patchID;
// 			sscanf(pt_tmp.ID1, "%d_%d", &stripID, &patchID);
// 			RPC rpc;
// 			rpcf.ReadRpc_singleFile(strips[stripID].rpb_paths[patchID], rpc);
// 			double deltax, deltay;
// 			double cx, cy;
// 			rpcf.RPC_Ground2Image(Lat, Lon, Hei, rpc, cy, cx);
// 			deltay = fabs(cy - pt_tmp.y1);
// 			deltax = fabs(cx - pt_tmp.x1);
// 			fprintf(fp, "%s %lf %lf %lf %lf\n", pt_tmp.ID1, pt_tmp.x1, pt_tmp.y1, deltax, deltay);
// 		}
// 
// 		bool ifrepeated_2 = false;
// 		for (int i = 0; i < k; i++)
// 		{
// 			if (strcmp(pt_tmp.ID2, Pts[i].ID1)==0 || strcmp(pt_tmp.ID2, Pts[i].ID2)==0)
// 			{
// 				ifrepeated_2 = true;
// 			}
// 		}
// 
// 		if (ifrepeated_2 == false)
// 		{
// 			int stripID, patchID;
// 			sscanf(pt_tmp.ID2, "%d_%d", &stripID, &patchID);
// 			RPC rpc;
// 			rpcf.ReadRpc_singleFile(strips[stripID].rpb_paths[patchID], rpc);
// 			double deltax, deltay;
// 			double cx, cy;
// 			rpcf.RPC_Ground2Image(Lat, Lon, Hei, rpc, cy, cx);
// 			deltay = fabs(cy - pt_tmp.y2);
// 			deltax = fabs(cx - pt_tmp.x2);
// 
// 			fprintf(fp, "%s %lf %lf %lf %lf\n", pt_tmp.ID2, pt_tmp.x2, pt_tmp.y2, deltax, deltay);
// 		}
// 	}

	return 1;
}


int  CImageFeatureMatching::FindConnectPt(CorrespondencePts pt, MchPtInfor *mchPairs, int pairNum, bool **ifprocessed, vector<CorrespondencePts> &pt_array)
{
	char *ptID1 = pt.ID1, *ptID2 = pt.ID2;
	int i, j;
	bool ifrepeat;
	for (i = 0; i < pairNum;i++)
	{
		char *pairID1, *pairID2;
		pairID1 = mchPairs[i].ID1;
		pairID2 = mchPairs[i].ID2;

		//first senario
		if (strcmp(ptID1,pairID1)==0 && strcmp(ptID2, pairID2)!=0)
		{
			for (j = 0; j < mchPairs[i].ptNum;j++)
			{
				if (ifprocessed[i][j]==false)
				{
					CorrespondencePts pt_iter;
					strcpy(pt_iter.ID1, pairID1);
					strcpy(pt_iter.ID2, pairID2);

					pt_iter.x1 = mchPairs[i].corresList[j].x1;
					pt_iter.y1 = mchPairs[i].corresList[j].y1;
					pt_iter.x2 = mchPairs[i].corresList[j].x2;
					pt_iter.y2 = mchPairs[i].corresList[j].y2;

					if (fabs(pt.x1 - pt_iter.x1)<0.5 && fabs(pt.y1 - pt_iter.y1)<0.5)
					{
						ChechRepeatofPairID(pt_iter.ID1, pt_iter.ID2, pt_array, ifrepeat);
						if (ifrepeat == false)
						{
							pt_array.push_back(pt_iter);
							ifprocessed[i][j] = true;
						}
					}
				}
			}
		}

		//second scenario
		if (strcmp(ptID1, pairID2)==0 && strcmp(ptID2, pairID1)!=0)
		{
			for (j = 0; j < mchPairs[i].ptNum; j++)
			{
				if (ifprocessed[i][j] == false)
				{
					CorrespondencePts pt_iter;
					strcpy(pt_iter.ID1, pairID1);
					strcpy(pt_iter.ID2, pairID2);

					pt_iter.x1 = mchPairs[i].corresList[j].x1;
					pt_iter.y1 = mchPairs[i].corresList[j].y1;
					pt_iter.x2 = mchPairs[i].corresList[j].x2;
					pt_iter.y2 = mchPairs[i].corresList[j].y2;

					if (fabs(pt.x1 - pt_iter.x2)<0.5 && fabs(pt.y1 - pt_iter.y2)<0.5)
					{
						ChechRepeatofPairID(pt_iter.ID1, pt_iter.ID2, pt_array, ifrepeat);
						if (ifrepeat == false)
						{
							pt_array.push_back(pt_iter);
							ifprocessed[i][j] = true;
						}
					}
				}
			}
		}

		//third scenario
		if (strcmp(ptID1, pairID2)!=0 && strcmp(ptID2, pairID1)==0)
		{
			for (j = 0; j < mchPairs[i].ptNum; j++)
			{
				if (ifprocessed[i][j] == false)
				{
					CorrespondencePts pt_iter;
					strcpy(pt_iter.ID1, pairID1);
					strcpy(pt_iter.ID2, pairID2);

					pt_iter.x1 = mchPairs[i].corresList[j].x1;
					pt_iter.y1 = mchPairs[i].corresList[j].y1;
					pt_iter.x2 = mchPairs[i].corresList[j].x2;
					pt_iter.y2 = mchPairs[i].corresList[j].y2;

					if (fabs(pt.x2 - pt_iter.x1)<0.5 && fabs(pt.y2 - pt_iter.y1)<0.5)
					{
						ChechRepeatofPairID(pt_iter.ID1, pt_iter.ID2, pt_array, ifrepeat);
						if (ifrepeat == false)
						{
							pt_array.push_back(pt_iter);
							ifprocessed[i][j] = true;
						}
					}
				}
			}
		}

		//fourth scenario
		if (strcmp(ptID1, pairID1)!=0 && strcmp(ptID2, pairID2)==0)
		{
			for (j = 0; j < mchPairs[i].ptNum; j++)
			{
				if (ifprocessed[i][j] == false)
				{
					CorrespondencePts pt_iter;
					strcpy(pt_iter.ID1, pairID1);
					strcpy(pt_iter.ID2, pairID2);

					pt_iter.x1 = mchPairs[i].corresList[j].x1;
					pt_iter.y1 = mchPairs[i].corresList[j].y1;
					pt_iter.x2 = mchPairs[i].corresList[j].x2;
					pt_iter.y2 = mchPairs[i].corresList[j].y2;

					if (fabs(pt.x2 - pt_iter.x2)<0.5 && fabs(pt.y2 - pt_iter.y2)<0.5)
					{
						ChechRepeatofPairID(pt_iter.ID1, pt_iter.ID2, pt_array, ifrepeat);
						if (ifrepeat == false)
						{
							pt_array.push_back(pt_iter);
							ifprocessed[i][j] = true;
						}
					}
				}
			}
		}
	}
	return 1;
}

int CImageFeatureMatching::ChechRepeatofPairID(char *ID1, char *ID2, vector<CorrespondencePts> pts, bool &ifRepeat)
{
	ifRepeat = false;
	int i;
	int size = pts.size();
	for (i = 0; i < size;i++)
	{
		char *pairID1, *pairID2;
		pairID1 = pts[i].ID1;
		pairID2 = pts[i].ID2;

		if (strcmp(ID1, pairID1)==0 && strcmp(ID2, pairID2)==0)
		{
			ifRepeat = true;
			break;
		}
	}
	return 1;
}

int CImageFeatureMatching::Multi_view_ForwardIntersection(vector<CorrespondencePts> pt_array, Long_Strip_Img *strips, int StripNum,
	double &Lat, double &Lon, double &hei)
{
	CRPC_Fun rpcf;

	int totalNum = pt_array.size();
	int i;
	double *lat_array, *lon_array, *hei_array;
	lat_array = new double[totalNum];
	lon_array = new double[totalNum];
	hei_array = new double[totalNum];
	for (i = 0; i < totalNum; i++)
	{
		char *ID1, *ID2;
		double x1, y1, x2, y2;
		ID1 = pt_array[i].ID1;
		x1 = pt_array[i].x1;  y1 = pt_array[i].y1;
		ID2 = pt_array[i].ID2;
		x2 = pt_array[i].x2; y2 = pt_array[i].y2;

		int stripID1, stripID2;
		int patchID1, patchID2;
		GetStripPatchID(ID1, stripID1, patchID1);
		GetStripPatchID(ID2, stripID2, patchID2);

		RPC rpc1, rpc2;
		rpcf.ReadRpc_singleFile(strips[stripID1].rpb_paths[patchID1], rpc1);
		rpcf.ReadRpc_singleFile(strips[stripID2].rpb_paths[patchID2], rpc2);

		//compute ground points
		double tLat, tLon, tHei;
		rpcf.RPC_ForwardIntersection(rpc1, rpc2, x1, y1, x2, y2, tLat, tLon, tHei);

		lat_array[i] = tLat;
		lon_array[i] = tLon;
		hei_array[i] = tHei;
	}

	//output results
	Lat = 0;
	Lon = 0;
	hei = 0;
	int k = 0;
	for (i = 0; i < totalNum; i++)
	{
		Lat += lat_array[i];
		Lon += lon_array[i];
		hei += hei_array[i];
	}

	Lat /= totalNum;
	Lon /= totalNum;
	hei /= totalNum;

	//free memory
	delete[]lat_array; lat_array = NULL;
	delete[]lon_array; lon_array = NULL;
	delete[]hei_array; hei_array = NULL;

	return 1;
}

// 	int i, j, k;
// 	int pairNum = 0;
// 	MchPtInfor *mchPairs;
// 	ReadStereoListInfor(stereo_pt_list, mchPairs, pairNum);
// 
// 	//Read rpc
// 	RPC *rpcs;
// 	ReadRpcInfo(rpc_path, imgNum, rpcs);
// 
// 	//read tfw;
// 	TfwPara *tfws;
// 	tfws = new TfwPara[imgNum];
// 	for (i = 0; i < imgNum;i++)
// 	{
// 		ReadTfwInforFromList(rectif_list_path, i, tfws[i]);
// 	}
// 
// 	bool **ifprocessed;
// 	ifprocessed = new bool *[pairNum];
// 	for (i = 0; i < pairNum;i++)
// 	{
// 		ifprocessed[i] = new bool[mchPairs[i].ptNum];
// 		memset(ifprocessed[i], 0, sizeof(bool)*mchPairs[i].ptNum);
// 	}
// 
// 	bool **ifprocessed_backup;
// 	ifprocessed_backup = new bool *[pairNum];
// 	for (i = 0; i < pairNum; i++)
// 	{
// 		ifprocessed_backup[i] = new bool[mchPairs[i].ptNum];
// 		memset(ifprocessed_backup[i], 0, sizeof(bool)*mchPairs[i].ptNum);
// 	}
// 
// 	FILE *fp_output = fopen(output_list, "w");
// 	for (i = 0; i < pairNum;i++)
// 	{
// 		int cur_ptNum = mchPairs[i].ptNum;
// 		int ID1, ID2;
// 		ID1 = mchPairs[i].ID1;
// 		ID2 = mchPairs[i].ID2;
// 		vector <CorrespondencePts> pt_array;
// 		for (j = 0; j < cur_ptNum;j++)
// 		{
// 			if (ifprocessed[i][j]==false) //not processed before
// 			{
// 				//backup for the ifprocessed
// 				for (k = 0; k < pairNum; k++)
// 				{
// 					memcpy(ifprocessed_backup[k], ifprocessed[k], sizeof(bool)*mchPairs[k].ptNum);
// 				}
// 
// 				CorrespondencePts cur_pt;
// 				cur_pt.ID1 = ID1;
// 				cur_pt.ID2 = ID2;
// 				cur_pt.x1 = mchPairs[i].corresList[j].x1;
// 				cur_pt.y1 = mchPairs[i].corresList[j].y1;
// 				cur_pt.x2 = mchPairs[i].corresList[j].x2;
// 				cur_pt.y2 = mchPairs[i].corresList[j].y2;
// 
// 				pt_array.push_back(cur_pt);
// 				ifprocessed[i][j] = true;
// 				int iter = 0;
// 				while (pt_array.size()>iter)
// 				{
// 					cur_pt = pt_array[iter];
// 
// 					FindConnectPt(cur_pt, mchPairs, pairNum, ifprocessed, pt_array);
// 
// 					iter++;
// 				}
// 
// 				//remove outliers and compute the corresponding ground point
// 				vector<CorrespondencePts> new_pt_array;
// 				double Lat, Lon, Hei;
// 				ComputeGroundPtsFromMultiMchPts(pt_array, rpcs, tfws, levelID,
// 					new_pt_array, Lat, Lon, Hei, mchPairs, pairNum, ifprocessed_backup);
// 
// 				if (pt_array.size()==new_pt_array.size()) //totally correct matches
// 				{
// 					//output result
// 					int size = new_pt_array.size();
// 					OutputTrackingPtsResults(fp_output, new_pt_array, Lat, Lon, Hei);
// 
// 					//free memory
// 					pt_array.clear();
// 					vector<CorrespondencePts>().swap(pt_array);
// 					new_pt_array.clear();
// 					vector<CorrespondencePts>().swap(new_pt_array);
// 				}
// 				else //exist outliers
// 				{
// 					for (k = 0; k < pairNum; k++)
// 					{
// 						memcpy(ifprocessed[k], ifprocessed_backup[k], sizeof(bool)*mchPairs[k].ptNum);
// 					}
// 
// 					//free memory
// 					pt_array.clear();
// 					vector<CorrespondencePts>().swap(pt_array);
// 					new_pt_array.clear();
// 					vector<CorrespondencePts>().swap(new_pt_array);
// 
// 					//check the valid of the first pt
// 					if (ifprocessed[i][j] == true)
// 					{
// 						continue;
// 					}
// 
// 					cur_pt;
// 					cur_pt.ID1 = ID1;
// 					cur_pt.ID2 = ID2;
// 					cur_pt.x1 = mchPairs[i].corresList[j].x1;
// 					cur_pt.y1 = mchPairs[i].corresList[j].y1;
// 					cur_pt.x2 = mchPairs[i].corresList[j].x2;
// 					cur_pt.y2 = mchPairs[i].corresList[j].y2;
// 
// 					pt_array.push_back(cur_pt);
// 					ifprocessed[i][j] = true;
// 					int iter = 0;
// 					while (pt_array.size() > iter)
// 					{
// 						cur_pt = pt_array[iter];
// 
// 						FindConnectPt(cur_pt, mchPairs, pairNum, ifprocessed, pt_array);
// 
// 						iter++;
// 					}
// 
// 					//This function should be improved as the purely multi-view forward intersection
// // 					ComputeGroundPtsFromMultiMchPts(pt_array, rpcs, tfws, levelID,
// // 						new_pt_array, Lat, Lon, Hei, mchPairs, pairNum, ifprocessed_backup);
// 					Multi_view_ForwardIntersection(pt_array, rpcs, tfws, levelID,
// 						Lat, Lon, Hei);
// 
// 					OutputTrackingPtsResults(fp_output, pt_array, Lat, Lon, Hei);
// 
// 					//free memory
// 					pt_array.clear();
// 					vector<CorrespondencePts>().swap(pt_array);
// 				}
// 			}
// 		}
// 	}
// 
// 	//free memory
// 	fclose(fp_output); fp_output = NULL;
// 	for (i = 0; i < pairNum;i++)
// 	{
// 		delete[]mchPairs[i].corresList; mchPairs[i].corresList = NULL;
// 		delete[]ifprocessed[i]; ifprocessed[i] = NULL;
// 		delete[]ifprocessed_backup[i]; ifprocessed_backup[i] = NULL;
// 	}
// 	delete[]tfws; tfws = NULL;
// 	delete[]mchPairs; mchPairs = NULL;
// 	delete[]ifprocessed; ifprocessed = NULL;
// 	delete[]ifprocessed_backup; ifprocessed_backup = NULL;
// 	delete[]rpcs; rpcs = NULL;

// int CImageFeatureMatching::Feature_matching_TopLevel(char *Pyr_img_list, char *Feature_pt_list, char **rpc_path, int satNum, 
// 	int **stereolist, int *stereoNum, char *rectif_list_path, char *matching_pt_path, char *stereo_list_path)
// {
// 	int i, j;
// 	int pyrLevelID = pyrLevel - 1;
// 	//Read feature point list path
// 	FILE *fp_pt_list = fopen(Feature_pt_list, "r");
// 	int imgNum, pyrlevel = 0;
// 	fscanf(fp_pt_list, "%d %d", &imgNum, &pyrlevel);
// 	char **feature_path;
// 	feature_path = new char *[imgNum];
// 	for (i = 0; i < imgNum;i++)
// 	{
// 		feature_path[i] = new char[MaxLen];
// 	}
// 	for (i = 0; i < imgNum;i++)
// 	{
// 		for (j = 0; j < pyrlevel - 1; j++)
// 		{
// 			fscanf(fp_pt_list, "%*s");
// 		}
// 		fscanf(fp_pt_list, "%s", feature_path[i]);
// 	}
// 	
// 	//Read pyramid image list path
// 	FILE *fp_pyr_img = fopen(Pyr_img_list, "r");
// 	fscanf(fp_pyr_img, "%*d %*d %*d");
// 	char **img_toplevel_path;
// 	img_toplevel_path = new char *[imgNum];
// 	for (i = 0; i < imgNum; i++)
// 	{
// 		img_toplevel_path[i] = new char[MaxLen];
// 	}
// 	for (i = 0; i < imgNum; i++)
// 	{
// 		for (j = 0; j < pyrlevel - 1; j++)
// 		{
// 			fscanf(fp_pyr_img, "%*s");
// 		}
// 		fscanf(fp_pyr_img, "%s", img_toplevel_path[i]);
// 	}
// 
// 	//Pair matching
// 	FILE *fp_output_list = fopen(stereo_list_path, "w");
// 	for (i = 0; i < satNum; i++)
// 	{
// 		RPC rpc_i;
// 		ReadRpc_singleFile(rpc_path[i], rpc_i);
// 		for (j = 0; j < stereoNum[i];j++)
// 		{
// 			if (stereolist[i][j]<=i)
// 			{
// 				continue;
// 			}
// 			RPC rpc_j;
// 			int ID_j = stereolist[i][j];
// 			ReadRpc_singleFile(rpc_path[ID_j], rpc_j);
// 
// 			//txt to storage correspondence of i and j
// 			char MatchPt_name[MaxLen];
// 			sprintf(MatchPt_name, "%d_%d.txt", i, ID_j);
// 			char MatchPt_path[MaxLen];
// 			strcpy(MatchPt_path, matching_pt_path);
// 			strcat(MatchPt_path, MatchPt_name);
// 
// 			//Get Feature points of i
// 			int feaNum_i = 0;
// 			float *fx_i = NULL, *fy_i = NULL;
// /*			float *orifx_i = NULL, *orify_i = NULL;*/
// 			GetFeaturePoints(feature_path[i], fx_i, fy_i, feaNum_i);//orifx_i, orify_i, 
// 
// 			int feaNum_j = 0;
// 			float *fx_j = NULL, *fy_j = NULL;
// /*			float *orifx_j = NULL, *orify_j = NULL;*/
// 			GetFeaturePoints(feature_path[ID_j], fx_j, fy_j, feaNum_j);//orifx_j, orify_j, 
// 
// 			//Get tfw information
// 			TfwPara tfw_i, tfw_j;
// 			ReadTfwInforFromList(rectif_list_path, i, tfw_i);
// 			ReadTfwInforFromList(rectif_list_path, ID_j, tfw_j);
// 
// 			//Matching
// 			vector<double> xi, yi, xj, yj;
// 			PairMatching(pyrLevelID, img_toplevel_path[i], img_toplevel_path[ID_j], rpc_i, rpc_j, tfw_i, tfw_j,
// 				fx_i, fy_i, feaNum_i,
// 				fx_j, fy_j, feaNum_j,
// 				xi, yi, xj, yj);
// 
// 			//output Correspondences
// 			int CorrespondenceNum = xi.size();
// 			if (CorrespondenceNum>MinMchNum)
// 			{
// 				//output the pt file in the list
// 				fprintf(fp_output_list, "%s\n", MatchPt_path);
// 
// 				//compute the correspondences in the original image
// 				double gX_i, gY_i, gZ_i;
// 				double gX_j, gY_j, gZ_j;
// 				gZ_i = avgH;
// 				gZ_j = avgH;
// 				
// 				int pyrsize = pyrSize;
// 				double scale = powl(pyrsize, pyrlevel - 1);
// 
// 				//write the pt file
// 				FILE *fp_out = fopen(MatchPt_path, "w");
// 				int t;
// 				for (t = 0; t < CorrespondenceNum; t++)
// 				{
// 					gX_i = scale*tfw_i.A*xi[t] + scale*tfw_i.B*yi[t] + tfw_i.C;
// 					gY_i = scale*tfw_i.D*xi[t] + scale*tfw_i.E*yi[t] + tfw_i.F;
// 
// 					gX_j = scale*tfw_j.A*xj[t] + scale*tfw_j.B*yj[t] + tfw_j.C;
// 					gY_j = scale*tfw_j.D*xj[t] + scale*tfw_j.E*yj[t] + tfw_j.F;
// 
// 					//UTM to LLH
// 					double Lat_i, Lon_i, Lat_j, Lon_j;
// 					UTM2LL(gX_i, gY_i, m_zone, Lat_i, Lon_i);
// 					UTM2LL(gX_j, gY_j, m_zone, Lat_j, Lon_j);
// 
// 					double ori_x_i, ori_y_i;
// 					double ori_x_j, ori_y_j;
// 					RPC_Ground2Image(Lat_i, Lon_i, gZ_i, rpc_i, ori_y_i, ori_x_i);
// 					RPC_Ground2Image(Lat_j, Lon_j, gZ_j, rpc_j, ori_y_j, ori_x_j);
// 
// 					//compute the corresponding ground point
// 
// 
// 					ori_y_i /= scale;
// 					ori_x_i /= scale;
// 					ori_y_j /= scale;
// 					ori_x_j /= scale;
// 
// 					fprintf(fp_out, "%lf %lf %lf %lf\n", xi[t], yi[t], xj[t], yj[t]/*, ori_x_i, ori_y_i, ori_x_j, ori_y_j*/); // %lf %lf %lf %lf
// 				}
// 				fclose(fp_out); fp_out = NULL;
// 			}
// 
// 			//free memeroy
// 			delete[]fx_i; fx_i = NULL;
// 			delete[]fy_i; fy_i = NULL;
// 			delete[]fx_j; fx_j = NULL;
// 			delete[]fy_j; fy_j = NULL;
// // 			delete[]orifx_i; orifx_i = NULL;
// // 			delete[]orify_i; orify_i = NULL;
// // 			delete[]orifx_j; orifx_j = NULL;
// // 			delete[]orify_j; orify_j = NULL;
// 			xi.clear(); yi.clear();
// 			xj.clear(); yj.clear();
// 			vector<double>().swap(xi);
// 			vector<double>().swap(yi);
// 			vector<double>().swap(xj);
// 			vector<double>().swap(yj);
// 		}
// 	}
// 
// 	//free memory
// 	fclose(fp_output_list); fp_output_list = NULL;
// 	for (i = 0; i < imgNum;i++)
// 	{
// 		delete[]feature_path[i]; 
// 		delete[]img_toplevel_path[i];
// 	}
// 	delete[]feature_path; feature_path = NULL;
// 	delete[]img_toplevel_path; img_toplevel_path = NULL;
// 	fclose(fp_pt_list); fp_pt_list = NULL;
// 	fclose(fp_pyr_img); fp_pyr_img = NULL;
// 	return 1;
// }
// 