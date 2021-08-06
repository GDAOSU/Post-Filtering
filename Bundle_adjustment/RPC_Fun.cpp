#include "stdafx.h"
#include "RPC_Fun.h"
#include "imagebase.h"
#include "Simple_Matrix_Computation.h"
#include "OtherFuns.h"
#include "LL_UTM.h"
#include "ImageProcess.h"
#include <vector>
using namespace std;

#define MaxLen 512

CRPC_Fun::CRPC_Fun()
{
	m_reprojection_thresh = 0.5;
	m_virtual_GCP_plan_size = 50;
	m_virtual_GCP_height_size = 20;
	strcpy(m_basis_ID, "0_0");
}


CRPC_Fun::~CRPC_Fun()
{
}

int CRPC_Fun::ReadRpc_txt(char *rpcPath, RPC &rpcs)
{
	FILE *fp = fopen(rpcPath, "r");

	strcpy(rpcs.satID, "");
	strcpy(rpcs.bandID, "");
	strcpy(rpcs.SpecId, "");
	strcpy(rpcs.BEGIN_GROUP, "");
	strcpy(rpcs.errBias, "");
	strcpy(rpcs.errRand, "");

	char str[255];
	fgets(str, 255, fp);
	sscanf(str, "LINE_OFF: %lf pixels", &rpcs.lineOffset);

	fgets(str, 255, fp);
	sscanf(str, "SAMP_OFF: %lf pixels", &rpcs.sampOffset);

	fgets(str, 255, fp);
	sscanf(str, "LAT_OFF: %lf degrees", &rpcs.latOffset);

	fgets(str, 255, fp);
	sscanf(str, "LONG_OFF: %lf degrees", &rpcs.longOffset);

	fgets(str, 255, fp);
	sscanf(str, "HEIGHT_OFF: %lf meters", &rpcs.heightOffset);

	fgets(str, 255, fp);
	sscanf(str, "LINE_SCALE: %lf pixels", &rpcs.lineScale);

	fgets(str, 255, fp);
	sscanf(str, "SAMP_SCALE: %lf pixels", &rpcs.sampScale);

	fgets(str, 255, fp);
	sscanf(str, "LAT_SCALE: %lf degrees", &rpcs.latScale);

	fgets(str, 255, fp);
	sscanf(str, "LONG_SCALE: %lf degrees", &rpcs.longScale);

	fgets(str, 255, fp);
	sscanf(str, "HEIGHT_SCALE: %lf meters", &rpcs.heightScale);

	int j;
	for (j = 0; j < 20; j++)
	{
		fgets(str, 255, fp);
		sscanf(str, "%*s %lf", &rpcs.lineNumCoef[j]);
	}

	for (j = 0; j < 20; j++)
	{
		fgets(str, 255, fp);
		sscanf(str, "%*s %lf", &rpcs.lineDenCoef[j]);
	}

	for (j = 0; j < 20; j++)
	{
		fgets(str, 255, fp);
		sscanf(str, "%*s %lf", &rpcs.sampNumCoef[j]);
	}

	for (j = 0; j < 20; j++)
	{
		fgets(str, 255, fp);
		sscanf(str, "%*s %lf", &rpcs.sampDenCoef[j]);
	}

	fclose(fp);

	return 1;
}

int CRPC_Fun::ReadRpc_singleFile(char *rpcPath, RPC &rpcs)
{
	FILE *fp = fopen(rpcPath, "r");

	char str[255];
	fgets(str, 255, fp);
	str[strlen(str)-1] = NULL;
	strcpy(rpcs.satID, str);

	fgets(str, 255, fp);
	str[strlen(str) - 1] = NULL;
	strcpy(rpcs.bandID, str);

	fgets(str, 255, fp);
	str[strlen(str) - 1] = NULL;
	strcpy(rpcs.SpecId, str);

	fgets(str, 255, fp);
	str[strlen(str) - 1] = NULL;
	strcpy(rpcs.BEGIN_GROUP, str);

	fgets(str, 255, fp);
	str[strlen(str) - 1] = NULL;
	strcpy(rpcs.errBias, str);

	fgets(str, 255, fp);
	str[strlen(str) - 1] = NULL;
	strcpy(rpcs.errRand, str);

	fgets(str, 255, fp);
	sscanf(str, "\tlineOffset = %lf;\n", &rpcs.lineOffset);

	fgets(str, 255, fp);
	sscanf(str, "\tsampOffset = %lf;\n", &rpcs.sampOffset);

	fgets(str, 255, fp);
	sscanf(str, "\tlatOffset = %lf;\n", &rpcs.latOffset);

	fgets(str, 255, fp);
	sscanf(str, "\tlongOffset = %lf;\n", &rpcs.longOffset);

	fgets(str, 255, fp);
	sscanf(str, "\theightOffset = %lf;\n", &rpcs.heightOffset);

	fgets(str, 255, fp);
	sscanf(str, "\tlineScale = %lf;\n", &rpcs.lineScale);

	fgets(str, 255, fp);
	sscanf(str, "\tsampScale = %lf;\n", &rpcs.sampScale);

	fgets(str, 255, fp);
	sscanf(str, "\tlatScale = %lf;\n", &rpcs.latScale);

	fgets(str, 255, fp);
	sscanf(str, "\tlongScale = %lf;\n", &rpcs.longScale);

	fgets(str, 255, fp);
	sscanf(str, "\theightScale = %lf;\n", &rpcs.heightScale);

	fgets(str, 255, fp);
	int j;
	for (j = 0; j < 20; j++)
	{
		fgets(str, 255, fp);
		sscanf(str, "%lf,\n", &rpcs.lineNumCoef[j]);
	}

	fgets(str, 255, fp);
	for (j = 0; j < 20; j++)
	{
		fgets(str, 255, fp);
		sscanf(str, "%lf,\n", &rpcs.lineDenCoef[j]);
	}

	fgets(str, 255, fp);
	for (j = 0; j < 20; j++)
	{
		fgets(str, 255, fp);
		sscanf(str, "%lf,\n", &rpcs.sampNumCoef[j]);
	}

	fgets(str, 255, fp);
	for (j = 0; j < 20; j++)
	{
		fgets(str, 255, fp);
		sscanf(str, "%lf,\n", &rpcs.sampDenCoef[j]);
	}

	fclose(fp); fp = NULL;


	return 1;
}

int CRPC_Fun::WriteRPBFile(RPC rpc, char *outputPath)
{
	FILE *fp_out = fopen(outputPath, "w");

	fprintf(fp_out, "%s\n", rpc.satID);
	fprintf(fp_out, "%s\n", rpc.bandID);
	fprintf(fp_out, "%s\n", rpc.SpecId);
	fprintf(fp_out, "%s\n", rpc.BEGIN_GROUP);
	fprintf(fp_out, "%s\n", rpc.errBias);
	fprintf(fp_out, "%s\n", rpc.errRand);

	fprintf(fp_out, "\tlineOffset = %lf;\n", rpc.lineOffset);

	fprintf(fp_out, "\tsampOffset = %lf;\n", rpc.sampOffset);

	fprintf(fp_out, "\tlatOffset = %lf;\n", rpc.latOffset);

	fprintf(fp_out, "\tlongOffset = %lf;\n", rpc.longOffset);

	fprintf(fp_out, "\theightOffset = %lf;\n", rpc.heightOffset);

	fprintf(fp_out, "\tlineScale = %lf;\n", rpc.lineScale);

	fprintf(fp_out, "\tsampScale = %lf;\n", rpc.sampScale);

	fprintf(fp_out, "\tlatScale = %lf;\n", rpc.latScale);

	fprintf(fp_out, "\tlongScale = %lf;\n", rpc.longScale);

	fprintf(fp_out, "\theightScale = %lf;\n", rpc.heightScale);

	fprintf(fp_out, "\t%s\n", "lineNumCoef = (");
	int j;
	for (j = 0; j < 19; j++)
	{
		fprintf(fp_out, "			%e,\n", rpc.lineNumCoef[j]);
	}
	fprintf(fp_out, "			%e);\n", rpc.lineNumCoef[j]);

	fprintf(fp_out, "\t%s\n", "lineDenCoef = (");
	for (j = 0; j < 19; j++)
	{
		fprintf(fp_out, "			%e,\n", rpc.lineDenCoef[j]);
	}
	fprintf(fp_out, "			%e);\n", rpc.lineDenCoef[j]);

	fprintf(fp_out, "\t%s\n", "sampNumCoef = (");
	for (j = 0; j < 19; j++)
	{
		fprintf(fp_out, "			%e,\n", rpc.sampNumCoef[j]);
	}
	fprintf(fp_out, "			%e);\n", rpc.sampNumCoef[j]);

	fprintf(fp_out, "\t%s\n", "sampDenCoef = (");
	for (j = 0; j < 19; j++)
	{
		fprintf(fp_out, "			%e,\n", rpc.sampDenCoef[j]);
	}
	fprintf(fp_out, "			%e);\n", rpc.sampDenCoef[j]);

	fprintf(fp_out, "%s\n", "END_GROUP = IMAGE");
	fprintf(fp_out, "%s\n", "END;");

	fclose(fp_out); fp_out = NULL;

	return 1;
}

int CRPC_Fun::ReadRpcInfo(char **rpcPath, int satNum, RPC *& rpcs)
{
	int i, j;
	rpcs = new RPC[satNum];
	for (i = 0; i < satNum;i++)
	{
		FILE *fp = fopen(rpcPath[i], "r");

		char str[255];
		fgets(str, 255, fp);
		fgets(str, 255, fp);
		fgets(str, 255, fp);
		fgets(str, 255, fp);
		fgets(str, 255, fp);
		fgets(str, 255, fp);

		fgets(str, 255, fp);
		sscanf(str, "\tlineOffset = %lf;\n", &rpcs[i].lineOffset);

		fgets(str, 255, fp);
		sscanf(str, "\tsampOffset = %lf;\n", &rpcs[i].sampOffset);

		fgets(str, 255, fp);
		sscanf(str, "\tlatOffset = %lf;\n", &rpcs[i].latOffset);

		fgets(str, 255, fp);
		sscanf(str, "\tlongOffset = %lf;\n", &rpcs[i].longOffset);

		fgets(str, 255, fp);
		sscanf(str, "\theightOffset = %lf;\n", &rpcs[i].heightOffset);

		fgets(str, 255, fp);
		sscanf(str, "\tlineScale = %lf;\n", &rpcs[i].lineScale);

		fgets(str, 255, fp);
		sscanf(str, "\tsampScale = %lf;\n", &rpcs[i].sampScale);

		fgets(str, 255, fp);
		sscanf(str, "\tlatScale = %lf;\n", &rpcs[i].latScale);

		fgets(str, 255, fp);
		sscanf(str, "\tlongScale = %lf;\n", &rpcs[i].longScale);

		fgets(str, 255, fp);
		sscanf(str, "\theightScale = %lf;\n", &rpcs[i].heightScale);

		fgets(str, 255, fp);
		for (j = 0; j < 20;j++)
		{
			fgets(str, 255, fp);
			sscanf(str, "%lf,\n", &rpcs[i].lineNumCoef[j]);
		}

		fgets(str, 255, fp);
		for (j = 0; j < 20; j++)
		{
			fgets(str, 255, fp);
			sscanf(str, "%lf,\n", &rpcs[i].lineDenCoef[j]);
		}

		fgets(str, 255, fp);
		for (j = 0; j < 20; j++)
		{
			fgets(str, 255, fp);
			sscanf(str, "%lf,\n", &rpcs[i].sampNumCoef[j]);
		}

		fgets(str, 255, fp);
		for (j = 0; j < 20; j++)
		{
			fgets(str, 255, fp);
			sscanf(str, "%lf,\n", &rpcs[i].sampDenCoef[j]);
		}

		fclose(fp); fp = NULL;
	}
	return 1;
}

int CRPC_Fun::Check_Patch_Coverage_In_SameStrip(char *imgPaths_cur, char *rpbPath_cur, char *imgPaths_nxt, char *rpbPath_nxt)
{
	//Read rpc file
	RPC rpc_cur, rpc_nxt;
	ReadRpc_singleFile(rpbPath_cur, rpc_cur);
	ReadRpc_singleFile(rpbPath_nxt, rpc_nxt);

	//Read image file
	int w_cur, h_cur, w_nxt, h_nxt;
	CImageBase img;
	img.Open(imgPaths_cur);
	w_cur = img.GetCols();
	h_cur = img.GetRows();
	img.Close();

	img.Open(imgPaths_nxt);
	w_nxt = img.GetCols();
	h_nxt = img.GetRows();
	img.Close();

	//processing
	//projection from current image
	double Hei = 0;
	double Lat[4], Lon[4];
	double xc[4] = { 0, w_cur - 1, w_cur - 1, 0 };
	double yc[4] = { 0, 0, h_cur - 1, h_cur - 1 };

	for (int i = 0; i < 4;i++)
	{
		RPC_Image2Ground(yc[i], xc[i], Hei, rpc_cur, Lat[i], Lon[i]);
	}

	//back-projection to the next image
	double x_nxt[4], y_nxt[4];
	for (int i = 0; i < 4;i++)
	{
		RPC_Ground2Image(Lat[i], Lon[i], Hei, rpc_nxt, y_nxt[i], x_nxt[i]);
	}

	int px_nxt[4], py_nxt[4];
	for (int i = 0; i < 4;i++)
	{
		px_nxt[i] = (int)(x_nxt[i] + 0.5);
		py_nxt[i] = (int)(y_nxt[i] + 0.5);
	}

	bool ifbecovered = true;
	for (int i = 0; i < 4;i++)
	{
		if (y_nxt[i]<0 || y_nxt[i]>=h_nxt)
		{
			ifbecovered = false;
			break;
		}
	}

	if (ifbecovered == true)
	{
		return 0;
	}
	else
		return 1;
	
	return 1;
}

int CRPC_Fun::Check_AOI_Overlap(char *imgPaths, char *rpcPaths, double *lat, double *lon, double *hei, int extra_size,
	int &min_px, int &min_py, int &max_px, int &max_py, bool &ifcover)
{
	//read image
	CImageBase img;
	int imgw, imgh;
	img.Open(imgPaths);
	imgw = img.GetCols();
	imgh = img.GetRows();
	img.Close();

	//read rpc path
	RPC rpc;
	ReadRpc_singleFile(rpcPaths, rpc);

	//there are four relationships between aoi and images
	//1. the images contain the aoi
	//2. aoi contain the image
	//3. they are intersected
	//4. they are separated

	//Project ground point onto images
	//////////////////////////////////////notice: I should test the height plane as default or from rpc file
	double *row, *col;
	int nodeNum = 4;
	row = new double[nodeNum];
	col = new double[nodeNum];
	for (int i = 0; i < nodeNum; i++)
	{
		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc, row[i], col[i]);
	}

	ifcover = false;
	int InImgNum = 0;
	for (int i = 0; i < nodeNum; i++)
	{
		if (row[i] >= 0 && row[i] < imgh - 0.5 && col[i] >= 0 && col[i] < imgw - 0.5)
		{
			InImgNum++;
		}
	}

	//project image to ground
	double ix[4] = { 0, imgw - 1, imgw - 1, 0 };
	double iy[4] = { 0, 0, imgh - 1, imgh - 1 };
	double avgHei = 0;
	double lat_img[4], lon_img[4];
	double lat_min = lat[0], lat_max = lat[0], lon_min = lon[0], lon_max = lon[0];
	for (int i = 0; i < nodeNum; i++)
	{
		if (lat_min > lat[i]) lat_min = lat[i];
		if (lat_max < lat[i]) lat_max = lat[i];
		if (lon_min > lon[i]) lon_min = lon[i];
		if (lon_max < lon[i]) lon_max = lon[i];

		avgHei += hei[i];
	}
	avgHei /= nodeNum;

	for (int i = 0; i < 4;i++)
	{
		RPC_Image2Ground(iy[i], ix[i], avgHei, rpc, lat_img[i], lon_img[i]);
	}

	double lat_img_min = lat_img[0], lat_img_max = lat_img[0], lon_img_min = lon_img[0], lon_img_max = lon_img[0];
	for (int i = 0; i < 4; i++)
	{
		if (lat_img_min > lat_img[i]) lat_img_min = lat_img[i];
		if (lat_img_max < lat_img[i]) lat_img_max = lat_img[i];
		if (lon_img_min > lon_img[i]) lon_img_min = lon_img[i];
		if (lon_img_max < lon_img[i]) lon_img_max = lon_img[i];
	}

	int InAOINum = 0;
	for (int i = 0; i < 4; i++)
	{
		if (lat_img[i]>=lat_min && lat_img[i]<=lat_max && lon_img[i]>=lon_min && lon_img[i]<=lon_max)
		{
			InAOINum++;
		}
	}
	
	if (InImgNum == nodeNum) //relationship 1
	{
		ifcover = true;
		double min_row, max_row, min_col, max_col;
		min_row = row[0]; max_row = row[0];
		min_col = col[0]; max_col = col[0];

		for (int i = 1; i < nodeNum; i++)
		{
			if (min_row > row[i])
			{
				min_row = row[i];
			}
			if (max_row < row[i])
			{
				max_row = row[i];
			}
			if (min_col > col[i])
			{
				min_col = col[i];
			}
			if (max_col < col[i])
			{
				max_col = col[i];
			}
		}

		min_col -= extra_size;
		min_row -= extra_size;
		max_col += extra_size;
		max_row += extra_size;

		min_px = (int)(min_col + 0.5);
		if (min_px < 0) min_px = 0;

		max_px = (int)(max_col + 0.5);
		if (max_px > imgw - 1) max_px = imgw - 1;

		min_py = (int)(min_row + 0.5);
		if (min_py < 0) min_py = 0;

		max_py = (int)(max_row + 0.5);
		if (max_py > imgh - 1) max_py = imgh - 1;

	}
	else if (InAOINum==4) //relationship 2
	{
		ifcover = true;
		min_px = 0;
		min_py = 0;
		max_px = imgw - 1;
		max_py = imgh - 1;
	}
	else
	{
		double lat_intersection[4], lon_intersection[4];
		if (AOI_Intersection(lat_min, lon_min, lat_max, lon_max, lat_img_min, lon_img_min, lat_img_max, lon_img_max,
			lat_intersection, lon_intersection))  //relationship 3
		{
			ifcover = true;
			double row_inters[4], col_inters[4];
			for (int i = 0; i < 4; i++)
			{
				RPC_Ground2Image(lat_intersection[i], lon_intersection[i], avgHei, rpc, row_inters[i], col_inters[i]);
			}

			double min_row, max_row, min_col, max_col;
			min_row = row_inters[0]; max_row = row_inters[0];
			min_col = col_inters[0]; max_col = col_inters[0];

			for (int i = 1; i < 4; i++)
			{
				if (min_row > row_inters[i])
				{
					min_row = row_inters[i];
				}
				if (max_row < row_inters[i])
				{
					max_row = row_inters[i];
				}
				if (min_col > col_inters[i])
				{
					min_col = col_inters[i];
				}
				if (max_col < col_inters[i])
				{
					max_col = col_inters[i];
				}
			}

			min_col -= extra_size;
			min_row -= extra_size;
			max_col += extra_size;
			max_row += extra_size;

			min_px = (int)(min_col + 0.5);
			if (min_px < 0) min_px = 0;

			max_px = (int)(max_col + 0.5);
			if (max_px > imgw - 1) max_px = imgw - 1;

			min_py = (int)(min_row + 0.5);
			if (min_py < 0) min_py = 0;

			max_py = (int)(max_row + 0.5);
			if (max_py > imgh - 1) max_py = imgh - 1;
		}
		else //relationship 4
		{
			ifcover = false;
			min_px = 0;
			min_py = 0;
			max_px = 0;
			max_py = 0;
		}
	}

	//free memory
	delete[]row; row = NULL;
	delete[]col; col = NULL;

	return 1;
}

int CRPC_Fun::RPC_coefficient(double P, double L, double H, int ID, double &result)
{
	if (ID<1 || ID>20)
	{
		return 0;
	}

	if (ID==1)
	{
		result = 1;
	}
	else if (ID==2)
	{
		result = L;
	}
	else if (ID==3)
	{
		result = P;
	}
	else if (ID==4)
	{
		result = H;
	}
	else if (ID == 5)
	{
		result = L*P;
	}
	else if (ID == 6)
	{
		result = L*H;
	}
	else if (ID == 7)
	{
		result = P*H;
	}
	else if (ID == 8)
	{
		result = L*L;
	}
	else if (ID == 9)
	{
		result = P*P;
	}
	else if (ID == 10)
	{
		result = H*H;
	}
	else if (ID == 11)
	{
		result = P*L*H;
	}
	else if (ID == 12)
	{
		result = L*L*L;
	}
	else if (ID == 13)
	{
		result = L*P*P;
	}
	else if (ID == 14)
	{
		result = L*H*H;
	}
	else if (ID == 15)
	{
		result = L*L*P;
	}
	else if (ID == 16)
	{
		result = P*P*P;
	}
	else if (ID == 17)
	{
		result = P*H*H;
	}
	else if (ID == 18)
	{
		result = L*L*H;
	}
	else if (ID == 19)
	{
		result = P*P*H;
	}
	else if (ID == 20)
	{
		result = H*H*H;
	}

	return 1;
}

int CRPC_Fun::RPC_Ground2Image_normalize(double P, double L, double H, RPC rpc, double &r, double &c)
{
	double line_num = 0, line_den = 0;
	double SAMP_num = 0, SAMP_den = 0;

	int i;
	for (i = 0; i < 20;i++)
	{
		double pi = 0;
		RPC_coefficient(P, L, H, i + 1, pi);

		line_num += rpc.lineNumCoef[i] * pi;
		line_den += rpc.lineDenCoef[i] * pi;

		SAMP_num += rpc.sampNumCoef[i] * pi;
		SAMP_den += rpc.sampDenCoef[i] * pi;
	}

	r = line_num / line_den;
	c = SAMP_num / SAMP_den;

	return 1;
}

int CRPC_Fun::RPC_Ground2Image(double Lat, double Lon, double Height, RPC rpc, double &row, double &column)
{
	double P, L, H, r, c;
	P = (Lat - rpc.latOffset) / rpc.latScale;
	L = (Lon - rpc.longOffset) / rpc.longScale;
	H = (Height - rpc.heightOffset) / rpc.heightScale;

	RPC_Ground2Image_normalize(P, L, H, rpc, r, c);

	row = r*rpc.lineScale + rpc.lineOffset;
	column = c*rpc.sampScale + rpc.sampOffset;

	return 1;
}

int CRPC_Fun::RPC_Image2Ground_normalize(double r, double c, double H, RPC rpc, double &P, double &L)
{
	int i;
	double thresh = 0.00000001;
	double P0, L0;
	P0 = 0; L0 = 0;

	double deltaX_scale[2] = { 0 };
	deltaX_scale[0] = 1; deltaX_scale[1] = 1;

	double Am[2*2], Xm[2], Lm[2];
	while (fabs(deltaX_scale[0])>thresh || fabs(deltaX_scale[1])>thresh)
	{
		//compute Lm
		double line_num = 0, line_den = 0;
		double samp_num = 0, samp_den = 0;
		for (i = 0; i < 20;i++)
		{
			double pi;
			RPC_coefficient(P0, L0, H, i + 1, pi);
			line_num += rpc.lineNumCoef[i] * pi;
			line_den += rpc.lineDenCoef[i] * pi;
			samp_num += rpc.sampNumCoef[i] * pi;
			samp_den += rpc.sampDenCoef[i] * pi;
		}
		Lm[0] = -(line_num - r*line_den);
		Lm[1] = -(samp_num - c*samp_den);

		//compute Am
		double line_num_der_P = 0, line_den_der_P = 0;
		double line_num_der_L = 0, line_den_der_L = 0;
		double samp_num_der_P = 0, samp_den_der_P = 0;
		double samp_num_der_L = 0, samp_den_der_L = 0;

		for (i = 0; i < 20; i++)
		{
			double pi;
			RPC_coefficient_derivative(P0, L0, H, i + 1, 'P', pi);
			line_num_der_P += rpc.lineNumCoef[i] * pi;
			line_den_der_P += rpc.lineDenCoef[i] * pi;
			samp_num_der_P += rpc.sampNumCoef[i] * pi;
			samp_den_der_P += rpc.sampDenCoef[i] * pi;

			RPC_coefficient_derivative(P0, L0, H, i + 1, 'L', pi);
			line_num_der_L += rpc.lineNumCoef[i] * pi;
			line_den_der_L += rpc.lineDenCoef[i] * pi;
			samp_num_der_L += rpc.sampNumCoef[i] * pi;
			samp_den_der_L += rpc.sampDenCoef[i] * pi;
		}
		Am[0] = line_num_der_P - r*line_den_der_P;
		Am[1] = line_num_der_L - r*line_den_der_L;
		Am[2] = samp_num_der_P - c*samp_den_der_P;
		Am[3] = samp_num_der_L - c*samp_den_der_L;

		//compute Xm
		Xm[0] = -(Am[1] * Lm[1] - Am[3] * Lm[0]) / (Am[0] * Am[3] - Am[1] * Am[2]);
		Xm[1] = (Am[0] * Lm[1] - Am[2] * Lm[0]) / (Am[0] * Am[3] - Am[1] * Am[2]);

		P0 += Xm[0];
		L0 += Xm[1];
		deltaX_scale[0] = Xm[0] * rpc.latScale;
		deltaX_scale[1] = Xm[1] * rpc.longScale;
	}

	P = P0;
	L = L0;

	return 1;
}

int CRPC_Fun::RPC_Image2Ground(double row, double column, double Height, RPC rpc, double &lat, double &lon)
{
	double P, L, H, r, c;
	r = (row - rpc.lineOffset) / rpc.lineScale;
	c = (column - rpc.sampOffset) / rpc.sampScale;

	H = (Height - rpc.heightOffset) / rpc.heightScale;

	RPC_Image2Ground_normalize(r, c, H, rpc, P, L);

	lat = P*rpc.latScale + rpc.latOffset;
	lon = L*rpc.longScale + rpc.longOffset;

	return 1;
}

int CRPC_Fun::RPC_coefficient_derivative(double P, double L, double H, int ID, char var, double &result)
{
	if (ID < 1 || ID>20)
	{
		return 0;
	}

	if (var != 'P' && var != 'L' && var != 'H')
	{
		return 0;
	}

	if (ID == 1)
	{
		result = 0;
	}
	else if (ID == 2)
	{
		if (var == 'P')
		{
			result = 0;
		}
		else if (var=='L')
		{
			result = 1;
		}
		else
		{
			result = 0;
		}
	}
	else if (ID == 3)
	{
		if (var == 'P')
		{
			result = 1;
		}
		else if (var == 'L')
		{
			result = 0;
		}
		else
		{
			result = 0;
		}
	}
	else if (ID == 4)
	{
		if (var == 'P')
		{
			result = 0;
		}
		else if (var == 'L')
		{
			result = 0;
		}
		else
		{
			result = 1;
		}
	}
	else if (ID == 5)
	{
		if (var == 'P')
		{
			result = L;
		}
		else if (var == 'L')
		{
			result = P;
		}
		else
		{
			result = 0;
		}
	}
	else if (ID == 6)
	{
		if (var == 'P')
		{
			result = 0;
		}
		else if (var == 'L')
		{
			result = H;
		}
		else
		{
			result = L;
		}
	}
	else if (ID == 7)
	{
		if (var == 'P')
		{
			result = H;
		}
		else if (var == 'L')
		{
			result = 0;
		}
		else
		{
			result = P;
		}
	}
	else if (ID == 8)
	{
		if (var == 'P')
		{
			result = 0;
		}
		else if (var == 'L')
		{
			result = 2 * L;
		}
		else
		{
			result = 0;
		}
	}
	else if (ID == 9)
	{
		if (var == 'P')
		{
			result = 2 * P;
		}
		else if (var == 'L')
		{
			result = 0;
		}
		else
		{
			result = 0;
		}
	}
	else if (ID == 10)
	{
		if (var == 'P')
		{
			result = 0;
		}
		else if (var == 'L')
		{
			result = 0;
		}
		else
		{
			result = 2 * H;
		}
	}
	else if (ID == 11)
	{
		if (var == 'P')
		{
			result = L*H;
		}
		else if (var == 'L')
		{
			result = P*H;
		}
		else
		{
			result = P*L;
		}
	}
	else if (ID == 12)
	{
		if (var == 'P')
		{
			result = 0;
		}
		else if (var == 'L')
		{
			result = 3*L*L;
		}
		else
		{
			result = 0;
		}
	}
	else if (ID == 13)
	{
		if (var == 'P')
		{
			result = 2*L*P;
		}
		else if (var == 'L')
		{
			result = P*P;
		}
		else
		{
			result = 0;
		}
	}
	else if (ID == 14)
	{
		if (var == 'P')
		{
			result = 0;
		}
		else if (var == 'L')
		{
			result = H*H;
		}
		else
		{
			result = 2*L*H;
		}
	}
	else if (ID == 15)
	{
		if (var == 'P')
		{
			result = L*L;
		}
		else if (var == 'L')
		{
			result = 2*L*P;
		}
		else
		{
			result = 0;
		}
	}
	else if (ID == 16)
	{
		if (var == 'P')
		{
			result = 3*P*P;
		}
		else if (var == 'L')
		{
			result = 0;
		}
		else
		{
			result = 0;
		}
	}
	else if (ID == 17)
	{
		if (var == 'P')
		{
			result = H*H;
		}
		else if (var == 'L')
		{
			result = 0;
		}
		else
		{
			result = 2*P*H;
		}
	}
	else if (ID == 18)
	{
		if (var == 'P')
		{
			result = 0;
		}
		else if (var == 'L')
		{
			result = 2*L*H;
		}
		else
		{
			result = L*L;
		}
	}
	else if (ID == 19)
	{
		if (var == 'P')
		{
			result = 2*P*H;
		}
		else if (var == 'L')
		{
			result = 0;
		}
		else
		{
			result = P*P;
		}
	}
	else if (ID == 20)
	{
		if (var == 'P')
		{
			result = 0;
		}
		else if (var == 'L')
		{
			result = 0;
		}
		else
		{
			result = 3*H*H;
		}
	}

	return 1;
}

int CRPC_Fun::Multi_view_Forward_intersection(double *x, double *y, RPC *rpcs, int ptNum, double &lat, double &lon, double &hei)
{
	int totalNum = ptNum*(ptNum - 1) / 2;
	int i;
	double *lat_array, *lon_array, *hei_array;
	lat_array = new double[totalNum];
	lon_array = new double[totalNum];
	hei_array = new double[totalNum];

	int iter = 0;
	for (i = 0; i < ptNum - 1; i++)
		for (int j = i + 1; j < ptNum;j++)
		{
			double x1, y1, x2, y2;
			x1 = x[i]; y1 = y[i];
			x2 = x[j]; y2 = y[j];

			RPC rpc1, rpc2;
			rpc1 = rpcs[i];
			rpc2 = rpcs[j];

			RPC_ForwardIntersection(rpc1, rpc2, x1, y1, x2, y2, lat_array[iter], lon_array[iter], hei_array[iter]);
			iter++;
		}

	//output results
	lat = 0;
	lon = 0;
	hei = 0;
	for (i = 0; i < totalNum; i++)
	{
		lat += lat_array[i];
		lon += lon_array[i];
		hei += hei_array[i];
	}

	lat /= totalNum;
	lon /= totalNum;
	hei /= totalNum;

	//free memory
	delete[]lat_array; lat_array = NULL;
	delete[]lon_array; lon_array = NULL;
	delete[]hei_array; hei_array = NULL;

	return 1;
}

int CRPC_Fun::ComputeRPCForPlaneRectification(Long_Strip_Img *strips, Long_Strip_Img *&rect_imgs, int stripNum,
	int Virtual_GCP_Plane_Size, int Virtual_GCP_Height_Size, char *zone, double avgHeight)
{
	CLL_UTM geo_conv;
	CFileOperation file;

	for (int i = 0; i < stripNum; i++)
	{
		int patchNum = rect_imgs[i].patchNum;
		for (int j = 0; j < patchNum;j++)
		{
			double *lat, *lon, *hei;
			double *ix, *iy;
			int ptNum = 0;

			char *rect_path;
			char *tfw_path;
			char *img_path;
			char *rpb_path;

			rect_path = rect_imgs[i].img_paths[j];
			tfw_path = rect_imgs[i].rpb_paths[j];
			img_path = strips[i].img_paths[j];
			rpb_path = strips[i].rpb_paths[j];

			CImageBase img;
			img.Open(rect_path);
			int w_rect, h_rect;
			w_rect = img.GetCols();
			h_rect = img.GetRows();
			img.Close();

			TfwPara tfw_para;
			ReadTfwFile(tfw_path, tfw_para);

			double X_corner[4], Y_corner[4];
			X_corner[0] = tfw_para.A * 0 + tfw_para.B * 0 + tfw_para.C;
			Y_corner[0] = tfw_para.D * 0 + tfw_para.E * 0 + tfw_para.F;
			X_corner[1] = tfw_para.A * 0 + tfw_para.B * (h_rect - 1) + tfw_para.C;
			Y_corner[1] = tfw_para.D * 0 + tfw_para.E * (h_rect - 1) + tfw_para.F;
			X_corner[2] = tfw_para.A * (w_rect - 1) + tfw_para.B * (h_rect - 1) + tfw_para.C;
			Y_corner[2] = tfw_para.D * (w_rect - 1) + tfw_para.E * (h_rect - 1) + tfw_para.F;
			X_corner[3] = tfw_para.A * (w_rect - 1) + tfw_para.B * 0 + tfw_para.C;
			Y_corner[3] = tfw_para.D * (w_rect - 1) + tfw_para.E * 0 + tfw_para.F;

			double minX, minY, maxX, maxY;
			minX = min(X_corner[0], min(X_corner[1], min(X_corner[2], X_corner[3])));
			maxX = max(X_corner[0], max(X_corner[1], max(X_corner[2], X_corner[3])));
			minY = min(Y_corner[0], min(Y_corner[1], min(Y_corner[2], Y_corner[3])));
			maxY = max(Y_corner[0], max(Y_corner[1], max(Y_corner[2], Y_corner[3])));

			RPC rpc;
			ReadRpc_singleFile(rpb_path, rpc);
			double minH, maxH;
			minH = rpc.heightOffset - rpc.heightScale;
			maxH = rpc.heightOffset + rpc.heightScale;

			double stepX, stepY, stepZ;
			stepX = (maxX - minX) / Virtual_GCP_Plane_Size;
			stepY = (maxY - minY) / Virtual_GCP_Plane_Size;
			stepZ = (maxH - minH) / Virtual_GCP_Height_Size;

			double gX, gY, gZ;
			ptNum = 0;
			for (gY = minY; gY <= maxY + 0.1; gY += stepY)
				for (gX = minX; gX <= maxX + 0.1; gX += stepX)
					for (gZ = minH; gZ <= maxH + 0.1; gZ += stepZ)
					{
						ptNum++;
					}

			lat = new double[ptNum];
			lon = new double[ptNum];
			hei = new double[ptNum];
			ix = new double[ptNum];
			iy = new double[ptNum];

			int iter = 0;
			for (gY = minY; gY <= maxY + 0.1; gY += stepY)
				for (gX = minX; gX <= maxX + 0.1; gX += stepX)
					for (gZ = minH; gZ <= maxH + 0.1; gZ+=stepZ)
					{
						//gX, gY, gZ are UTM coordinates of virtual GCP

						//UTM 2 lat, lon
						//double tlat, tlon;
						geo_conv.UTM2LL(gX, gY, zone, lat[iter], lon[iter]);
						hei[iter] = gZ;

						//lat lon, hei 2 row col
						double tx, ty;
						RPC_Ground2Image(lat[iter], lon[iter], hei[iter], rpc, ty, tx);

						//row col 2 lat lon on avgheight
						double lat_rect, lon_rect;
						RPC_Image2Ground(ty, tx, avgHeight, rpc, lat_rect, lon_rect);

						//lat lon on avgheight to UTM X, Y on avgheight
						double X_rect, Y_rect;
						geo_conv.LL2UTM_fixedZone(lat_rect, lon_rect, zone, X_rect, Y_rect);

						ix[iter] = (tfw_para.B*tfw_para.F - tfw_para.B*Y_rect - tfw_para.C*tfw_para.E + tfw_para.E*X_rect) / (tfw_para.A*tfw_para.E - tfw_para.B*tfw_para.D);
						iy[iter] = -(tfw_para.A*tfw_para.F - tfw_para.A*Y_rect - tfw_para.C*tfw_para.D + tfw_para.D*X_rect) / (tfw_para.A*tfw_para.E - tfw_para.B*tfw_para.D);

						iter++;
					}

			//compute RPC for rectified images with the virtual GCPs
			RPC rpc_plane;
			strcpy(rpc_plane.satID, rpc.satID);
			strcpy(rpc_plane.bandID, rpc.bandID);
			strcpy(rpc_plane.SpecId, rpc.SpecId);
			strcpy(rpc_plane.BEGIN_GROUP, rpc.BEGIN_GROUP);
			strcpy(rpc_plane.errBias, rpc.errBias);
			strcpy(rpc_plane.errRand, rpc.errRand);

			//1. compute the header information of the final rpc
			rpc_plane.lineOffset = (h_rect - 1) / 2;
			rpc_plane.sampOffset = (w_rect - 1) / 2;
			rpc_plane.lineScale = rpc_plane.lineOffset + 1;
			rpc_plane.sampScale = rpc_plane.sampOffset + 1;

			rpc_plane.heightOffset = rpc.heightOffset;
			rpc_plane.heightScale = rpc.heightScale;

			double Lat_corner[4], Lon_corner[4];
			for (int t = 0; t < 4;t++)
			{
				geo_conv.UTM2LL(X_corner[t], Y_corner[t], zone, Lat_corner[t], Lon_corner[t]);
			}

			double minLat, maxLat, minLon, maxLon;
			minLat = min(Lat_corner[0], min(Lat_corner[1], min(Lat_corner[2], Lat_corner[3])));
			maxLat = max(Lat_corner[0], max(Lat_corner[1], max(Lat_corner[2], Lat_corner[3])));
			minLon = min(Lon_corner[0], min(Lon_corner[1], min(Lon_corner[2], Lon_corner[3])));
			maxLon = max(Lon_corner[0], max(Lon_corner[1], max(Lon_corner[2], Lon_corner[3])));

			rpc_plane.latOffset = (minLat + maxLat) / 2;
			rpc_plane.longOffset = (minLon + maxLon) / 2;
			rpc_plane.latScale = maxLat - rpc_plane.latOffset;
			rpc_plane.longScale = maxLon - rpc_plane.longOffset;

			//compute rpc for the plane rectified results
			double avgd = 0, maxd = 0;
			ComputeRPCfromVirtualGCPs(ix, iy, lat, lon, hei, ptNum, rpc_plane, maxd, avgd);

			printf("strip ID = %d, patch ID = %d, rpc-regeneration error, avgd = %lf , maxd = %lf\n", i, j, avgd, maxd);

			char rpc_output_path[MaxLen];
			file.FindDirPath_prj(rect_imgs[i].rpb_paths[j], rpc_output_path, true);
			strcat(rpc_output_path, ".RPB");
			strcpy(rect_imgs[i].refined_rpb_paths[j], rpc_output_path);
			WriteRPBFile(rpc_plane, rect_imgs[i].refined_rpb_paths[j]);
			//printf("The corresponding rpc is saved in : \n  %s\n", rpc_output_path);

			//free memory
			delete[]lat; lat = NULL;
			delete[]lon; lon = NULL;
			delete[]hei; hei = NULL;
			delete[]ix; ix = NULL;
			delete[]iy; iy = NULL;
		}
	}

	
	return 1;
}

int CRPC_Fun::ComputeRPCfromVirtualGCPs(double *ix, double *iy, double *lat, double *lon, double *hei, int ptNum,
	RPC &rpc, double &maxd, double &avgd)
{
	CSimple_Matrix_Computation matc;

	double *r, *c;
	double *P, *L, *H;
	r = new double[ptNum];
	c = new double[ptNum];
	P = new double[ptNum];
	L = new double[ptNum];
	H = new double[ptNum];

	for (int i = 0; i < ptNum;i++)
	{
		r[i] = (iy[i] - rpc.lineOffset) / rpc.lineScale;
		c[i] = (ix[i] - rpc.sampOffset) / rpc.sampScale;

		P[i] = (lat[i] - rpc.latOffset) / rpc.latScale;
		L[i] = (lon[i] - rpc.longOffset) / rpc.longScale;
		H[i] = (hei[i] - rpc.heightOffset) / rpc.heightScale;
	}

	double *A, *B, *X;
	A = new double[(2 * ptNum) * 78];
	B = new double[2 * ptNum];
	X = new double[78];

	memset(A, 0, sizeof(double)*(2 * ptNum) * 78);
	memset(B, 0, sizeof(double) * 2 * ptNum);
	for (int i = 0; i < ptNum; i++)
	{
		//writing A
		//row equation
		for (int j = 0; j < 20; j++)
		{
			double pi = 0;
			RPC_coefficient(P[i], L[i], H[i], j + 1, pi);

			A[(i * 2 + 0) * 78 + j] = pi;
		}

		for (int j = 20; j < 39; j++)
		{
			double pi = 0;
			RPC_coefficient(P[i], L[i], H[i], j - 19 + 1, pi);

			A[(i * 2 + 0) * 78 + j] = -r[i] * pi;
		}

		//col equation
		for (int j = 39; j < 59; j++)
		{
			double pi = 0;
			RPC_coefficient(P[i], L[i], H[i], j - 39 + 1, pi);

			A[(i * 2 + 1) * 78 + j] = pi;
		}

		for (int j = 59; j < 78; j++)
		{
			double pi = 0;
			RPC_coefficient(P[i], L[i], H[i], j - 58 + 1, pi);

			A[(i * 2 + 1) * 78 + j] = -c[i] * pi;
		}

		//writing L
		B[i * 2 + 0] = r[i];
		B[i * 2 + 1] = c[i];
	}

	double alpha = 0;
	matc.LeastSquares(A, B, X, 2 * ptNum, 78, alpha);

	for (int i = 0; i < 20;i++)
	{
		rpc.lineNumCoef[i] = X[i];
	}

	rpc.lineDenCoef[0] = 1;
	for (int i = 1; i < 20; i++)
	{
		rpc.lineDenCoef[i] = X[i + 19];
	}

	for (int i = 0; i < 20; i++)
	{
		rpc.sampNumCoef[i] = X[i + 39];
	}

	rpc.sampDenCoef[0] = 1;
	for (int i = 1; i < 20; i++)
	{
		rpc.sampDenCoef[i] = X[i + 58];//59
	}

	maxd = 0;
	avgd = 0;
	for (int i = 0; i < ptNum; i++)
	{
		double tlat, tlon, thei;
		tlat = lat[i];
		tlon = lon[i];
		thei = hei[i];

		double tx, ty;
		RPC_Ground2Image(tlat, tlon, thei, rpc, ty, tx);

		double td;
		td = sqrt((tx - ix[i])*(tx - ix[i]) + (ty - iy[i])*(ty - iy[i]));

		if (td > maxd)
			maxd = td;

		avgd += td;
	}
	avgd /= ptNum;

// 	double maxd0 = maxd;
// 	RPC rpc0 = rpc;
//  	if (maxd>0.1)
// 	{
// 		printf("The max distance is too big (larger 0.1 pixel) for RPC re-generation!\n");
// 		
// 		//only change this
// 		int rpc_num = 5;
// 
// 		alpha = 0.0005;
// 		RPC *rpc_ridge = new RPC[rpc_num];
// 		double *max_dis = new double[rpc_num];
// 		memset(max_dis, 0, sizeof(double)*rpc_num);
// 		for (int i = 0; i < rpc_num;i++)
// 		{
// 			rpc_ridge[i] = rpc;
// 		}
// 		double alpha0 = alpha * powl(10, rpc_num / 2);
// 
// 		for (int i = 0; i < 2 * ptNum * 78; i++)
// 		{
// 			A[i] = A[i] * (1000.0 / sqrt(2 * ptNum));
// 		}
// 		for (int i = 0; i < 2 * ptNum; i++)
// 		{
// 			B[i] = B[i] * (1000.0 / sqrt(2 * ptNum));
// 		}
// 		for (int b = 0; b < rpc_num;b++)
// 		{
// 			// compute new alpha
// 			alpha = alpha0 / powl(10, b);
// 
// 			maxd = 0;
// 			avgd = 0;
// 
// 			LeastSquares_adapt(A, B, X, 2 * ptNum, 78, alpha);
// 
// 			for (int i = 0; i < 20; i++)
// 			{
// 				rpc_ridge[b].lineNumCoef[i] = X[i];
// 			}
// 
// 			rpc_ridge[b].lineDenCoef[0] = 1;
// 			for (int i = 1; i < 20; i++)
// 			{
// 				rpc_ridge[b].lineDenCoef[i] = X[i + 19];
// 			}
// 
// 			for (int i = 0; i < 20; i++)
// 			{
// 				rpc_ridge[b].sampNumCoef[i] = X[i + 39];
// 			}
// 
// 			rpc_ridge[b].sampDenCoef[0] = 1;
// 			for (int i = 1; i < 20; i++)
// 			{
// 				rpc_ridge[b].sampDenCoef[i] = X[i + 58];//59
// 			}
// 
// 			for (int i = 0; i < ptNum; i++)
// 			{
// 				double tlat, tlon, thei;
// 				tlat = lat[i];
// 				tlon = lon[i];
// 				thei = hei[i];
// 
// 				double tx, ty;
// 				RPC_Ground2Image(tlat, tlon, thei, rpc_ridge[b], ty, tx);
// 
// 				double td;
// 				td = sqrt((tx - ix[i])*(tx - ix[i]) + (ty - iy[i])*(ty - iy[i]));
// 
// 				if (td > maxd)
// 					maxd = td;
// 
// 				avgd += td;
// 			}
// 			avgd /= ptNum;
// 
// 			max_dis[b] = maxd;
// 		}
// 
// 		//compute the optimal alpha
// 		maxd = max_dis[0];
// 		rpc = rpc_ridge[0];
// 		for (int i = 0; i < rpc_num;i++)
// 		{
// 			if (maxd > max_dis[i])
// 			{
// 				maxd = max_dis[i];
// 				rpc = rpc_ridge[i];
// 			}
// 		}
// 
// 		if (maxd>maxd0)
// 		{
// 			maxd = maxd0;
// 			rpc = rpc0;
// 		}
// 
// 		//free memory
// 		delete[]rpc_ridge; rpc_ridge = NULL;
// 	}

	//free memory
	delete[]r; r = NULL;
	delete[]c; c = NULL;
	delete[]P; P = NULL;
	delete[]L; L = NULL;
	delete[]H; H = NULL;
	delete[]A; A = NULL;
	delete[]B; B = NULL;
	delete[]X; X = NULL;
	return 1;
}


int CRPC_Fun::RPC_ComputeConstantBiasUsingGCP(double *ix, double *iy, double *lat, double *lon, double *hei, int ptNum, RPC rpc,
	double &drow, double &dcol, double &error_reprojection)
{
	if (ptNum==0)
	{
		drow = 0;
		dcol = 0;


		error_reprojection = 0;

		return  0;
	}
	CImageFeatureMatching feaM;

	double *reproj_x, *reproj_y;
	reproj_x = new double[ptNum];
	reproj_y = new double[ptNum];

	for (int i = 0; i < ptNum;i++)
	{
		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc, reproj_y[i], reproj_x[i]);
	}

	double *drows, *dcols;
	drows = new double[ptNum];
	dcols = new double[ptNum];
	for (int i = 0; i < ptNum;i++)
	{
		drows[i] = reproj_y[i] - iy[i];
		dcols[i] = reproj_x[i] - ix[i];
	}

// 	feaM.MedianFilter(drows, ptNum, drow);
// 	feaM.MedianFilter(dcols, ptNum, dcol);
	//To speed up running time
	feaM.GetWeightedCenter(drows, ptNum, 3, drow);
	feaM.GetWeightedCenter(dcols, ptNum, 3, dcol);

	RPC rpc_refine = rpc;
	RefineRPC_bias_constant(rpc_refine, drow, dcol);

	error_reprojection = 0;
	for (int i = 0; i < ptNum; i++)
	{
		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_refine, reproj_y[i], reproj_x[i]);
	}

	for (int i = 0; i < ptNum;i++)
	{
		double dis;
		dis = sqrt((reproj_y[i] - iy[i])*(reproj_y[i] - iy[i]) + (reproj_x[i] - ix[i]) * (reproj_x[i] - ix[i]));

		error_reprojection += dis;
	}
	error_reprojection /= ptNum;

	//free memory
	delete[]reproj_x; reproj_x = NULL;
	delete[]reproj_y; reproj_y = NULL;
	delete[]drows; drows = NULL;
	delete[]dcols; dcols = NULL;

	return 1;
}

int CRPC_Fun::RPC_ForwardIntersection(RPC rpc1, RPC rpc2, double ix1, double iy1, double ix2, double iy2,
	double &Lat, double &Lon, double &Hei)
{
	//normalzie
	double r1, c1;
	double r2, c2;
	r1 = (iy1 - rpc1.lineOffset) / rpc1.lineScale;
	c1 = (ix1 - rpc1.sampOffset) / rpc1.sampScale;

	r2 = (iy2 - rpc2.lineOffset) / rpc2.lineScale;
	c2 = (ix2 - rpc2.sampOffset) / rpc2.sampScale;

	//compute initial value
	double height = (rpc1.heightOffset + rpc2.heightOffset) / 2;
	double P0_1, L0_1, H0_1;
//	double P, L, H;
	H0_1 = 0;
	double Lat0, Lon0, Hei0;
	RPC_Image2Ground_normalize(r1, c1, H0_1, rpc1, P0_1, L0_1);


	Lat0 = P0_1*rpc1.latScale + rpc1.latOffset;
	Lon0 = L0_1*rpc1.longScale + rpc1.longOffset;
	Hei0 = H0_1*rpc1.heightScale + rpc1.heightOffset;

	//normalized initial values of the second image
	double P0_2, L0_2, H0_2;

	//Forward Intersection
	int i, j;
	double thresh = 0.1;
	double avg_dis = 1;
	double previous = 0;

	double Am[4 * 3], Xm[3], Lm[4];
	double A33[9], L3[3];
	MatrixXd Ge(3, 3);
	VectorXd He(3);
	
	while (fabs(avg_dis  - previous)>thresh && avg_dis>thresh)
	{
		previous = avg_dis;
		//compute normalized initial values
		P0_1 = (Lat0 - rpc1.latOffset) / rpc1.latScale;
		L0_1 = (Lon0 - rpc1.longOffset) / rpc1.longScale;
		H0_1 = (Hei0 - rpc1.heightOffset) / rpc1.heightScale;

		P0_2 = (Lat0 - rpc2.latOffset) / rpc2.latScale;
		L0_2 = (Lon0 - rpc2.longOffset) / rpc2.longScale;
		H0_2 = (Hei0 - rpc2.heightOffset) / rpc2.heightScale;

		//compute Lm
		double line_num1 = 0, line_den1 = 0;
		double samp_num1 = 0, samp_den1 = 0;
		double line_num2 = 0, line_den2 = 0;
		double samp_num2 = 0, samp_den2 = 0;
		for (i = 0; i < 20; i++)
		{
			double pi;
			RPC_coefficient(P0_1, L0_1, H0_1, i + 1, pi);
			line_num1 += rpc1.lineNumCoef[i] * pi;
			line_den1 += rpc1.lineDenCoef[i] * pi;
			samp_num1 += rpc1.sampNumCoef[i] * pi;
			samp_den1 += rpc1.sampDenCoef[i] * pi;

			RPC_coefficient(P0_2, L0_2, H0_2, i + 1, pi);
			line_num2 += rpc2.lineNumCoef[i] * pi;
			line_den2 += rpc2.lineDenCoef[i] * pi;
			samp_num2 += rpc2.sampNumCoef[i] * pi;
			samp_den2 += rpc2.sampDenCoef[i] * pi;
		}
		Lm[0] = -(line_num1 - r1*line_den1);
		Lm[1] = -(samp_num1 - c1*samp_den1);
		Lm[2] = -(line_num2 - r2*line_den2);
		Lm[3] = -(samp_num2 - c2*samp_den2);

		//compute Am
		double line_num_der_P1 = 0, line_den_der_P1 = 0;
		double line_num_der_L1 = 0, line_den_der_L1 = 0;
		double samp_num_der_P1 = 0, samp_den_der_P1 = 0;
		double samp_num_der_L1 = 0, samp_den_der_L1 = 0;
		double line_num_der_H1 = 0, line_den_der_H1 = 0;
		double samp_num_der_H1 = 0, samp_den_der_H1 = 0;

		double line_num_der_P2 = 0, line_den_der_P2 = 0;
		double line_num_der_L2 = 0, line_den_der_L2 = 0;
		double samp_num_der_P2 = 0, samp_den_der_P2 = 0;
		double samp_num_der_L2 = 0, samp_den_der_L2 = 0;
		double line_num_der_H2 = 0, line_den_der_H2 = 0;
		double samp_num_der_H2 = 0, samp_den_der_H2 = 0;
		for (i = 0; i < 20; i++)
		{
			double pi;
			//P
			RPC_coefficient_derivative(P0_1, L0_1, H0_1, i + 1, 'P', pi);
			line_num_der_P1 += rpc1.lineNumCoef[i] * pi;
			line_den_der_P1 += rpc1.lineDenCoef[i] * pi;
			samp_num_der_P1 += rpc1.sampNumCoef[i] * pi;
			samp_den_der_P1 += rpc1.sampDenCoef[i] * pi;

			RPC_coefficient_derivative(P0_2, L0_2, H0_2, i + 1, 'P', pi);
			line_num_der_P2 += rpc2.lineNumCoef[i] * pi;
			line_den_der_P2 += rpc2.lineDenCoef[i] * pi;
			samp_num_der_P2 += rpc2.sampNumCoef[i] * pi;
			samp_den_der_P2 += rpc2.sampDenCoef[i] * pi;

			//L
			RPC_coefficient_derivative(P0_1, L0_1, H0_1, i + 1, 'L', pi);
			line_num_der_L1 += rpc1.lineNumCoef[i] * pi;
			line_den_der_L1 += rpc1.lineDenCoef[i] * pi;
			samp_num_der_L1 += rpc1.sampNumCoef[i] * pi;
			samp_den_der_L1 += rpc1.sampDenCoef[i] * pi;

			RPC_coefficient_derivative(P0_2, L0_2, H0_2, i + 1, 'L', pi);
			line_num_der_L2 += rpc2.lineNumCoef[i] * pi;
			line_den_der_L2 += rpc2.lineDenCoef[i] * pi;
			samp_num_der_L2 += rpc2.sampNumCoef[i] * pi;
			samp_den_der_L2 += rpc2.sampDenCoef[i] * pi;

			//H
			RPC_coefficient_derivative(P0_1, L0_1, H0_1, i + 1, 'H', pi);
			line_num_der_H1 += rpc1.lineNumCoef[i] * pi;
			line_den_der_H1 += rpc1.lineDenCoef[i] * pi;
			samp_num_der_H1 += rpc1.sampNumCoef[i] * pi;
			samp_den_der_H1 += rpc1.sampDenCoef[i] * pi;

			RPC_coefficient_derivative(P0_2, L0_2, H0_2, i + 1, 'H', pi);
			line_num_der_H2 += rpc2.lineNumCoef[i] * pi;
			line_den_der_H2 += rpc2.lineDenCoef[i] * pi;
			samp_num_der_H2 += rpc2.sampNumCoef[i] * pi;
			samp_den_der_H2 += rpc2.sampDenCoef[i] * pi;
		}
		//equ1
		Am[0] = (line_num_der_P1 - r1*line_den_der_P1)/rpc1.latScale;
		Am[1] = (line_num_der_L1 - r1*line_den_der_L1)/rpc1.longScale;
		Am[2] = (line_num_der_H1 - r1*line_den_der_H1)/rpc1.heightScale;

		//equ2
		Am[3] = (samp_num_der_P1 - c1*samp_den_der_P1) / rpc1.latScale;
		Am[4] = (samp_num_der_L1 - c1*samp_den_der_L1) / rpc1.longScale;
		Am[5] = (samp_num_der_H1 - c1*samp_den_der_H1) / rpc1.heightScale;

		//equ3
		Am[6] = (line_num_der_P2 - r2*line_den_der_P2) / rpc2.latScale;
		Am[7] = (line_num_der_L2 - r2*line_den_der_L2) / rpc2.longScale;
		Am[8] = (line_num_der_H2 - r2*line_den_der_H2) / rpc2.heightScale;

		//equ4
		Am[9] = (samp_num_der_P2 - c2*samp_den_der_P2) / rpc2.latScale;
		Am[10] = (samp_num_der_L2 - c2*samp_den_der_L2) / rpc2.longScale;
		Am[11] = (samp_num_der_H2 - c2*samp_den_der_H2) / rpc2.heightScale;

		//conver to 3*3 matrix
		CSimple_Matrix_Computation mc;
		mc.GetLinearFun(Am, 4, 3, Lm, A33, L3);
		
		//compute solutions
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3;j++)
			{
				Ge(i, j) = A33[i * 3 + j];
			}
		for (i = 0; i < 3;i++)
		{
			He(i) = L3[i];
		}

		VectorXd Xe = Ge.ldlt().solve(He);

		for (i = 0; i < 3;i++)
		{
			Xm[i] = Xe(i);
		}
		Lat0 += Xe(0);
		Lon0 += Xe(1);
		Hei0 += Xe(2);

		double new_r_1, new_c_1, new_r_2, new_c_2;
		RPC_Ground2Image(Lat0, Lon0, Hei0, rpc1, new_r_1, new_c_1);
		RPC_Ground2Image(Lat0, Lon0, Hei0, rpc2, new_r_2, new_c_2);

		double dis1, dis2;
		dis1 = sqrt((new_r_1 - iy1)*(new_r_1 - iy1) + (new_c_1 - ix1)*(new_c_1 - ix1));
		dis2 = sqrt((new_r_2 - iy2)*(new_r_2 - iy2) + (new_c_2 - ix2)*(new_c_2 - ix2));

		avg_dis = (dis1 + dis2) / 2;
	}

	Lat = Lat0;
	Lon = Lon0;
	Hei = Hei0;

	return 1;
}

int CRPC_Fun::RPC_update_From_AOI(char *original_rpc_path, char *new_rpc_path, int min_px, int min_py)
{
	RPC rpc_para;
	ReadRpc_singleFile(original_rpc_path, rpc_para);

	RPC rpc_new;
	rpc_new = rpc_para;
	rpc_new.lineOffset = rpc_new.lineOffset - min_py;
	rpc_new.sampOffset = rpc_new.sampOffset - min_px;

	WriteRPBFile(rpc_new, new_rpc_path);

	return 1;
}


int CRPC_Fun::GetCornerPostions(int *imw, int *imh, double *height, RPC *rpcs, int satNum, double *&lats, double *&lons)
{
	int i;
	double row, column;

	lats = new double[satNum * 4];
	lons = new double[satNum * 4];
	for (i = 0; i < satNum;i++)
	{
		row = 0; column = 0;
		RPC_Image2Ground(row, column, height[i], rpcs[i], lats[i * 4 + 0], lons[i * 4 + 0]);

		row = 0; column = imw[i] - 1;
		RPC_Image2Ground(row, column, height[i], rpcs[i], lats[i * 4 + 1], lons[i * 4 + 1]);

		row = imh[i] - 1; column = imw[i] - 1;
		RPC_Image2Ground(row, column, height[i], rpcs[i], lats[i * 4 + 2], lons[i * 4 + 2]);

		row = imh[i] - 1; column = 0;
		RPC_Image2Ground(row, column, height[i], rpcs[i], lats[i * 4 + 3], lons[i * 4 + 3]);
	}

	return 1;
}

int CRPC_Fun::ComputeTriArea(double x[3], double y[3], double &S)
{
	double a, b, c;
	a = sqrt((x[1] - x[0])*(x[1] - x[0]) + (y[1] - y[0])*(y[1] - y[0]));
	b = sqrt((x[2] - x[1])*(x[2] - x[1]) + (y[2] - y[1])*(y[2] - y[1]));
	c = sqrt((x[0] - x[2])*(x[0] - x[2]) + (y[0] - y[2])*(y[0] - y[2]));

	double p = (a + b + c) / 2;
	double S2 = p*(p - a)*(p - b)*(p - c);
	if (S2<=0)
	{
		S = 0;
		return 0;
	}

	S = sqrt(S2);
	return 1;
}

int CRPC_Fun::ReadTfwFile(char *tfw_path, TfwPara &para)
{
	//Read tfw information
	FILE *fp_tfw = fopen(tfw_path, "r");
	fscanf(fp_tfw, "%lf %lf %lf %lf %lf %lf", &para.A, &para.D, &para.B, &para.E, &para.C, &para.F);
	fclose(fp_tfw); fp_tfw = NULL;
	return 1;
}

int CRPC_Fun::ReadTfwFile(char *tfw_path, double &A, double &B, double &C, double &D, double &E, double &F)
{

	//Read tfw information
	FILE *fp_tfw = fopen(tfw_path, "r");
	fscanf(fp_tfw, "%lf %lf %lf %lf %lf %lf", &A, &D, &B, &E, &C, &F);
	fclose(fp_tfw); fp_tfw = NULL;

	return 1;
}

int CRPC_Fun::ComputeOverlapArea(double *ux, double *uy, double *S, int ID1, int ID2, double &overlapS)
{
	int i;
	double minx = 0, miny = 0, maxx = 0, maxy = 0;
	if (S[ID1]<S[ID2])
	{
		minx = ux[ID1 * 4 + 0]; miny = uy[ID1 * 4 + 0];
		maxx = ux[ID1 * 4 + 0]; maxy = uy[ID1 * 4 + 0];
		for (i = 1; i < 4;i++)
		{
			if (minx > ux[ID1 * 4 + i]) minx = ux[ID1 * 4 + i];
			if (miny > uy[ID1 * 4 + i]) miny = uy[ID1 * 4 + i];

			if (maxx < ux[ID1 * 4 + i]) maxx = ux[ID1 * 4 + i];
			if (maxy < uy[ID1 * 4 + i]) maxy = uy[ID1 * 4 + i];
		}
	}
	else
	{
		minx = ux[ID2 * 4 + 0]; miny = uy[ID2 * 4 + 0];
		maxx = ux[ID2 * 4 + 0]; maxy = uy[ID2 * 4 + 0];
		for (i = 1; i < 4; i++)
		{
			if (minx > ux[ID2 * 4 + i]) minx = ux[ID2 * 4 + i];
			if (miny > uy[ID2 * 4 + i]) miny = uy[ID2 * 4 + i];

			if (maxx < ux[ID2 * 4 + i]) maxx = ux[ID2 * 4 + i];
			if (maxy < uy[ID2 * 4 + i]) maxy = uy[ID2 * 4 + i];
		}
	}

	double vx1[4], vy1[4];
	double vx2[4], vy2[4];
	for (i = 0; i < 4; i++)
	{
		vx1[i] = ux[ID1 * 4 + i]; vy1[i] = uy[ID1 * 4 + i];
		vx2[i] = ux[ID2 * 4 + i]; vy2[i] = uy[ID2 * 4 + i];
	}

	double x, y;
	overlapS = 0;
	double stepx, stepy;
	stepy = (maxy - miny) / 1000;
	stepx = (maxx - minx) / 1000;
	for (y = miny; y <= maxy+0.0001; y += stepy)
		for (x = minx; x <= maxx+0.0001; x+=stepx)
		{
			int flag1, flag2;
			flag1 = IsPointInBuffer(vx1, vy1, x, y);
			flag2 = IsPointInBuffer(vx2, vy2, x, y);
			if (flag1 == 1 && flag2 == 1)
			{
				overlapS += stepx*stepy;
			}
		}

	return 1;
}

int CRPC_Fun::IsPointInBuffer(double vx[4], double vy[4], double x, double y)
{
	int flag;
	double cross01, cross23, cross12, cross30;
	GetCross(vx[0], vy[0], vx[1], vy[1], x, y, cross01);
	GetCross(vx[2], vy[2], vx[3], vy[3], x, y, cross23);

	GetCross(vx[1], vy[1], vx[2], vy[2], x, y, cross12);
	GetCross(vx[3], vy[3], vx[0], vy[0], x, y, cross30);

	if (cross01*cross23 >= 0 && cross12*cross30 >= 0)
	{
		flag = 1;
	}
	else
		flag = 0;

	return flag;
}

int  CRPC_Fun::GetCross(double x1, double y1, double x2, double y2, double x, double y, double &crossresult)
{
	crossresult = (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1);
	return 1;
}

int  CRPC_Fun::ComputeImageProjectionArea(double ux[4], double uy[4], double &S)
{
	double tx[3], ty[3];
	double S1, S2;

	tx[0] = ux[0]; ty[0] = uy[0];
	tx[1] = ux[1]; ty[1] = uy[1];
	tx[2] = ux[2]; ty[2] = uy[2];
	ComputeTriArea(tx, ty, S1);

	tx[0] = ux[2]; ty[0] = uy[2];
	tx[1] = ux[3]; ty[1] = uy[3];
	tx[2] = ux[0]; ty[2] = uy[0];
	ComputeTriArea(tx, ty, S2);

	S = S1 + S2;
	return 1;
}


int CRPC_Fun::Get_Average_HeightFromRPCs(RPC *rpcs, int num, double &avgH)
{
	avgH = 0;
	for (int i = 0; i < num;i++)
	{
		avgH += rpcs[i].heightOffset;
	}
	avgH /= num;

	return 1;
}

int CRPC_Fun::RPC_compute_GSD_and_Height(char *img_path, char *rpc_path, double &GSD, double &height, char *zoneID)
{
	CRPC_Fun frpc;
	CLL_UTM geo_conv;
	COtherFuns fo;

	RPC rpc;
	frpc.ReadRpc_singleFile(rpc_path, rpc);
	height = rpc.heightOffset;

	CImageBase img;
	int w, h;
	img.Open(img_path);
	w = img.GetCols();
	h = img.GetRows();
	img.Close();

	double ix[4] = { 0, w - 1, w - 1, 0 }, iy[4] = { 0, 0, h - 1, h - 1 };
	double lat[4], lon[4];
	double gX[4], gY[4];
//	char zoneID[10];

	for (int i = 0; i < 4;i++)
	{
		RPC_Image2Ground(iy[i], ix[i], height, rpc, lat[i], lon[i]);
	}

	geo_conv.LL2UTM(lat[0], lon[0], gX[0], gY[0], zoneID);
	for (int i = 1; i < 4;i++)
	{
		geo_conv.LL2UTM_fixedZone(lat[i], lon[i], zoneID, gX[i], gY[i]);
	}

	double S1, S2;
	double x1[3] = { gX[0], gX[1], gX[2] }, y1[3] = { gY[0], gY[1], gY[2] };
	double x2[3] = { gX[2], gX[3], gX[1] }, y2[3] = { gY[2], gY[3], gY[1] };

	fo.ComputeTriangleArea(x1, y1, S1);
	fo.ComputeTriangleArea(x2, y2, S2);

	double S = S1 + S2;
	double S_img = w*h;

	GSD = sqrt(S / S_img);

	return 1;
}

int CRPC_Fun::AOI_Intersection(double minLat1, double minLon1, double maxLat1, double maxLon1,
	double minLat2, double minLon2, double maxLat2, double maxLon2, double *lat, double *lon)
{
	double Lat_intersection[16] = { minLat1, minLat1, maxLat1, maxLat1, minLat2, minLat2, maxLat2, maxLat2,
		minLat1, minLat1, maxLat1, maxLat1, minLat2, minLat2, maxLat2, maxLat2 };
	double Lon_intersection[16] = { minLon2, maxLon2, minLon2, maxLon2, minLon1, maxLon1, minLon1, maxLon1,
		minLon1, maxLon1, maxLon1, minLon1, minLon2, maxLon2, maxLon2, minLon2 };

	int InNum = 0;
	int Iter = 0;
	for (int i = 0; i < 16;i++)
	{
		if (Lat_intersection[i] >= minLat1 && Lat_intersection[i] <= maxLat1 &&
			Lat_intersection[i] >= minLat2 && Lat_intersection[i] <= maxLat2 &&
			Lon_intersection[i] >= minLon1 && Lon_intersection[i] <= maxLon1 &&
			Lon_intersection[i] >= minLon2 && Lon_intersection[i] <= maxLon2)
		{
			InNum++;
			if (InNum>4)
			{
				break;
			}
			lat[Iter] = Lat_intersection[i];
			lon[Iter] = Lon_intersection[i];
			Iter++;
		}
	}

	if (InNum == 4)
	{
		return 1;
	}
	else
		return 0;

	return 1;
}

int CRPC_Fun::ComputeTranslateBetweenTiles(char *imgPath1, char *rpbPath1,
	char *imgPath2, char *rpbPath2, int &tx, int &ty)
{
	//define class objects
	CImageProcess ipro;
	int w1, h1, w2, h2;
	ipro.GetImageSize(imgPath1, w1, h1);
	ipro.GetImageSize(imgPath2, w2, h2);

	RPC rpc1, rpc2;
	ReadRpc_singleFile(rpbPath1, rpc1);
	ReadRpc_singleFile(rpbPath2, rpc2);

	double avgHei = (rpc1.heightOffset + rpc2.heightOffset) / 2;
	int num = 0;
	int step = 100;

	int stepx, stepy;
	stepx = w1 / step;
	stepy = h1 / step;
	vector<double> txs, tys;
	if (stepx > 100) stepx = 100;
	if (stepy > 100) stepy = 100;

	for (int y = 0; y < h1; y += stepy)
		for (int x = 0; x < w1; x += stepx)
		{
			double lat, lon;
			double px, py;
			px = x; py = y;
			RPC_Image2Ground(py, px, avgHei, rpc1, lat, lon);

			double px2, py2;
			RPC_Ground2Image(lat, lon, avgHei, rpc2, py2, px2);

			if (px2 >= 0 && px2 <= w2 - 1 && py2 >= 0 && py2 <= h2 - 1)
			{
				txs.push_back(px - px2);
				tys.push_back(py - py2);
				num++;
			}
		}

	if (num==0) //There is no overlap between the two patches
	{
		//free memory
		txs.clear();
		tys.clear();

		vector<double>().swap(txs);
		vector<double>().swap(tys);

		tx = 0; ty = 0;

		return 0;
	}

	//compute the accurate translate value
	double tx_acc = 0, ty_acc = 0;
	for (int i = 0; i < num; i++)
	{
		tx_acc += txs[i];
		ty_acc += tys[i];
	}
	tx_acc /= num;
	ty_acc /= num;

	double rmse_x = 0, rmse_y = 0;
	for (int i = 0; i < num; i++)
	{
		rmse_x += fabs(txs[i] - tx_acc);
		rmse_y += fabs(tys[i] - ty_acc);
	}

	rmse_x /= num;
	rmse_y /= num;

	tx = (int)(tx_acc+0.5);
	ty = (int)(ty_acc + 0.5);

	//free memory
	txs.clear();
	tys.clear();

	vector<double>().swap(txs);
	vector<double>().swap(tys);

// 	FILE *fp = fopen("D:\\out.txt", "w");
// 	for (y = 0; y < h1; y += stepx)
// 		for (x = 0; x < w1; x += stepy)
// 		{
// 			double lat, lon;
// 			double px, py;
// 			px = sx1 + x; py = sy1 + y;
// 			RPC_Image2Ground(py, px, avgHei, rpc1, lat, lon);
// 
// 			double px2, py2;
// 			RPC_Ground2Image(lat, lon, avgHei, rpc2, py2, px2);
// 
// 			if (px2 >= sx2 && px2 <= sx2 + w2 - 1 && py2 >= sy2 && py2 <= sy2 + h2 - 1)
// 			{
// 				fprintf(fp, "%lf\t%lf\t%lf\t%lf\n", px2, py2, px - px2, py - py2 - (int)(ty + 0.5));
// 			}
// 		}
// 	fclose(fp); fp = NULL;

	return 1;
}

// int CBundleAdjust::ComputeTranslateBetweenTiles(unsigned short int *img1, int sx1, int sy1, int w1, int h1,
// 	unsigned short int *img2, int sx2, int sy2, int w2, int h2,
// 	RPC rpc1, RPC rpc2, double &tx, double &ty)
// {
// 	int x, y;
// 	double avgHei = 0;
// 	int num = 0;
// 	int step = 100;
// 
// 	vector<double> txs, tys;
// 	for (y = 0; y < h1; y+=step)
// 		for (x = 0; x < w1;x+=step)
// 		{
// 			double lat, lon;
// 			double px, py; 
// 			px = sx1 + x; py = sy1 + y;
// 			RPC_Image2Ground(py, px, avgHei, rpc1, lat, lon);
// 
// 			double px2, py2;
// 			RPC_Ground2Image(lat, lon, avgHei, rpc2, py2, px2);
// 
// 			if (px2>=sx2 && px2<=sx2 + w2 -1 && py2>=sy2 && py2<=sy2+h2-1)
// 			{
// 				txs.push_back(px - px2);
// 				tys.push_back(py - py2);
// 				num++;
// 			}
// 		}
// 
// 	//compute the accurate translate value
// 	double tx_acc = 0, ty_acc = 0;
// 	for (int i = 0; i < num;i++)
// 	{
// 		tx_acc += txs[i];
// 		ty_acc += tys[i];
// 	}
// 	tx_acc /= num;
// 	ty_acc /= num;
// 
// 	double rmse_x = 0, rmse_y = 0;
// 	for (int i = 0; i < num;i++)
// 	{
// 		rmse_x += fabs(txs[i] - tx_acc);
// 		rmse_y += fabs(tys[i] - ty_acc);
// 	}
// 
// 	rmse_x /= num;
// 	rmse_y /= num;
// 
// 	tx = tx_acc; 
// 	ty = ty_acc;
// 
// // 	while (rmse_x>0.2 || rmse_y>0.2)
// // 	{
// // 		tx_acc = 0, ty_acc = 0;
// // 		double wx_sum = 0, wy_sum = 0;
// // 		for (int i = 0; i < num; i++)
// // 		{
// // 			double wx, wy;
// // 			wx = exp(-fabs(txs[i] - tx_acc) / (1.5*rmse_x));
// // 			wy = exp(-fabs(tys[i] - ty_acc) / (1.5*rmse_y));
// // 			tx_acc += wx*txs[i];
// // 			ty_acc += wy*tys[i];
// // 			wx_sum += wx;
// // 			wy_sum += wy;
// // 		}
// // 		tx_acc /= wx_sum;
// // 		ty_acc /= wy_sum;
// // 
// // 		rmse_x = 0, rmse_y = 0;
// // 		for (int i = 0; i < num; i++)
// // 		{
// // 			double wx, wy;
// // 			wx = exp(-fabs(txs[i] - tx_acc) / (1.5*rmse_x));
// // 			wy = exp(-fabs(tys[i] - ty_acc) / (1.5*rmse_y));
// // 			rmse_x += wx*fabs(txs[i] - tx_acc);
// // 			rmse_y += wy*fabs(tys[i] - ty_acc);
// // 		}
// // 
// // 		rmse_x /= wx_sum;
// // 		rmse_y /= wy_sum;
// // 	}
// 
// 	//free memory
// 	txs.clear();
// 	tys.clear();
// 
// 	vector<double>().swap(txs);
// 	vector<double>().swap(tys);
// 
// 	return 1;
// }
// 
// 

int CRPC_Fun::SortPatches(Long_Strip_Img strip)
{
	int coverNum = strip.patchNum;
	if (coverNum==0)
	{
		return 0;
	}
	else if (coverNum == 1)
	{
		return 1;
	}

 	RPC * rpcs = new RPC[coverNum];
	for (int i = 0; i < coverNum; i++)
	{
		ReadRpc_singleFile(strip.rpb_paths[i], rpcs[i]);
	}
	for (int i = 0; i < coverNum - 1; i++)
		for (int j = coverNum - 1; j >= i + 1;j--)
		{
			int tx, ty;
			ComputeTranslateBetweenTiles(strip.img_paths[j - 1], strip.rpb_paths[j - 1], strip.img_paths[j], strip.rpb_paths[j], tx, ty);

			if (ty<0)
			{
				char str[1024];
				strcpy(str, strip.img_paths[j - 1]);
				strcpy(strip.img_paths[j - 1], strip.img_paths[j]);
				strcpy(strip.img_paths[j], str);

				strcpy(str, strip.rpb_paths[j - 1]);
				strcpy(strip.rpb_paths[j - 1], strip.rpb_paths[j]);
				strcpy(strip.rpb_paths[j], str);
 			}
		}

	//free memory
 	delete[]rpcs; rpcs = NULL;
	return 1;
}

int CRPC_Fun::Derivate_RPC_var(RPC para, BiasPara_constant bias_para, double row, double col, double Lat, double Lon, double Hei,
	char *fun_option, char * var_option, double &result)
{
	//normalize the coordinates of Lat, Lon, Hei
	double P, L, H;
	P = (Lat - para.latOffset) / para.latScale;
	L = (Lon - para.longOffset) / para.longScale;
	H = (Hei - para.heightOffset) / para.heightScale;

	if (strcmp(fun_option,"row")==0)
	{
		if (strcmp(var_option, "a") == 0)
		{
			result = row;
		}
		else if (strcmp(var_option, "b") == 0)
		{
			result = col;
		}
		else if (strcmp(var_option, "c") == 0)
		{
			result = 1;
		}
		else if (strcmp(var_option, "d") == 0)
		{
			result = 0;
		}
		else if (strcmp(var_option, "e") == 0)
		{
			result = 0;
		}
		else if (strcmp(var_option, "f") == 0)
		{
			result = 0;
		}
		else if (strcmp(var_option, "Lat") == 0)
		{
			double Line_num = 0, Line_den = 0; 
			double Line_num_der_P = 0, Line_den_der_P = 0;
			double P_der_Lat = 0;
			for (int i = 0; i < 20; i++)
			{
				double pi;
				RPC_coefficient_derivative(P, L, H, i + 1, 'P', pi);
				Line_num_der_P += para.lineNumCoef[i] * pi;
				Line_den_der_P += para.lineDenCoef[i] * pi;

				RPC_coefficient(P, L, H, i + 1, pi);
				Line_num += para.lineNumCoef[i] * pi;
				Line_den += para.lineDenCoef[i] * pi;
			}

			P_der_Lat = 1 / para.latScale;
			result = (Line_num_der_P / Line_den - Line_den_der_P*Line_num / (Line_den*Line_den)) * P_der_Lat;
			result = result*(-para.lineScale);
		}
		else if (strcmp(var_option, "Lon") == 0)
		{
			double Line_num = 0, Line_den = 0;
			double Line_num_der_L = 0, Line_den_der_L = 0;
			double L_der_Lon = 0;
			for (int i = 0; i < 20; i++)
			{
				double pi;
				RPC_coefficient_derivative(P, L, H, i + 1, 'L', pi);
				Line_num_der_L += para.lineNumCoef[i] * pi;
				Line_den_der_L += para.lineDenCoef[i] * pi;

				RPC_coefficient(P, L, H, i + 1, pi);
				Line_num += para.lineNumCoef[i] * pi;
				Line_den += para.lineDenCoef[i] * pi;
			}

			L_der_Lon = 1 / para.longScale;
			result = (Line_num_der_L / Line_den - Line_den_der_L*Line_num / (Line_den*Line_den)) * L_der_Lon;
			result = result*(-para.lineScale);
		}
		else if (strcmp(var_option, "Hei") == 0)
		{
			double Line_num = 0, Line_den = 0;
			double Line_num_der_H = 0, Line_den_der_H = 0;
			double H_der_Hei = 0;
			for (int i = 0; i < 20; i++)
			{
				double pi;
				RPC_coefficient_derivative(P, L, H, i + 1, 'H', pi);
				Line_num_der_H += para.lineNumCoef[i] * pi;
				Line_den_der_H += para.lineDenCoef[i] * pi;

				RPC_coefficient(P, L, H, i + 1, pi);
				Line_num += para.lineNumCoef[i] * pi;
				Line_den += para.lineDenCoef[i] * pi;
			}

			H_der_Hei = 1 / para.heightScale;
			result = (Line_num_der_H / Line_den - Line_den_der_H*Line_num / (Line_den*Line_den)) * H_der_Hei;
			result = result*(-para.lineScale);
		}
	}
	else if (strcmp(fun_option,"col")==0)
	{
		if (strcmp(var_option, "a") == 0)
		{
			result = 0;
		}
		else if (strcmp(var_option, "b") == 0)
		{
			result = 0;
		}
		else if (strcmp(var_option, "c") == 0)
		{
			result = 0;
		}
		else if (strcmp(var_option, "d") == 0)
		{
			result = row;
		}
		else if (strcmp(var_option, "e") == 0)
		{
			result = col;
		}
		else if (strcmp(var_option, "f") == 0)
		{
			result = 1;
		}
		else if (strcmp(var_option, "Lat") == 0)
		{
			double Samp_num = 0, Samp_den = 0;
			double Samp_num_der_P = 0, Samp_den_der_P = 0;
			double P_der_Lat = 0;

			for (int i = 0; i < 20; i++)
			{
				double pi;
				RPC_coefficient_derivative(P, L, H, i + 1, 'P', pi);
				Samp_num_der_P += para.sampNumCoef[i] * pi;
				Samp_den_der_P += para.sampDenCoef[i] * pi;

				RPC_coefficient(P, L, H, i + 1, pi);
				Samp_num += para.sampNumCoef[i] * pi;
				Samp_den += para.sampDenCoef[i] * pi;
			}

			P_der_Lat = 1 / para.latScale;
			result = (Samp_num_der_P / Samp_den - Samp_den_der_P*Samp_num / (Samp_den*Samp_den)) * P_der_Lat;
			result = result*(-para.sampScale);
		}
		else if (strcmp(var_option, "Lon") == 0)
		{
			double Samp_num = 0, Samp_den = 0;
			double Samp_num_der_L = 0, Samp_den_der_L = 0;
			double L_der_Lon = 0;

			for (int i = 0; i < 20; i++)
			{
				double pi;
				RPC_coefficient_derivative(P, L, H, i + 1, 'L', pi);
				Samp_num_der_L += para.sampNumCoef[i] * pi;
				Samp_den_der_L += para.sampDenCoef[i] * pi;

				RPC_coefficient(P, L, H, i + 1, pi);
				Samp_num += para.sampNumCoef[i] * pi;
				Samp_den += para.sampDenCoef[i] * pi;
			}

			L_der_Lon = 1 / para.longScale;
			result = (Samp_num_der_L / Samp_den - Samp_den_der_L*Samp_num / (Samp_den*Samp_den)) * L_der_Lon;
			result = result*(-para.sampScale);
		}
		else if (strcmp(var_option, "Hei") == 0)
		{
			double Samp_num = 0, Samp_den = 0;
			double Samp_num_der_H = 0, Samp_den_der_H = 0;
			double H_der_Hei = 0;
			for (int i = 0; i < 20; i++)
			{
				double pi;
				RPC_coefficient_derivative(P, L, H, i + 1, 'H', pi);
				Samp_num_der_H += para.lineNumCoef[i] * pi;
				Samp_den_der_H += para.lineDenCoef[i] * pi;

				RPC_coefficient(P, L, H, i + 1, pi);
				Samp_num += para.lineNumCoef[i] * pi;
				Samp_den += para.lineDenCoef[i] * pi;
			}

			H_der_Hei = 1 / para.heightScale;
			result = (Samp_num_der_H / Samp_den - Samp_den_der_H*Samp_num / (Samp_den*Samp_den)) * H_der_Hei;
			result = result*(-para.sampScale);
		}
	}
	return 1;
}

int CRPC_Fun::Compute_Taylor_Constant_Term(double aff_para1, double aff_para2, double aff_para3, RPC rpc_para, char *option,
	double row, double col, double Lat, double Lon, double Hei,
	double &constant_result)
{
	constant_result = 0;
	double P, L, H;
	P = (Lat - rpc_para.latOffset) / rpc_para.latScale;
	L = (Lon - rpc_para.longOffset) / rpc_para.longScale;
	H = (Hei - rpc_para.heightOffset) / rpc_para.heightScale;

	double r, c;
	RPC_Ground2Image_normalize(P, L, H, rpc_para, r, c);

	if (strcmp(option,"row")==0)
	{
		double off, scale;
		off = rpc_para.lineOffset;
		scale = rpc_para.lineScale;
		
		constant_result += (aff_para1*row + aff_para2*col + aff_para3);
		constant_result -= scale*r;
		constant_result -= (off - row);
	}
	else if (strcmp(option,"col")==0)
	{
		double off, scale;
		off = rpc_para.sampOffset;
		scale = rpc_para.sampScale;

		constant_result += (aff_para1*row + aff_para2*col + aff_para3);
		constant_result -= scale*c;
		constant_result -= (off - col);
	}

	return 1;
}

//////////////////////////////////////////////////////////////////////////Test NA 测试结果完全正确
/*double *A_test = new double[(2 * imgPtNum)*(2 * imgNum)];
memset(A_test, 0, sizeof(double)*(2 * imgPtNum)*(2 * imgNum));

int iter = 0;
for (int i = 0; i < ptNum; i++)
{
int trackNum = 0;
trackNum = pts[i].ptNum;
double Lat, Lon, Hei;
Lat = pts[i].Lat;
Lon = pts[i].Lon;
Hei = pts[i].Hei;
for (int j = 0; j < trackNum; j++)
{
char *imgID_str = pts[i].img_pts[j].imgID;
int imgID = GetImgID(imgIDs, imgNum, imgID_str);

double row, col;
row = pts[i].img_pts[j].y;
col = pts[i].img_pts[j].x;

Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "c", A[0]);
A[1] = 0;

A[2] = 0;
Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "f", A[3]);

A_test[(iter * 2 + 0)*(2 * imgNum) + (imgID * 2 + 0)] = A[0];
A_test[(iter * 2 + 1)*(2 * imgNum) + (imgID * 2 + 1)] = A[3];
iter++;
}
}

double *AT_test = new double[(2 * imgNum)*(2 * imgPtNum)];
matc.TransposeMatrix(A_test, 2 * imgPtNum, 2 * imgNum, AT_test);

double *ATA_test = new double[(2 * imgNum)*(2 * imgNum)];
matc.MultiMatrix(AT_test, A_test, ATA_test, 2 * imgNum, 2 * imgNum, 2 * imgPtNum);

double dif = 0;
for (int i = 0; i < 2 * imgNum * 2 * imgNum;i++)
{
dif += fabs(NA[i] - ATA_test[i]);
}

delete[]A_test;
delete[]AT_test;
delete[]ATA_test;*/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////Test LA
// 	double *A_test = new double[(2 * imgPtNum)*(2 * imgNum)];
// 	memset(A_test, 0, sizeof(double)*(2 * imgPtNum)*(2 * imgNum));
// 
// 	int iter = 0;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		int trackNum = 0;
// 		trackNum = pts[i].ptNum;
// 		double Lat, Lon, Hei;
// 		Lat = pts[i].Lat;
// 		Lon = pts[i].Lon;
// 		Hei = pts[i].Hei;
// 		for (int j = 0; j < trackNum; j++)
// 		{
// 			char *imgID_str = pts[i].img_pts[j].imgID;
// 			int imgID = GetImgID(imgIDs, imgNum, imgID_str);
// 
// 			double row, col;
// 			row = pts[i].img_pts[j].y;
// 			col = pts[i].img_pts[j].x;
// 
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "c", A[0]);
// 			A[1] = 0;
// 
// 			A[2] = 0;
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "f", A[3]);
// 
// 			A_test[(iter * 2 + 0)*(2 * imgNum) + (imgID * 2 + 0)] = A[0];
// 			A_test[(iter * 2 + 1)*(2 * imgNum) + (imgID * 2 + 1)] = A[3];
// 			iter++;
// 		}
// 	}
// 
// 	int total_img_pt_num = iter * 2;
// 	double *L_test;
// 	L_test = new double[total_img_pt_num];
// 	memset(L_test, 0, sizeof(double)*total_img_pt_num);
// 
// 	iter = 0;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		int trackNum = 0;
// 		trackNum = pts[i].ptNum;
// 		double Lat, Lon, Hei;
// 		Lat = pts[i].Lat;
// 		Lon = pts[i].Lon;
// 		Hei = pts[i].Hei;
// 		for (int j = 0; j < trackNum; j++)
// 		{
// 			char *imgID_str = pts[i].img_pts[j].imgID;
// 			int imgID = GetImgID(imgIDs, imgNum, imgID_str);
// 
// 			double row, col;
// 			row = pts[i].img_pts[j].y;
// 			col = pts[i].img_pts[j].x;
// 
// 			double l_constant[2];
// 			//row
// 			Compute_Taylor_Constant_Term(0, 0, bias_para0[imgID].drow, rpc_paras[imgID], "row", row, col, Lat, Lon, Hei, l_constant[0]);
// 			l_constant[0] = -l_constant[0];
// 			//col
// 			Compute_Taylor_Constant_Term(0, 0, bias_para0[imgID].dcol, rpc_paras[imgID], "col", row, col, Lat, Lon, Hei, l_constant[1]);
// 			l_constant[1] = -l_constant[1];
// 
// 			L_test[iter] = l_constant[0];
// 			iter++;
// 			L_test[iter] = l_constant[1];
// 			iter++;
// 		}
// 	}
// 
// 	double *ATL_test = new double[2 * imgNum * 1];
// 	double *AT_test = new double[(2 * imgNum)*(2 * imgPtNum)];
// 	matc.TransposeMatrix(A_test, 2 * imgPtNum, 2 * imgNum, AT_test);
// 	matc.MultiMatrix(AT_test, L_test, ATL_test, 2 * imgNum, 1, 2 * imgPtNum);
// 
// 	double result = 0;
// 	for (int i = 0; i < 2 * imgNum; i++)
// 	{
// 		result += fabs(ATL_test[i] - LA[i]);
// 	}
// 
// 	delete[]ATL_test; ATL_test = NULL;
// 	delete[]A_test; A_test = NULL;
// 	delete[]L_test; L_test = NULL;
// 	delete[]AT_test; AT_test = NULL;
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////Test NB almost the same
// 	double *B_test = new double[(2 * imgPtNum)*(3 * ptNum)];
// 	memset(B_test, 0, sizeof(double)*(2 * imgPtNum)*(3 * ptNum));
// 
// 	int iter = 0;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		int trackNum = 0;
// 		trackNum = pts[i].ptNum;
// 		double Lat, Lon, Hei;
// 		Lat = pts[i].Lat;
// 		Lon = pts[i].Lon;
// 		Hei = pts[i].Hei;
// 		for (int j = 0; j < trackNum; j++)
// 		{
// 			char *imgID_str = pts[i].img_pts[j].imgID;
// 			int imgID = GetImgID(imgIDs, imgNum, imgID_str);
// 
// 			double row, col;
// 			row = pts[i].img_pts[j].y;
// 			col = pts[i].img_pts[j].x;
// 
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lat", B[0]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lon", B[1]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Hei", B[2]);
// 
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lat", B[3]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lon", B[4]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Hei", B[5]);
// 
// 			matc.TransposeMatrix(B, 2, 3, BT);
// 			matc.MultiMatrix(BT, B, BTB, 3, 3, 2);
// 
// 			B_test[iter*(3 * ptNum) + (i * 3 + 0)] = B[0];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 1)] = B[1];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 2)] = B[2];
// 			iter++;
// 			B_test[iter*(3 * ptNum) + (i * 3 + 0)] = B[3];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 1)] = B[4];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 2)] = B[5];
// 			iter++;
// 		}
// 	}
// 
// 	double *BT_test = new double[(3 * ptNum)*(2 * imgPtNum)];
// 	double *BTB_test = new double[(3 * ptNum)*(3 * ptNum)];
// 	matc.TransposeMatrix(B_test, 2 * imgPtNum, 3 * ptNum, BT_test);
// 	matc.MultiMatrix(BT_test, B_test, BTB_test, 3 * ptNum, 3 * ptNum, 2 * imgPtNum);
// 
// 	double result = 0;
// 	for (int i = 0; i < ptNum;i++)
// 	{
// 		result += fabs(NB[i * 9 + 0] - BTB_test[(i * 3 + 0)*(3 * ptNum) + (i * 3 + 0)]);
// 		result += fabs(NB[i * 9 + 1] - BTB_test[(i * 3 + 0)*(3 * ptNum) + (i * 3 + 1)]);
// 		result += fabs(NB[i * 9 + 2] - BTB_test[(i * 3 + 0)*(3 * ptNum) + (i * 3 + 2)]);
// 
// 		result += fabs(NB[i * 9 + 3] - BTB_test[(i * 3 + 1)*(3 * ptNum) + (i * 3 + 0)]);
// 		result += fabs(NB[i * 9 + 4] - BTB_test[(i * 3 + 1)*(3 * ptNum) + (i * 3 + 1)]);
// 		result += fabs(NB[i * 9 + 5] - BTB_test[(i * 3 + 1)*(3 * ptNum) + (i * 3 + 2)]);
// 
// 		result += fabs(NB[i * 9 + 6] - BTB_test[(i * 3 + 2)*(3 * ptNum) + (i * 3 + 0)]);
// 		result += fabs(NB[i * 9 + 7] - BTB_test[(i * 3 + 2)*(3 * ptNum) + (i * 3 + 1)]);
// 		result += fabs(NB[i * 9 + 8] - BTB_test[(i * 3 + 2)*(3 * ptNum) + (i * 3 + 2)]);
// 	}
// 
// 	printf("diff = %lf\n", result);
// 	return 1;
// 	delete[]B_test; B_test = NULL;
// 	delete[]BT_test; BT_test = NULL;
// 	delete[]BTB_test; BTB_test = NULL;
//////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////// exactly the same
//test the NB_inverse
// double diff = 0;
// for (int i = 0; i < ptNum; i++)
// {
// 	double BTB_test[9];
// 	double BTB_inverse_test[9];
// 
// 	for (int j = 0; j < 9; j++)
// 	{
// 		BTB_test[j] = NB[i * 9 + j];
// 		BTB_inverse_test[j] = NB_inverse[i * 9 + j];
// 	}
// 
// 	double E[9];
// 	matc.MultiMatrix(BTB_test, BTB_inverse_test, E, 3, 3, 3);
// 
// 	diff += (fabs(E[0] - 1) + fabs(E[4] - 1) + fabs(E[8] - 1));
// }
// 
// int aaa = 1;
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////check the correctness of N_bias and L_bias, there is sth. wrong in it
// 	double *A_test = new double[(2 * imgPtNum)*(2 * imgNum)];
// 	double *AT_test = new double[(2 * imgNum)*(2 * imgPtNum)];
// 	double *B_test = new double[(2 * imgPtNum)*(3 * ptNum)];
// 	double *AtB_test = new double[(2 * imgNum)*(3 * ptNum)];
// 	double *BtA_test = new double[(3 * ptNum)*(2 * imgNum)];
// 	double *BTB_inverse = new double[(3 * ptNum)*(3 * ptNum)];
// 	double *ATB_BTB_inverse = new double[(2 * imgNum)*(3 * ptNum)];
// 	double *AB_test = new double[(2 * imgNum)*(2 * imgNum)];
// 	double *NAB_LB = new double[2 * imgNum * 1];
// 	double *L_bias_test = new double[2 * imgNum];
// 	double *NAB_B_inverse = new double[(2 * imgNum)*(3 * ptNum)];
// 
// 	memset(A_test, 0, sizeof(double)*(2 * imgPtNum)*(2 * imgNum));
// 	memset(AT_test, 0, sizeof(double)*(2 * imgNum)*(2 * imgPtNum));
// 	memset(B_test, 0, sizeof(double)*(2 * imgPtNum)*(3 * ptNum));
// 	memset(AtB_test, 0, sizeof(double)*(2 * imgNum)*(3 * ptNum));
// 	memset(BtA_test, 0, sizeof(double)*(3 * ptNum)*(2 * imgNum));
// 	memset(BTB_inverse, 0, sizeof(double)*(3 * ptNum)*(3 * ptNum));
// 	memset(ATB_BTB_inverse, 0, sizeof(double)*(2 * imgNum)*(3 * ptNum));
// 	memset(AB_test, 0, sizeof(double)*(2 * imgNum)*(2 * imgNum));
// 	memset(NAB_LB, 0, sizeof(double)*(2 * imgNum)*1);
// 	memset(L_bias_test, 0, sizeof(double)*(2 * imgNum) * 1);
// 	memset(NAB_B_inverse, 0, sizeof(double)*(2 * imgNum)*(3 * ptNum));
// 
// 	int iter = 0;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		int trackNum = 0;
// 		trackNum = pts[i].ptNum;
// 		double Lat, Lon, Hei;
// 		Lat = pts[i].Lat;
// 		Lon = pts[i].Lon;
// 		Hei = pts[i].Hei;
// 		int *imgID_array = new int[trackNum];
// 		for (int j = 0; j < trackNum; j++)
// 		{
// 			char *imgID_str = pts[i].img_pts[j].imgID;
// 			imgID_array[j] = GetImgID(imgIDs, imgNum, imgID_str);
// 		}
// 
// 		for (int j = 0; j < trackNum; j++)
// 		{
// 			int imgID = imgID_array[j];
// 			double row, col;
// 			row = pts[i].img_pts[j].y;
// 			col = pts[i].img_pts[j].x;
// 
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "c", A[0]);
// 			A[1] = 0;
// 
// 			A[2] = 0;
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "f", A[3]);
// 
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lat", B[0]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lon", B[1]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Hei", B[2]);
// 
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lat", B[3]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lon", B[4]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Hei", B[5]);
// 
// 			A_test[iter*(2 * imgNum) + (imgID * 2 + 0)] = A[0];
// 			A_test[iter*(2 * imgNum) + (imgID * 2 + 1)] = A[1];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 0)] = B[0];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 1)] = B[1];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 2)] = B[2];
// 			iter++;
// 			A_test[iter*(2 * imgNum) + (imgID * 2 + 0)] = A[2];
// 			A_test[iter*(2 * imgNum) + (imgID * 2 + 1)] = A[3];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 0)] = B[3];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 1)] = B[4];
// 			B_test[iter*(3 * ptNum) + (i * 3 + 2)] = B[5];
// 			iter++;
// 		}
// 
// 		delete[]imgID_array; imgID_array = NULL;
// 	}
// 
// 	matc.TransposeMatrix(A_test, 2 * imgPtNum, 2 * imgNum, AT_test);
// 	matc.MultiMatrix(AT_test, B_test, AtB_test, 2 * imgNum, 3 * ptNum, 2 * imgPtNum);
// 	matc.TransposeMatrix(AtB_test, 2 * imgNum, 3 * ptNum, BtA_test);
// 
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		BTB_inverse[(i * 3 + 0)*(3 * ptNum) + (i * 3 + 0)] = NB_inverse[i * 9 + 0];
// 		BTB_inverse[(i * 3 + 0)*(3 * ptNum) + (i * 3 + 1)] = NB_inverse[i * 9 + 1];
// 		BTB_inverse[(i * 3 + 0)*(3 * ptNum) + (i * 3 + 2)] = NB_inverse[i * 9 + 2];
// 
// 		BTB_inverse[(i * 3 + 1)*(3 * ptNum) + (i * 3 + 0)] = NB_inverse[i * 9 + 3];
// 		BTB_inverse[(i * 3 + 1)*(3 * ptNum) + (i * 3 + 1)] = NB_inverse[i * 9 + 4];
// 		BTB_inverse[(i * 3 + 1)*(3 * ptNum) + (i * 3 + 2)] = NB_inverse[i * 9 + 5];
// 
// 		BTB_inverse[(i * 3 + 2)*(3 * ptNum) + (i * 3 + 0)] = NB_inverse[i * 9 + 6];
// 		BTB_inverse[(i * 3 + 2)*(3 * ptNum) + (i * 3 + 1)] = NB_inverse[i * 9 + 7];
// 		BTB_inverse[(i * 3 + 2)*(3 * ptNum) + (i * 3 + 2)] = NB_inverse[i * 9 + 8];
// 	}
// 
// 	matc.MultiMatrix(AtB_test, BTB_inverse, ATB_BTB_inverse, 2 * imgNum, 3 * ptNum, 3 * ptNum);
// 	matc.MultiMatrix(ATB_BTB_inverse, BtA_test, AB_test, 2 * imgNum, 2 * imgNum, 3 * ptNum);
// 
// 	double *N_bias_test = new double[2 * imgNum * 2 * imgNum];
// 	memset(N_bias_test, 0, sizeof(double) * 2 * imgNum * 2 * imgNum);
// 	for (int i = 0; i < 2 * imgNum * 2 * imgNum; i++)
// 	{
// 		N_bias_test[i] = NA[i] - AB_test[i];
// 	}
// 
// 	double diff = 0;
// 	for (int i = 0; i < 2 * imgNum * 2 * imgNum; i++)
// 	{
// 		diff += fabs(N_bias_test[i] - N_bias[i]);
// 	}
// 
// 	matc.MultiMatrix(ATB_BTB_inverse, LB, NAB_LB, 2 * imgNum, 1, 3 * ptNum);
// 	for (int i = 0; i < 2 * imgNum;i++)
// 	{
// 		L_bias_test[i] = LA[i] - NAB_LB[i];
// 	}
// 
// 	diff = 0;
// 	for (int i = 0; i<2 * imgNum; i++)
// 	{
// 		diff += fabs(L_bias_test[i] - L_bias[i]);
// 	}
// 
// 	delete[]A_test; A_test = NULL;
// 	delete[]B_test; B_test = NULL;
// 	delete[]AT_test; AT_test = NULL;
// 	delete[]AtB_test; AtB_test = NULL;
// 	delete[]BtA_test; BtA_test = NULL;
// 	delete[]BTB_inverse; BTB_inverse = NULL;
// 	delete[]ATB_BTB_inverse; ATB_BTB_inverse = NULL;
// 	delete[]AB_test; AB_test = NULL;
// 	delete[]N_bias_test; N_bias_test = NULL;
// 	delete[]NAB_LB; NAB_LB = NULL;
// 	delete[]L_bias_test; L_bias_test = NULL;
// 	delete[]NAB_B_inverse; NAB_B_inverse = NULL;
//////////////////////////////////////////////////////////////////////////

int CRPC_Fun::ComputeNormalEquations_constant(Ground_Img_Pt *pts, int ptNum, RPC *rpc_paras, BiasPara_constant *bias_para0, int imgNum, char **imgIDs,
	double RMSE,
	double *&N_bias, double *&L_bias)
{
	//define object class
	CSimple_Matrix_Computation matc;

	int imgPtNum = 0;
	for (int i = 0; i < ptNum;i++)
	{
		int trackNum = 0;
		trackNum = pts[i].ptNum;
		imgPtNum += trackNum;
	}

	double sigma_coef = 10000000000/*1.5*/;
	//1.  compute NA and LA
	double *NA = new double[(2*imgNum)*(2*imgNum)];
	memset(NA, 0, sizeof(double) *(2 * imgNum)*(2 * imgNum));
	
	double *A = new double[2 * 2];
	memset(A, 0, sizeof(double) * 2 * 2);
	double *AT = new double[2 * 2];
	memset(AT, 0, sizeof(double) * 2 * 2);
	double *ATA = new double[2 * 2];
	memset(ATA, 0, sizeof(double) * 2 * 2);
	for (int i = 0; i < ptNum;i++)
	{
		int trackNum = 0;
		trackNum = pts[i].ptNum;
		double Lat, Lon, Hei;
		Lat = pts[i].Lat;
		Lon = pts[i].Lon;
		Hei = pts[i].Hei;

		for (int j = 0; j < trackNum;j++)
		{
			char *imgID_str = pts[i].img_pts[j].imgID;
			int imgID = GetImgID(imgIDs, imgNum, imgID_str);

			double row, col;
			row = pts[i].img_pts[j].y;
			col = pts[i].img_pts[j].x;

			double deltax, deltay;
			deltax = pts[i].img_pts[j].deltax;
			deltay = pts[i].img_pts[j].deltay;
			double dis_xy = sqrt(deltax*deltax + deltay*deltay);
			double wei = exp(-dis_xy / (sigma_coef * RMSE));
			wei = sqrt(wei);

			//////////////////////////////////////////////////////////////////////////
// 			if (trackNum == 2)
// 			{
// 				wei = 0.3;
// 			}
// 			else if (trackNum == 3)
// 			{
// 				wei = 0.6;
// 			}
// 			else
				wei = 1;
			//////////////////////////////////////////////////////////////////////////

			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "c", A[0]);
			A[1] = 0;
			A[2] = 0;
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "f", A[3]);
			if (RMSE<1)
			{
				for (int mm = 0; mm < 4; mm++)
				{
					A[mm] = A[mm] * wei;
				}
			}

			matc.TransposeMatrix(A, 2, 2, AT);
			matc.MultiMatrix(AT, A, ATA, 2, 2, 2);

			NA[(imgID * 2 + 0) * (imgNum * 2) + (imgID * 2 + 0)] += ATA[0];
			NA[(imgID * 2 + 1) * (imgNum * 2) + (imgID * 2 + 1)] += ATA[3];
		}
	}

	//Add smooth item
// 	int smooth_term_num = 0;
// 	for (int i = 0; i < imgNum;i++)
// 	{
// 		char *cur_ID = imgIDs[i];
// 		int stripID_cur, patch_ID_cur;
// 		sscanf(cur_ID, "%d_%d", &stripID_cur, &patch_ID_cur);
// 		for (int j = 0; j < imgNum;j++)
// 		{
// 			if (i==j)
// 			{
// 				continue;
// 			}
// 			else
// 			{
// 				char *nxt_ID = imgIDs[j];
// 				int stripID_nxt, patch_ID_nxt;
// 				sscanf(nxt_ID, "%d_%d", &stripID_nxt, &patch_ID_nxt);
// 
// 				if (stripID_nxt == stripID_cur)
// 				{
// 					if (patch_ID_nxt - patch_ID_cur == 1)
// 					{
// 						smooth_term_num++;
// 					}
// 				}
// 			}
// 		}
// 	}
// 
// 	
// 	if (smooth_term_num!=0)
// 	{
// 		double *S = new double[(2 * smooth_term_num)*(2 * imgNum)];
// 		memset(S, 0, sizeof(double)*(2 * smooth_term_num)*(2 * imgNum));
// 
// 		//create S matrix for smooth term
// 		int iter = 0;
// 		for (int i = 0; i < imgNum; i++)
// 		{
// 			char *cur_ID = imgIDs[i];
// 			int stripID_cur, patch_ID_cur;
// 			sscanf(cur_ID, "%d_%d", &stripID_cur, &patch_ID_cur);
// 			for (int j = 0; j < imgNum; j++)
// 			{
// 				if (i == j)
// 				{
// 					continue;
// 				}
// 				else
// 				{
// 					char *nxt_ID = imgIDs[j];
// 					int stripID_nxt, patch_ID_nxt;
// 					sscanf(nxt_ID, "%d_%d", &stripID_nxt, &patch_ID_nxt);
// 
// 					if (stripID_nxt == stripID_cur)
// 					{
// 						if (patch_ID_nxt - patch_ID_cur == 1)
// 						{
// 							S[(iter * 2 + 0)*(2 * imgNum) + (i * 2 + 0)] = 1;
// 							S[(iter * 2 + 0)*(2 * imgNum) + (j * 2 + 0)] = -1;
// 
// 							S[(iter * 2 + 1)*(2 * imgNum) + (i * 2 + 1)] = 1;
// 							S[(iter * 2 + 1)*(2 * imgNum) + (j * 2 + 1)] = -1;
// 
// 							iter++;
// 						}
// 					}
// 				}
// 			}
// 		}
// 
// 		double *ST, *STS;
// 		ST = new double[(2 * imgNum)*(2 * smooth_term_num)];
// 		STS = new double[(2 * imgNum) * (2 * imgNum)];
// 		matc.TransposeMatrix(S, 2 * smooth_term_num, 2 * imgNum, ST);
// 		matc.MultiMatrix(ST, S, STS, 2 * imgNum, 2 * imgNum, 2 * smooth_term_num);
// 
// 		double *NA_STS = new double[(2 * imgNum) * (2 * imgNum)];
// 		matc.Matrix_Add(NA, STS, NA_STS, 2 * imgNum, 2 * imgNum);
// 
// 		memcpy(NA, NA_STS, sizeof(double)*(2 * imgNum)*(2 * imgNum));
// 
// 		//free memory
// 		delete[]S; S = NULL;
// 		delete[]ST; ST = NULL;
// 		delete[]STS; STS = NULL;
// 		delete[]NA_STS; NA_STS = NULL;
// 	}
	//////////////////////////////////////////////////////////////////////////add constraint
// 	int effectNum = 10;
// 	int baseID[5] = { 3, 4, 0, 1, 2 };
// 	double P[5] = { 0.1, 0.1, 0.1, 0.1, 0.1 };
// 	double *S = new double[effectNum * (2 * imgNum)];
// 	double *ST = new double[(2 * imgNum) * effectNum];
// 	double *STS = new double[(2 * imgNum)*(2 * imgNum)];
// 	memset(S, 0, sizeof(double) * effectNum * 2 * imgNum);
// 	memset(ST, 0, sizeof(double) * effectNum * 2 * imgNum);
// 	memset(STS, 0, sizeof(double) * (2 * imgNum)*(2 * imgNum));
// 	S[0 * (2 * imgNum) + (baseID[0] * 2 + 0)] = P[0];
// 	S[1 * (2 * imgNum) + (baseID[0] * 2 + 1)] = P[0];
// 	S[2 * (2 * imgNum) + (baseID[1] * 2 + 0)] = P[1];
// 	S[3 * (2 * imgNum) + (baseID[1] * 2 + 1)] = P[1];
// 	S[4 * (2 * imgNum) + (baseID[2] * 2 + 0)] = P[2];
// 	S[5 * (2 * imgNum) + (baseID[2] * 2 + 1)] = P[2];
// 	S[6 * (2 * imgNum) + (baseID[3] * 2 + 0)] = P[3];
// 	S[7 * (2 * imgNum) + (baseID[3] * 2 + 1)] = P[3];
// 	S[8 * (2 * imgNum) + (baseID[4] * 2 + 0)] = P[4];
// 	S[9 * (2 * imgNum) + (baseID[4] * 2 + 1)] = P[4];
// 	matc.TransposeMatrix(S, effectNum, imgNum * 2, ST);
// 	matc.MultiMatrix(ST, S, STS, 2 * imgNum, 2 * imgNum, effectNum);
// 
// 	for (int i = 0; i<(2 * imgNum)*(2 * imgNum);i++)
// 	{
// 		NA[i] += STS[i];
// 	}
// 
// 	delete[]S; S = NULL;
// 	delete[]ST; ST = NULL;
// 	delete[]STS; STS = NULL;
	//////////////////////////////////////////////////////////////////////////

	//compute LA
	double *LA = new double[(2 * imgNum) * 1];
	memset(LA, 0, sizeof(double)*(2 * imgNum));

	for (int i = 0; i < ptNum; i++)
	{
		int trackNum = 0;
		trackNum = pts[i].ptNum;
		double Lat, Lon, Hei;
		Lat = pts[i].Lat;
		Lon = pts[i].Lon;
		Hei = pts[i].Hei;
		for (int j = 0; j < trackNum; j++)
		{
			char *imgID_str = pts[i].img_pts[j].imgID;
			int imgID = GetImgID(imgIDs, imgNum, imgID_str);

			double row, col;
			row = pts[i].img_pts[j].y;
			col = pts[i].img_pts[j].x;

			double deltax, deltay;
			deltax = pts[i].img_pts[j].deltax;
			deltay = pts[i].img_pts[j].deltay;
			double dis_xy = sqrt(deltax*deltax + deltay*deltay);
			double wei = exp(-dis_xy / (sigma_coef * RMSE));
			wei = sqrt(wei);

			//////////////////////////////////////////////////////////////////////////
// 			if (trackNum == 2)
// 			{
// 				wei = 0.3;
// 			}
// 			else if (trackNum == 3)
// 			{
// 				wei = 0.6;
// 			}
// 			else
				wei = 1;
			//////////////////////////////////////////////////////////////////////////

			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "c", A[0]);
			A[1] = 0;
			A[2] = 0;
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "f", A[3]);
			if (RMSE < 1)
			{
				for (int mm = 0; mm < 4; mm++)
				{
					A[mm] = A[mm] * wei;
				}
			}

			matc.TransposeMatrix(A, 2, 2, AT);
			
			double l_constant[2];
			//row
			Compute_Taylor_Constant_Term(0, 0, bias_para0[imgID].drow, rpc_paras[imgID], "row", row, col, Lat, Lon, Hei, l_constant[0]);
			l_constant[0] = -l_constant[0];
			//col
			Compute_Taylor_Constant_Term(0, 0, bias_para0[imgID].dcol, rpc_paras[imgID], "col", row, col, Lat, Lon, Hei, l_constant[1]);
			l_constant[1] = -l_constant[1];
			if (RMSE < 1)
			{
				for (int mm = 0; mm < 2; mm++)
				{
					l_constant[mm] = l_constant[mm] * wei;
				}
			}

			double ATl[2];
			matc.MultiMatrix(AT, l_constant, ATl, 2, 1, 2);

			LA[imgID * 2 + 0] += ATl[0];
			LA[imgID * 2 + 1] += ATl[1];
		}
	}

	//2. compute NB and LB
	double *NB = new double[3 * 3 * ptNum];
	memset(NB, 0, sizeof(double) * 3 * 3 * ptNum);

	double *BTB = new double[3 * 3];
	memset(BTB, 0, sizeof(double) * 3 * 3);
	double *B = new double[2 * 3];
	double *BT = new double[3 * 2];
	for (int i = 0; i < ptNum; i++)
	{
		int trackNum = 0;
		trackNum = pts[i].ptNum;
		double Lat, Lon, Hei;
		Lat = pts[i].Lat;
		Lon = pts[i].Lon;
		Hei = pts[i].Hei;
		for (int j = 0; j < trackNum; j++)
		{
			char *imgID_str = pts[i].img_pts[j].imgID;
			int imgID = GetImgID(imgIDs, imgNum, imgID_str);

			double row, col;
			row = pts[i].img_pts[j].y;
			col = pts[i].img_pts[j].x;

			double deltax, deltay;
			deltax = pts[i].img_pts[j].deltax;
			deltay = pts[i].img_pts[j].deltay;
			double dis_xy = sqrt(deltax*deltax + deltay*deltay);
			double wei = exp(-dis_xy / (sigma_coef * RMSE));
			wei = sqrt(wei);

			//////////////////////////////////////////////////////////////////////////
// 			if (trackNum == 2)
// 			{
// 				wei = 0.3;
// 			}
// 			else if (trackNum == 3)
// 			{
// 				wei = 0.6;
// 			}
// 			else
				wei = 1;
			//////////////////////////////////////////////////////////////////////////

			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lat", B[0]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lon", B[1]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Hei", B[2]);

			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lat", B[3]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lon", B[4]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Hei", B[5]);
			if (RMSE<1)
			{
				for (int mm = 0; mm < 6; mm++)
				{
					B[mm] = B[mm] * wei;
				}
			}
			

			matc.TransposeMatrix(B, 2, 3, BT);
			matc.MultiMatrix(BT, B, BTB, 3, 3, 2);

			int k;
			for (k = 0; k < 9; k++)
			{
				NB[i * 9 + k] += BTB[k];
			}
		}
	}

	//compute inverse matrix of NB
	double *NB_inverse = new double[3 * 3 * ptNum];
	memset(NB_inverse, 0, sizeof(double) * 3 * 3 * ptNum);
	for (int i = 0; i < ptNum;i++)
	{
		double BTB[9];
		double BTB_inverse[9];

		for (int j = 0; j < 9;j++)
		{
			BTB[j] = NB[i * 9 + j];
		}

		matc.InverseMatrix(BTB, 3, BTB_inverse);

		for (int j = 0; j < 9;j++)
		{
			NB_inverse[i * 9 + j] = BTB_inverse[j];
		}
	}

	//compute LB
	double *LB = new double[3 * ptNum];
	memset(LB, 0, sizeof(double) * 3 * ptNum);
	for (int i = 0; i < ptNum; i++)
	{
		int trackNum = 0;
		trackNum = pts[i].ptNum;
		double Lat, Lon, Hei;
		Lat = pts[i].Lat;
		Lon = pts[i].Lon;
		Hei = pts[i].Hei;
		for (int j = 0; j < trackNum; j++)
		{
			char *imgID_str = pts[i].img_pts[j].imgID;
			int imgID = GetImgID(imgIDs, imgNum, imgID_str);

			double row, col;
			row = pts[i].img_pts[j].y;
			col = pts[i].img_pts[j].x;

			double deltax, deltay;
			deltax = pts[i].img_pts[j].deltax;
			deltay = pts[i].img_pts[j].deltay;
			double dis_xy = sqrt(deltax*deltax + deltay*deltay);
			double wei = exp(-dis_xy / (sigma_coef * RMSE));
			wei = sqrt(wei);

			//////////////////////////////////////////////////////////////////////////
// 			if (trackNum == 2)
// 			{
// 				wei = 0.3;
// 			}
// 			else if (trackNum == 3)
// 			{
// 				wei = 0.6;
// 			}
// 			else
				wei = 1;
			//////////////////////////////////////////////////////////////////////////

			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lat", B[0]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lon", B[1]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Hei", B[2]);

			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lat", B[3]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lon", B[4]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Hei", B[5]);
			if (RMSE < 1)
			{
				for (int mm = 0; mm < 6; mm++)
				{
					B[mm] = B[mm] * wei;
				}
			}

			matc.TransposeMatrix(B, 2, 3, BT);
			
			double l_constant[2];
			//row
			Compute_Taylor_Constant_Term(0, 0, bias_para0[imgID].drow, rpc_paras[imgID], "row", row, col, Lat, Lon, Hei, l_constant[0]);
			l_constant[0] = -l_constant[0];
			//col
			Compute_Taylor_Constant_Term(0, 0, bias_para0[imgID].dcol, rpc_paras[imgID], "col", row, col, Lat, Lon, Hei, l_constant[1]);
			l_constant[1] = -l_constant[1];
			if (RMSE < 1)
			{
				for (int mm = 0; mm < 2; mm++)
				{
					l_constant[mm] = l_constant[mm] * wei;
				}
			}

			double BTl[3];
			matc.MultiMatrix(BT, l_constant, BTl, 3, 1, 2);

			LB[i * 3 + 0] += BTl[0];
			LB[i * 3 + 1] += BTl[1];
			LB[i * 3 + 2] += BTl[2];
		}
	}

	
	//3. compute NAB*NB_inverse*NAB_t, this is the most difficult
	double *NA_B = new double[(2 * imgNum)*(2 * imgNum)];
	memset(NA_B, 0, sizeof(double) * (2 * imgNum)*(2 * imgNum));

	for (int i = 0; i < ptNum; i++)
	{
		int trackNum = 0;
		trackNum = pts[i].ptNum;
		double Lat, Lon, Hei;
		Lat = pts[i].Lat;
		Lon = pts[i].Lon;
		Hei = pts[i].Hei;
		int *imgID_array = new int[trackNum];
		for (int j = 0; j < trackNum;j++)
		{
			char *imgID_str = pts[i].img_pts[j].imgID;
			imgID_array[j] = GetImgID(imgIDs, imgNum, imgID_str);
		}

		//digonal elements
// 		for (int j = 0; j < trackNum;j++)
// 		{
// 			int imgID = imgID_array[j];
// 			double row, col;
// 			row = pts[i].img_pts[j].y;
// 			col = pts[i].img_pts[j].x;
// 
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "c", A[0]);
// 			A[1] = 0;
// 
// 			A[2] = 0;
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "f", A[3]);
// 
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lat", B[0]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lon", B[1]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Hei", B[2]);
// 
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lat", B[3]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lon", B[4]);
// 			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Hei", B[5]);
// 
// 			double *ATB = new double[2 * 3];
// 			matc.TransposeMatrix(A, 2, 2, AT);
// 			matc.MultiMatrix(AT, B, ATB, 2, 3, 2);
// 
// 			double *BTA = new double[3 * 2];
// 			matc.TransposeMatrix(ATB, 2, 3, BTA);
// 
// 			double BTB_inverse[9];
// 			for (int t = 0; t < 9;t++)
// 			{
// 				BTB_inverse[t] = NB_inverse[i * 9 + t];
// 			}
// 
// 			double *ATB_BTB_inverse = new double[2 * 3];
// 			matc.MultiMatrix(ATB, BTB_inverse, ATB_BTB_inverse, 2, 3, 3);
// 			double *ATBA = new double[2 * 2];
// 			matc.MultiMatrix(ATB_BTB_inverse, BTA, ATBA, 2, 2, 3);
// 
// 			NA_B[(imgID * 2 + 0)*(imgNum * 2) + (imgID * 2 + 0)] += ATBA[0];
// 			NA_B[(imgID * 2 + 0)*(imgNum * 2) + (imgID * 2 + 1)] += ATBA[1];
// 			NA_B[(imgID * 2 + 1)*(imgNum * 2) + (imgID * 2 + 0)] += ATBA[2];
// 			NA_B[(imgID * 2 + 1)*(imgNum * 2) + (imgID * 2 + 1)] += ATBA[3];
// 
// 			delete[]ATB; ATB = NULL;
// 			delete[]BTA; BTA = NULL;
// 			delete[]ATB_BTB_inverse; ATB_BTB_inverse = NULL;
// 			delete[]ATBA; ATBA = NULL;
// 		}

		//other element
		for (int j = 0; j < trackNum; j++)
			for (int k = 0; k < trackNum;k++)
			{
				int imgID_j = imgID_array[j];
				int imgID_k = imgID_array[k];
				double row_j, col_j;
				double row_k, col_k;
				row_j = pts[i].img_pts[j].y;
				col_j = pts[i].img_pts[j].x;
				row_k = pts[i].img_pts[k].y;
				col_k = pts[i].img_pts[k].x;

				double deltax_j, deltay_j;
				double deltax_k, deltay_k;
				deltax_j = pts[i].img_pts[j].deltax;
				deltay_j = pts[i].img_pts[j].deltay;
				deltax_k = pts[i].img_pts[k].deltax;
				deltay_k = pts[i].img_pts[k].deltay;
				double dis_xy_j = sqrt(deltax_j*deltax_j + deltay_j*deltay_j);
				double dis_xy_k = sqrt(deltax_k*deltax_k + deltay_k*deltay_k);
				double wei_j = exp(-dis_xy_j / (sigma_coef * RMSE));
				double wei_k = exp(-dis_xy_k / (sigma_coef * RMSE));
				wei_j = sqrt(wei_j);
				wei_k = sqrt(wei_k);

				//////////////////////////////////////////////////////////////////////////
// 				if (trackNum == 2)
// 				{
// 					wei_j = 0.3;
// 					wei_k = 0.3;
// 				}
// 				else if (trackNum == 3)
// 				{
// 					wei_j = 0.6;
// 					wei_k = 0.6;
// 				}
// 				else
				{
					wei_j = 1;
					wei_k = 1;
				}
				//////////////////////////////////////////////////////////////////////////

				double Aj[2 * 2], Ak[2 * 2];
				memset(Aj, 0, sizeof(double) * 4);
				memset(Ak, 0, sizeof(double) * 4);
				Derivate_RPC_var(rpc_paras[imgID_j], bias_para0[imgID_j], row_j, col_j, Lat, Lon, Hei, "row", "c", Aj[0]);
				Aj[1] = 0;
				Aj[2] = 0;
				Derivate_RPC_var(rpc_paras[imgID_j], bias_para0[imgID_j], row_j, col_j, Lat, Lon, Hei, "col", "f", Aj[3]);
				if (RMSE<1)
				{
					for (int mm = 0; mm < 4; mm++)
					{
						Aj[mm] = Aj[mm] * wei_j;
					}
				}

				Derivate_RPC_var(rpc_paras[imgID_k], bias_para0[imgID_k], row_k, col_k, Lat, Lon, Hei, "row", "c", Ak[0]);
				Ak[1] = 0;
				Ak[2] = 0;
				Derivate_RPC_var(rpc_paras[imgID_k], bias_para0[imgID_k], row_k, col_k, Lat, Lon, Hei, "col", "f", Ak[3]);
				if (RMSE<1)
				{
					for (int mm = 0; mm < 4; mm++)
					{
						Ak[mm] = Ak[mm] * wei_k;
					}
				}

				double Bj[6], Bk[6];
				Derivate_RPC_var(rpc_paras[imgID_j], bias_para0[imgID_j], row_j, col_j, Lat, Lon, Hei, "row", "Lat", Bj[0]);
				Derivate_RPC_var(rpc_paras[imgID_j], bias_para0[imgID_j], row_j, col_j, Lat, Lon, Hei, "row", "Lon", Bj[1]);
				Derivate_RPC_var(rpc_paras[imgID_j], bias_para0[imgID_j], row_j, col_j, Lat, Lon, Hei, "row", "Hei", Bj[2]);
				Derivate_RPC_var(rpc_paras[imgID_j], bias_para0[imgID_j], row_j, col_j, Lat, Lon, Hei, "col", "Lat", Bj[3]);
				Derivate_RPC_var(rpc_paras[imgID_j], bias_para0[imgID_j], row_j, col_j, Lat, Lon, Hei, "col", "Lon", Bj[4]);
				Derivate_RPC_var(rpc_paras[imgID_j], bias_para0[imgID_j], row_j, col_j, Lat, Lon, Hei, "col", "Hei", Bj[5]);
				if (RMSE<1)
				{
					for (int mm = 0; mm < 6; mm++)
					{
						Bj[mm] = Bj[mm] * wei_j;
					}
				}

				Derivate_RPC_var(rpc_paras[imgID_k], bias_para0[imgID_k], row_k, col_k, Lat, Lon, Hei, "row", "Lat", Bk[0]);
				Derivate_RPC_var(rpc_paras[imgID_k], bias_para0[imgID_k], row_k, col_k, Lat, Lon, Hei, "row", "Lon", Bk[1]);
				Derivate_RPC_var(rpc_paras[imgID_k], bias_para0[imgID_k], row_k, col_k, Lat, Lon, Hei, "row", "Hei", Bk[2]);
				Derivate_RPC_var(rpc_paras[imgID_k], bias_para0[imgID_k], row_k, col_k, Lat, Lon, Hei, "col", "Lat", Bk[3]);
				Derivate_RPC_var(rpc_paras[imgID_k], bias_para0[imgID_k], row_k, col_k, Lat, Lon, Hei, "col", "Lon", Bk[4]);
				Derivate_RPC_var(rpc_paras[imgID_k], bias_para0[imgID_k], row_k, col_k, Lat, Lon, Hei, "col", "Hei", Bk[5]);
				if (RMSE<1)
				{
					for (int mm = 0; mm < 6; mm++)
					{
						Bk[mm] = Bk[mm] * wei_k;
					}
				}

				double *AjTBj = new double[2 * 3];
				double AjT[4];
				matc.TransposeMatrix(Aj, 2, 2, AjT);
				matc.MultiMatrix(AjT, Bj, AjTBj, 2, 3, 2);

				double *BkTAk = new double[3 * 2];
				double BkT[3 * 2];
				matc.TransposeMatrix(Bk, 2, 3, BkT);
				matc.MultiMatrix(BkT, Ak, BkTAk, 3, 2, 2);

				double BTB_inverse[9];
				for (int t = 0; t < 9; t++)
				{
					BTB_inverse[t] = NB_inverse[i * 9 + t];
				}

				double *ATB_BTB_inverse = new double[2 * 3];
				matc.MultiMatrix(AjTBj, BTB_inverse, ATB_BTB_inverse, 2, 3, 3);
				double *ATBA = new double[2 * 2];
				matc.MultiMatrix(ATB_BTB_inverse, BkTAk, ATBA, 2, 2, 3);

				NA_B[(imgID_j * 2 + 0)*(imgNum * 2) + (imgID_k * 2 + 0)] += ATBA[0];
				NA_B[(imgID_j * 2 + 0)*(imgNum * 2) + (imgID_k * 2 + 1)] += ATBA[1];
				NA_B[(imgID_j * 2 + 1)*(imgNum * 2) + (imgID_k * 2 + 0)] += ATBA[2];
				NA_B[(imgID_j * 2 + 1)*(imgNum * 2) + (imgID_k * 2 + 1)] += ATBA[3];

				delete[]AjTBj; AjTBj = NULL;
				delete[]BkTAk; BkTAk = NULL;
				delete[]ATB_BTB_inverse; ATB_BTB_inverse = NULL;
				delete[]ATBA; ATBA = NULL;
			}

		delete[]imgID_array; imgID_array = NULL;
	}

	N_bias = new double[(2 * imgNum)*(2 * imgNum)];
	memset(N_bias, 0, sizeof(double)*(2 * imgNum)*(2 * imgNum));
	for (int i = 0; i < (2 * imgNum)*(2 * imgNum);i++)
	{
		N_bias[i] = NA[i] - NA_B[i];
	}

	//4. compute L_AB = NAB*NB_inverse*LB
	double *L_AB = new double[2 * imgNum];
	memset(L_AB, 0, sizeof(double) * 2 * imgNum);

	for (int i = 0; i < ptNum; i++)
	{
		int trackNum = 0;
		trackNum = pts[i].ptNum;
		double Lat, Lon, Hei;
		Lat = pts[i].Lat;
		Lon = pts[i].Lon;
		Hei = pts[i].Hei;
		int *imgID_array = new int[trackNum];
		for (int j = 0; j < trackNum; j++)
		{
			char *imgID_str = pts[i].img_pts[j].imgID;
			imgID_array[j] = GetImgID(imgIDs, imgNum, imgID_str);
		}

		for (int j = 0; j < trackNum; j++)
		{
			int imgID = imgID_array[j];
			double row, col;
			row = pts[i].img_pts[j].y;
			col = pts[i].img_pts[j].x;

			double deltax, deltay;
			deltax = pts[i].img_pts[j].deltax;
			deltay = pts[i].img_pts[j].deltay;
			double dis_xy = sqrt(deltax*deltax + deltay*deltay);
			double wei = exp(-dis_xy / (sigma_coef * RMSE));
			wei = sqrt(wei);

			//////////////////////////////////////////////////////////////////////////
// 			if (trackNum == 2)
// 			{
// 				wei = 0.3;
// 			}
// 			else if (trackNum == 3)
// 			{
// 				wei = 0.6;
// 			}
// 			else
			{
				wei = 1;
			}
			//////////////////////////////////////////////////////////////////////////

			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "c", A[0]);
			A[1] = 0;

			A[2] = 0;
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "f", A[3]);
			if (RMSE<1)
			{
				for (int mm = 0; mm < 4; mm++)
				{
					A[mm] = A[mm] * wei;
				}
			}

			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lat", B[0]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Lon", B[1]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "row", "Hei", B[2]);

			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lat", B[3]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Lon", B[4]);
			Derivate_RPC_var(rpc_paras[imgID], bias_para0[imgID], row, col, Lat, Lon, Hei, "col", "Hei", B[5]);
			if (RMSE<1)
			{
				for (int mm = 0; mm < 6; mm++)
				{
					B[mm] = B[mm] * wei;
				}
			}

			double *ATB = new double[2 * 3];
			matc.TransposeMatrix(A, 2, 2, AT);
			matc.MultiMatrix(AT, B, ATB, 2, 3, 2);

			double BTB_inverse[9];
			for (int k = 0; k < 9; k++)
			{
				BTB_inverse[k] = NB_inverse[i * 9 + k];
			}

			double *ATB_BTB_inverse = new double[2 * 3];
			matc.MultiMatrix(ATB, BTB_inverse, ATB_BTB_inverse, 2, 3, 3);
			double lb[3];
			lb[0] = LB[i * 3 + 0]; lb[1] = LB[i * 3 + 1]; lb[2] = LB[i * 3 + 2];

			double lab[2];
			matc.MultiMatrix(ATB_BTB_inverse, lb, lab, 2, 1, 3);

			L_AB[imgID * 2 + 0] += lab[0];
			L_AB[imgID * 2 + 1] += lab[1];

			delete[]ATB; ATB = NULL;
			delete[]ATB_BTB_inverse; ATB_BTB_inverse = NULL;
		}

		delete[]imgID_array; imgID_array = NULL;
	}

	L_bias = new double[2 * imgNum];
	memset(L_bias, 0, sizeof(double) * 2 * imgNum);
	for (int i = 0; i < 2 * imgNum;i++)
	{
		L_bias[i] = LA[i] - L_AB[i];
	}

	//free memory
	delete[]NA; NA = NULL;
	delete[]A; A = NULL;
	delete[]AT; AT = NULL;
	delete[]ATA; ATA = NULL;
	delete[]LA; LA = NULL;

 	delete[]NB; NB = NULL;
	delete[]NB_inverse; NB_inverse = NULL;
	delete[]BTB; BTB = NULL;
	delete[]B; B = NULL;
	delete[]BT; BT = NULL;
	delete[]LB; LB = NULL;

// 	delete[]NA_B; NA_B = NULL;
// 	delete[]L_AB; L_AB = NULL;

// 	delete[]NATB; NATB = NULL;
// 	delete[]ATB; ATB = NULL;
// 	delete[]NATB_T; NATB_T = NULL;
// 	
// 	delete[]NAB_NB_inverse; NAB_NB_inverse = NULL;
// 	delete[]NAB_NB_inverse_NAB; NAB_NB_inverse_NAB = NULL;
//  	delete[]CoeffMatrix; CoeffMatrix = NULL;
	

	return 1;
}

int CRPC_Fun::GetImgID(char **IDs, int num, char *cur_ID)
{
	int detected_ID = -1;
	for (int i = 0; i < num;i++)
	{
		if (strcmp(cur_ID, IDs[i])==0)
		{
			detected_ID = i;
			break;
		}
	}

	return detected_ID;
}

int CRPC_Fun::RPC_BundleAdjustment(Ground_Img_Pt *pts, int ptNum, Long_Strip_Img *strips, int StripNum, char *Bundle_list_path)
{
	RPC *rpcs = NULL;
	char **imgIDs = NULL;
	int imgNum = 0;

	GetImgNumFromPtList(pts, ptNum, strips, StripNum, imgNum, rpcs, imgIDs);

	//intialize
	BiasPara_constant *bias_para0 = new BiasPara_constant[imgNum];
	for (int i = 0; i < imgNum;i++)
	{
		bias_para0[i].drow = 0;
		bias_para0[i].dcol = 0;
	}

	//define the reprojection error as the metric to measure the iteration stopping conditions
	double reprojection_error = 100;
	reprojection_error = 0;
	int total_img_pt_num = 0;
	for (int i = 0; i < ptNum; i++)
	{
		int trackNum = pts[i].ptNum;
		for (int j = 0; j < trackNum; j++)
		{
			double deltax, deltay;
			deltax = pts[i].img_pts[j].deltax;
			deltay = pts[i].img_pts[j].deltay;

			/*reprojection_error += sqrt(deltax*deltax + deltay*deltay);*/
			reprojection_error += deltax*deltax + deltay*deltay;

			total_img_pt_num++;
		}
	}
	reprojection_error = sqrt(reprojection_error/total_img_pt_num);
//	printf("Interation: %d   Image plane reprojection error: %lf\n", 0, reprojection_error);

	int I = 1;
	double reprojection_error_before = 100;
	printf("\nStart accurate bunlde adjustment\n");
	while (fabs(reprojection_error_before - reprojection_error)>0.0001 /*|| reprojection_error_before>reprojection_error*/)
	{
		//1. compute the improved normal function
		double *N_bias = NULL, *L_bias = NULL, *X = NULL;
		ComputeNormalEquations_constant(pts, ptNum, rpcs, bias_para0, imgNum, imgIDs, reprojection_error, N_bias, L_bias);

		int basis_ID = GetImgID(imgIDs, imgNum, m_basis_ID);
		double *acc = new double[imgNum];
		memset(acc, 0, sizeof(double)*imgNum);
		int acc_num = 0;
		for (int i = 0; i < imgNum;i++)
		{
			if (i==basis_ID) {
				continue;
			}
			char *cur_ID = imgIDs[i];

			acc_num = 0;
			for (int j = 0; j < ptNum;j++)
			{
				int trackNum = pts[j].ptNum;
				if (trackNum>=2)
				{
					for (int t = 0; t < trackNum; t++)
					{
						if (strcmp(cur_ID, pts[j].img_pts[t].imgID) == 0)
						{
							acc[i] += sqrt(pts[j].img_pts[t].deltax*pts[j].img_pts[t].deltax + pts[j].img_pts[t].deltay*pts[j].img_pts[t].deltay);
							acc_num++;
						}
					}
				}
			}

			acc[i] /= acc_num;
		}

		printf("\n%lf\t%lf\t%lf\t%lf\t%lf\n", acc[0], acc[1], acc[2], acc[3], acc[4]);

		for (int i = 0; i < (2 * imgNum);i++)
		{
			N_bias[i*(2 * imgNum) + i] += 2*exp(-acc[i/2]);
		}
		delete[]acc; acc = NULL;

		//////////////////////////////////////////////////////////////////////////
		double N_bias_inverse[10 * 10];
		CSimple_Matrix_Computation matc;
		matc.InverseMatrix(N_bias, 10, N_bias_inverse);
		FILE *fp1 = fopen("D:\\NA.txt", "w");

		for (int ty = 0; ty < 10; ty++)
		{
			for (int tx = 0; tx < 10; tx++)
			{
				fprintf(fp1, "%lf\t", N_bias_inverse[ty * 10 + tx]);
			}
			fprintf(fp1, "\n");
		}

		fclose(fp1); fp1 = NULL;

		//return 1;
		//////////////////////////////////////////////////////////////////////////

		//2. using Eigen to solve the solution
		int Cols = imgNum * 2;
		X = new double[Cols];
		MatrixXd Ge(Cols, Cols);
		VectorXd He(Cols);
		for (int i = 0; i < Cols; i++)
			for (int j = 0; j < Cols; j++)
			{
				Ge(i, j) = N_bias[i*Cols + j];
			}
		for (int i = 0; i < Cols; i++)
		{
			He(i) = L_bias[i];
		}
		VectorXd Xe = Ge.fullPivLu().solve(He);
		for (int i = 0; i < Cols; i++)
		{
			X[i] = Xe(i);
		}

		//3. update the initial values
		int iter = 0;
		for (int i = 0; i < imgNum; i++)
		{
			bias_para0[i].drow += X[iter];
			iter++;
			bias_para0[i].dcol += X[iter];
			iter++;
		}

		//4. refine all rpcs
		for (int i = 0; i < imgNum;i++)
		{
			RefineRPC_bias_constant(rpcs[i], bias_para0[i].drow, bias_para0[i].dcol);
		}

		//5. re-initialize bias parameters
		for (int i = 0; i < imgNum;i++)
		{
			bias_para0[i].drow = 0;
			bias_para0[i].dcol = 0;
		}

		//6. update all Lat, Lon, Hei and deltax, deltay
		for (int i = 0; i < ptNum;i++)
		{
			int trackNum = pts[i].ptNum;
			double *x_track = new double[trackNum];
			double *y_track = new double[trackNum];
			RPC *rpc_track = new RPC[trackNum];

			for (int j = 0; j < trackNum;j++)
			{
				char *ID_track = pts[i].img_pts[j].imgID;
				x_track[j] = pts[i].img_pts[j].x;
				y_track[j] = pts[i].img_pts[j].y;

				int img_index = 0;
				img_index = GetImgID(imgIDs, imgNum, ID_track);
				rpc_track[j] = rpcs[img_index];
			}

			double tLat, tLon, tHei;
			Multi_view_Forward_intersection(x_track, y_track, rpc_track, trackNum, tLat, tLon, tHei);

			pts[i].Lat = tLat;
			pts[i].Lon = tLon;
			pts[i].Hei = tHei;

			//compute the new re-projection error
			for (int j = 0; j < trackNum; j++)
			{
				double reproj_x, reproj_y;
				RPC_Ground2Image(tLat, tLon, tHei, rpc_track[j], reproj_y, reproj_x);

				pts[i].img_pts[j].deltax = reproj_x - x_track[j];
				pts[i].img_pts[j].deltay = reproj_y - y_track[j];
			}
			
			//free memory
			delete[]x_track; x_track = NULL;
			delete[]y_track; y_track = NULL;
			delete[]rpc_track; rpc_track = NULL;
		}

		//7. compute the new re-projection errors
		reprojection_error_before = reprojection_error;
		reprojection_error = 0;
		int total_img_pt_num = 0;
		for (int i = 0; i < ptNum; i++)
		{
			int trackNum = pts[i].ptNum;
			for (int j = 0; j < trackNum;j++)
			{
				double deltax, deltay;
				deltax = pts[i].img_pts[j].deltax;
				deltay = pts[i].img_pts[j].deltay;

				//reprojection_error += sqrt(deltax*deltax + deltay*deltay);
				reprojection_error += sqrt(deltax*deltax + deltay*deltay);
				total_img_pt_num++;
			}
		}
		//reprojection_error /= total_img_pt_num;
		reprojection_error = reprojection_error / total_img_pt_num;

		printf("Interation: %d   Image plane reprojection error: %lf\n", I, reprojection_error);
		I++;

		//free memory
		delete[]N_bias; N_bias = NULL;
		delete[]L_bias; L_bias = NULL;
		delete[]X; X = NULL;
	}

	printf("Iteration finished\n");

	//Save bundle adjustment results
	for (int i = 0; i < StripNum;i++)
	{
		int patchNum = strips[i].patchNum;
		for (int j = 0; j < patchNum;j++)
		{
			char img_ID_str[100];
			sprintf_s(img_ID_str, "%d_%d", i, j);

			int img_ID = 0;
			img_ID = GetImgID(imgIDs, imgNum, img_ID_str);

			WriteRPBFile(rpcs[img_ID], strips[i].refined_rpb_paths[j]);
		}
	}

	//output new tracking points
	FILE *fp = fopen(Bundle_list_path, "w");

	for (int i = 0; i < ptNum; i++)
	{
		//check the reprojection error of matching points
		int validNum = pts[i].ptNum;
		for (int j = 0; j < pts[i].ptNum; j++)
		{
			double dis = sqrt(pts[i].img_pts[j].deltax*pts[i].img_pts[j].deltax + pts[i].img_pts[j].deltay*pts[i].img_pts[j].deltay);
			if (dis>1.2)
			{
				validNum--;
			}
		}

		if (validNum>=2)
		{
			fprintf(fp, "%lf %lf %lf %d\n", pts[i].Lat, pts[i].Lon, pts[i].Hei, validNum);
			for (int j = 0; j < pts[i].ptNum; j++)
			{
				double dis = sqrt(pts[i].img_pts[j].deltax*pts[i].img_pts[j].deltax + pts[i].img_pts[j].deltay*pts[i].img_pts[j].deltay);
				if (dis <= 1.2)
				{
					fprintf(fp, "%s %lf %lf %lf %lf\n", pts[i].img_pts[j].imgID, pts[i].img_pts[j].x, pts[i].img_pts[j].y, pts[i].img_pts[j].deltax, pts[i].img_pts[j].deltay);
				}
			}
		}
	}
	fprintf(fp, "END\n");
	fclose(fp); fp = NULL;

	//free memory
	delete[]rpcs; rpcs = NULL;
	for (int i = 0; i < imgNum;i++)
	{
		delete[]imgIDs[i]; imgIDs[i] = NULL;
	}
	delete[]imgIDs; imgIDs = NULL;
	delete[]bias_para0; bias_para0 = NULL;
	return 1;
}

int CRPC_Fun::GetImgNumFromPtList(Ground_Img_Pt *pts, int ptNum, Long_Strip_Img *strips, int StripNum, int &imgNum, RPC *& rpcs, char **&imgIDs)
{
	imgNum = 0;
	vector<string> vIDs_img;

	for (int i = 0; i < StripNum;i++)
	{
		int patchNum = strips[i].patchNum;
		for (int j = 0; j < patchNum;j++)
		{
			char cur_ID[20];
			sprintf_s(cur_ID, "%d_%d", i, j);

			bool ifsameID = false;
			for (int k = 0; k < ptNum;k++)
			{
				int trackNum = pts[k].ptNum;
				for (int n = 0; n < trackNum;n++)
				{
					char pt_ID[20];
					strcpy(pt_ID, pts[k].img_pts[n].imgID);

					if (strcmp(pt_ID,cur_ID)==0)
					{
						ifsameID = true;
						break;
					}
				}

				if (ifsameID==true)
				{
					break;
				}
			}

			vIDs_img.push_back(cur_ID);
		}
	}

	imgNum = vIDs_img.size();
	rpcs = new RPC[imgNum];
	imgIDs = new char *[imgNum];
	for (int i = 0; i < imgNum;i++)
	{
		imgIDs[i] = new char[100];
	}

	for (int i = 0; i < imgNum;i++)
	{
		char cur_ID[100];
		strcpy(cur_ID, vIDs_img[i].c_str());

		int stripID, patchID;
		sscanf(cur_ID, "%d_%d", &stripID, &patchID);

		char *rpbPath = strips[stripID].refined_rpb_paths[patchID];

		ReadRpc_singleFile(rpbPath, rpcs[i]);
		strcpy(imgIDs[i], cur_ID);
	}

	//free memory
	vIDs_img.clear();
	vector<string>().swap(vIDs_img);

// 	vector <int> imgID;
// 	int i, j;
// 	for (i = 0; i < ptNum; i++)
// 	{
// 		int trackNum = pts[i].imgNum;
// 		for (j = 0; j < trackNum;j++)
// 		{
// 			int ID = pts[i].ID[j];
// 			if (imgID.size()==0)
// 			{
// 				imgID.push_back(ID);
// 			}
// 			else
// 			{
// 				bool ifrepeated = false;
// 				int t;
// 				for (t = 0; t < imgID.size();t++)
// 				{
// 					int vID = imgID[t];
// 					if (vID==ID)
// 					{
// 						ifrepeated = true;
// 						break;
// 					}
// 				}
// 
// 				if (ifrepeated==false)
// 				{
// 					imgID.push_back(ID);
// 				}
// 			}
// 		}
// 	}
// 
// 
// 	imgNum = imgID.size();
// 
// 	imgID.clear();
// 	vector<int>().swap(imgID);

	return 1;
}

int CRPC_Fun::RelativeOrientation_bias_constant(double *lx, double *ly, double *rx, double *ry, int ptNum, RPC &rpc_left_ori, RPC &rpc_right_ori,
	double &drow, double &dcol, double &error_reprojection)
{
// 	drow = 0;
// 	dcol = 0;
// 
// 	RPC rpc_left, rpc_right;
// 	rpc_left = rpc_left_ori;
// 	rpc_right = rpc_right_ori;
// 
// 	double *lat, *lon, *hei;
// 	lat = new double[ptNum];
// 	lon = new double[ptNum];
// 	hei = new double[ptNum];
// 
// 	double *dis = new double[ptNum];
// 	memset(dis, 0, sizeof(double)*ptNum);
// 
// 	error_reprojection = 0;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		RPC_ForwardIntersection(rpc_left, rpc_right, lx[i], ly[i], rx[i], ry[i], lat[i], lon[i], hei[i]);
// 
// 		double lx_reproj, ly_reproj;
// 		double rx_reproj, ry_reproj;
// 		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_left, ly_reproj, lx_reproj);
// 		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, ry_reproj, rx_reproj);
// 
// 		double disL, disR;
// 		disL = sqrt((lx_reproj - lx[i])*(lx_reproj - lx[i]) + (ly_reproj - ly[i])*(ly_reproj - ly[i]));
// 		disR = sqrt((rx_reproj - rx[i])*(rx_reproj - rx[i]) + (ry_reproj - ry[i])*(ry_reproj - ry[i]));
// 
// 		dis[i] = (disL + disR) / 2;
// 
// 		error_reprojection += dis[i];
// 	}
// 	error_reprojection /= ptNum;
// 
// 	CImageFeatureMatching feaM;
// 	if (error_reprojection < 1)
// 	{
// 		//free memroy
// 		delete[]lat; lat = NULL;
// 		delete[]lon; lon = NULL;
// 		delete[]hei; hei = NULL;
// 		delete[]dis; dis = NULL;
// 
// 		return 1;
// 	}
// 
// 	double *drow_best, *dcol_best;
// 	drow_best = new double[ptNum];
// 	dcol_best = new double[ptNum];
// 	double *weight = new double[ptNum];
// 	double *dis_best = new double[ptNum];
// 	//intialize
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		drow_best[i] = 999999;
// 		dcol_best[i] = 999999;
// 		dis_best[i] = 999999;
// 
// 		weight[i] = 1;
// 	}
// 
// 	double minZ = rpc_right.heightOffset - rpc_right.heightScale;
// 	double maxZ = rpc_right.heightOffset + rpc_right.heightScale;
// 	double stepZ = 0.1;
// 	double tZ;
// 	double scale = 2;
// 	double drow_final, dcol_final;
// 	while (error_reprojection > 0.7)
// 	{
// 		for (tZ = minZ; tZ <= maxZ; tZ += stepZ)
// 		{
// 			for (int i = 0; i < ptNum; i++)
// 			{
// 				RPC_Image2Ground(ly[i], lx[i], tZ, rpc_left, lat[i], lon[i]);
// 				hei[i] = tZ;
// 
// 				double iy_right, ix_right;
// 				RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, iy_right, ix_right);
// 
// 				double tdis = sqrt((iy_right - ry[i])*(iy_right - ry[i]) + (ix_right - rx[i])*(ix_right - rx[i]));
// 				if (tdis < dis_best[i])
// 				{
// 					drow_best[i] = iy_right - ry[i];
// 					dcol_best[i] = ix_right - rx[i];
// 					dis_best[i] = tdis;
// 				}
// 			}
// 		}
// 
// 		//weighted median filtering
// 		int weighted_num = 0;
// 		for (int i = 0; i < ptNum; i++)
// 		{
// 			int w_num = (int)(weight[i] * scale + 0.5);
// 			weighted_num += w_num;
// 		}
// 
// 		double *drow_weighted = new double[weighted_num];
// 		double *dcol_weighted = new double[weighted_num];
// 		memset(drow_weighted, 0, sizeof(double)*weighted_num);
// 		memset(dcol_weighted, 0, sizeof(double)*weighted_num);
// 		int iter = 0;
// 		for (int i = 0; i < ptNum; i++)
// 		{
// 			int w_num = (int)(weight[i] * scale + 0.5);
// 			for (int j = 0; j < w_num; j++)
// 			{
// 				drow_weighted[iter] = drow_best[i];
// 				dcol_weighted[iter] = dcol_best[i];
// 				iter++;
// 			}
// 		}
// 
// 		//To speed running time
// 		feaM.GetWeightedCenter(drow_best, ptNum, 3.0, drow_final);
// 		feaM.GetWeightedCenter(dcol_best, ptNum, 3.0, dcol_final);
// 
// 		delete[]drow_weighted; drow_weighted = NULL;
// 		delete[]dcol_weighted; dcol_weighted = NULL;
// 
// 		RefineRPC_bias_constant(rpc_right, drow_final, dcol_final);
// 
// 		//According to the new rpc_right, re-compute the re-projection errors
// 		double weight_total = 0;
// 		for (int i = 0; i < ptNum; i++)
// 		{
// 			RPC_ForwardIntersection(rpc_left, rpc_right, lx[i], ly[i], rx[i], ry[i], lat[i], lon[i], hei[i]);
// 
// 			double lx_reproj, ly_reproj;
// 			double rx_reproj, ry_reproj;
// 			RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_left, ly_reproj, lx_reproj);
// 			RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, ry_reproj, rx_reproj);
// 
// 			double disL, disR;
// 			disL = sqrt((lx_reproj - lx[i])*(lx_reproj - lx[i]) + (ly_reproj - ly[i])*(ly_reproj - ly[i]));
// 			disR = sqrt((rx_reproj - rx[i])*(rx_reproj - rx[i]) + (ry_reproj - ry[i])*(ry_reproj - ry[i]));
// 
// 			dis[i] = (disL + disR) / 2;
// 		}
// 
// 		//recompute the weight
// 		double error_reprojection_before = error_reprojection;
// 		error_reprojection = 0;
// 		for (int i = 0; i < ptNum; i++)
// 		{
// 			weight[i] = exp(-dis[i] / (3 * error_reprojection_before));
// 			error_reprojection += weight[i] * dis[i];
// 			weight_total += weight[i];
// 		}
// 		error_reprojection /= weight_total;
// 	}
// 
// 	drow = drow_final;
// 	dcol = dcol_final;
// 
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		RPC_ForwardIntersection(rpc_left, rpc_right, lx[i], ly[i], rx[i], ry[i], lat[i], lon[i], hei[i]);
// 
// 		double lx_reproj, ly_reproj;
// 		double rx_reproj, ry_reproj;
// 		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_left, ly_reproj, lx_reproj);
// 		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, ry_reproj, rx_reproj);
// 
// 		double disL, disR;
// 		disL = sqrt((lx_reproj - lx[i])*(lx_reproj - lx[i]) + (ly_reproj - ly[i])*(ly_reproj - ly[i]));
// 		disR = sqrt((rx_reproj - rx[i])*(rx_reproj - rx[i]) + (ry_reproj - ry[i])*(ry_reproj - ry[i]));
// 
// 		dis[i] = (disL + disR) / 2;
// 	}
// 
// 	error_reprojection = 0;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		error_reprojection += dis[i];
// 	}
// 	error_reprojection /= ptNum;
// 
// 	//free memory
// 	delete[]lat; lat = NULL;
// 	delete[]lon; lon = NULL;
// 	delete[]hei; hei = NULL;
// 	delete[]dis; dis = NULL;
// 	delete[]drow_best; drow_best = NULL;
// 	delete[]dcol_best; dcol_best = NULL;
// 	delete[]dis_best; dis_best = NULL;
// 	delete[]weight; weight = NULL;

	drow = 0;
	dcol = 0;

	RPC rpc_left, rpc_right;
	rpc_left = rpc_left_ori;
	rpc_right = rpc_right_ori;

	double *lat, *lon, *hei;
	lat = new double[ptNum];
	lon = new double[ptNum];
	hei = new double[ptNum];

	double *dis = new double[ptNum];
	memset(dis, 0, sizeof(double)*ptNum);

	error_reprojection = 0;
	for (int i = 0; i < ptNum; i++)
	{
		RPC_ForwardIntersection(rpc_left, rpc_right, lx[i], ly[i], rx[i], ry[i], lat[i], lon[i], hei[i]);

		double lx_reproj, ly_reproj;
		double rx_reproj, ry_reproj;
		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_left, ly_reproj, lx_reproj);
		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, ry_reproj, rx_reproj);

		double disL, disR;
		disL = sqrt((lx_reproj - lx[i])*(lx_reproj - lx[i]) + (ly_reproj - ly[i])*(ly_reproj - ly[i]));
		disR = sqrt((rx_reproj - rx[i])*(rx_reproj - rx[i]) + (ry_reproj - ry[i])*(ry_reproj - ry[i]));

		dis[i] = (disL + disR) / 2;

		error_reprojection += dis[i];
	}
	error_reprojection /= ptNum;

	CImageFeatureMatching feaM;
	if (error_reprojection < 1)
	{
// 		memset(inliers, 0, sizeof(bool)*ptNum);
// 		for (int i = 0; i < ptNum; i++)
// 		{
// 			if (dis[i] < feaM.m_thresh_inliers)
// 			{
// 				inliers[i] = 1;
// 			}
// 		}

		//free memroy
		delete[]lat; lat = NULL;
		delete[]lon; lon = NULL;
		delete[]hei; hei = NULL;
		delete[]dis; dis = NULL;

		return 1;
	}

	double *drow_best, *dcol_best;
	drow_best = new double[ptNum];
	dcol_best = new double[ptNum];
	double *weight = new double[ptNum];
	double *dis_best = new double[ptNum];
	//intialize
	for (int i = 0; i < ptNum; i++)
	{
		drow_best[i] = 999999;
		dcol_best[i] = 999999;
		dis_best[i] = 999999;

		weight[i] = 1;
	}

	double minZ = rpc_right.heightOffset - rpc_right.heightScale;
	double maxZ = rpc_right.heightOffset + rpc_right.heightScale;
	double stepZ = 0.1;
	double tZ;
	double scale = 2;
	double drow_final, dcol_final;
	double error_reprojection_before = error_reprojection + 10;
	double delta_error = fabs(error_reprojection_before - error_reprojection);
	//while (error_reprojection > 0.8)
	//while (delta_error>0.1)
	{
		for (tZ = minZ; tZ <= maxZ; tZ += stepZ)
		{
			for (int i = 0; i < ptNum; i++)
			{
				RPC_Image2Ground(ly[i], lx[i], tZ, rpc_left, lat[i], lon[i]);
				hei[i] = tZ;

				double iy_right, ix_right;
				RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, iy_right, ix_right);

				double tdis = sqrt((iy_right - ry[i])*(iy_right - ry[i]) + (ix_right - rx[i])*(ix_right - rx[i]));
				if (tdis < dis_best[i])
				{
					drow_best[i] = iy_right - ry[i];
					dcol_best[i] = ix_right - rx[i];
					dis_best[i] = tdis;
				}
			}
		}

		//weighted median filtering
		int weighted_num = 0;
		for (int i = 0; i < ptNum; i++)
		{
			int w_num = (int)(weight[i] * scale + 0.5);
			weighted_num += w_num;
		}

// 		double *drow_weighted = new double[weighted_num];
// 		double *dcol_weighted = new double[weighted_num];
// 		memset(drow_weighted, 0, sizeof(double)*weighted_num);
// 		memset(dcol_weighted, 0, sizeof(double)*weighted_num);
// 		int iter = 0;
// 		for (int i = 0; i < ptNum; i++)
// 		{
// 			int w_num = (int)(weight[i] * scale + 0.5);
// 			for (int j = 0; j < w_num; j++)
// 			{
// 				drow_weighted[iter] = drow_best[i];
// 				dcol_weighted[iter] = dcol_best[i];
// 				iter++;
// 			}
// 		}

// 		feaM.MedianFilter(drow_weighted, weighted_num, drow_final);
// 		feaM.MedianFilter(dcol_weighted, weighted_num, dcol_final);
		//To speed running time
		feaM.GetWeightedCenter(drow_best, ptNum, 3.0, drow_final);
		feaM.GetWeightedCenter(dcol_best, ptNum, 3.0, dcol_final);

// 		delete[]drow_weighted; drow_weighted = NULL;
// 		delete[]dcol_weighted; dcol_weighted = NULL;

		RefineRPC_bias_constant(rpc_right, drow_final, dcol_final);

		//According to the new rpc_right, re-compute the re-projection errors
		double weight_total = 0;
		for (int i = 0; i < ptNum; i++)
		{
			RPC_ForwardIntersection(rpc_left, rpc_right, lx[i], ly[i], rx[i], ry[i], lat[i], lon[i], hei[i]);

			double lx_reproj, ly_reproj;
			double rx_reproj, ry_reproj;
			RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_left, ly_reproj, lx_reproj);
			RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, ry_reproj, rx_reproj);

			double disL, disR;
			disL = sqrt((lx_reproj - lx[i])*(lx_reproj - lx[i]) + (ly_reproj - ly[i])*(ly_reproj - ly[i]));
			disR = sqrt((rx_reproj - rx[i])*(rx_reproj - rx[i]) + (ry_reproj - ry[i])*(ry_reproj - ry[i]));

			dis[i] = (disL + disR) / 2;
		}

		//recompute the weight
		error_reprojection_before = error_reprojection;
		error_reprojection = 0;
		for (int i = 0; i < ptNum; i++)
		{
			weight[i] = exp(-dis[i] / (3 * error_reprojection_before));
			//////////////////////////////////////////////////////////////////////////
			//weight[i] = 1;
			//////////////////////////////////////////////////////////////////////////
			error_reprojection += weight[i] * dis[i];
			weight_total += weight[i];
		}
		error_reprojection /= weight_total;
		delta_error = fabs(error_reprojection_before - error_reprojection);

		//printf("Relative orientation error_reprojection =  %lf\n", error_reprojection);
	}

	drow = drow_final;
	dcol = dcol_final;

	for (int i = 0; i < ptNum; i++)
	{
		RPC_ForwardIntersection(rpc_left, rpc_right, lx[i], ly[i], rx[i], ry[i], lat[i], lon[i], hei[i]);

		double lx_reproj, ly_reproj;
		double rx_reproj, ry_reproj;
		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_left, ly_reproj, lx_reproj);
		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, ry_reproj, rx_reproj);

		double disL, disR;
		disL = sqrt((lx_reproj - lx[i])*(lx_reproj - lx[i]) + (ly_reproj - ly[i])*(ly_reproj - ly[i]));
		disR = sqrt((rx_reproj - rx[i])*(rx_reproj - rx[i]) + (ry_reproj - ry[i])*(ry_reproj - ry[i]));

		dis[i] = (disL + disR) / 2;
	}

	error_reprojection = 0;
	for (int i = 0; i < ptNum; i++)
	{
		error_reprojection += dis[i];
	}
	error_reprojection /= ptNum;

// 	memset(inliers, 0, sizeof(bool)*ptNum);
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		if (dis[i] < feaM.m_thresh_inliers)
// 		{
// 			inliers[i] = 1;
// 		}
// 	}

	//free memory
	delete[]lat; lat = NULL;
	delete[]lon; lon = NULL;
	delete[]hei; hei = NULL;
	delete[]dis; dis = NULL;
	delete[]drow_best; drow_best = NULL;
	delete[]dcol_best; dcol_best = NULL;
	delete[]dis_best; dis_best = NULL;
	delete[]weight; weight = NULL;

	return 1;
}

int CRPC_Fun::RelativeOrientation_bias_constant(double *lx, double *ly, double *rx, double *ry, int ptNum, 
	RPC &rpc_left_ori, RPC &rpc_right_ori, bool *inliers)
{
	RPC rpc_left, rpc_right;
	rpc_left = rpc_left_ori;
	rpc_right = rpc_right_ori;

	double *lat, *lon, *hei;
	lat = new double[ptNum];
	lon = new double[ptNum];
	hei = new double[ptNum];

	double *dis = new double[ptNum];
	memset(dis, 0, sizeof(double)*ptNum);

	double error_reprojection = 0;
// 	for (int i = 0; i < ptNum; i++)
// 	{
// 		RPC_ForwardIntersection(rpc_left, rpc_right, lx[i], ly[i], rx[i], ry[i], lat[i], lon[i], hei[i]);
// 
// 		double lx_reproj, ly_reproj;
// 		double rx_reproj, ry_reproj;
// 		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_left, ly_reproj, lx_reproj);
// 		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, ry_reproj, rx_reproj);
// 
// 		double disL, disR;
// 		disL = sqrt((lx_reproj - lx[i])*(lx_reproj - lx[i]) + (ly_reproj - ly[i])*(ly_reproj - ly[i]));
// 		disR = sqrt((rx_reproj - rx[i])*(rx_reproj - rx[i]) + (ry_reproj - ry[i])*(ry_reproj - ry[i]));
// 
// 		dis[i] = (disL + disR) / 2;
// 
// 		error_reprojection += dis[i];
// 	}
// 	error_reprojection /= ptNum;
// 	if (error_reprojection<0.5)
// 	{
// 		memset(inliers, 0, sizeof(bool)*ptNum);
// 		for (int i = 0; i < ptNum; i++)
// 		{
// 			if (dis[i] < feaM.m_thresh_inliers)
// 			{
// 				inliers[i] = 1;
// 			}
// 		}
// 
// 		//free memroy
// 		delete[]lat; lat = NULL;
// 		delete[]lon; lon = NULL;
// 		delete[]hei; hei = NULL;
// 		delete[]dis; dis = NULL;
// 
// 		return 1;
// 	}

	CImageFeatureMatching feaM;
	double *drow_best, *dcol_best, *dis_best;
	drow_best = new double[ptNum];
	dcol_best = new double[ptNum];
	dis_best = new double[ptNum];
	//intialize
	for (int i = 0; i < ptNum;i++)
	{
		drow_best[i] = 999999;
		dcol_best[i] = 999999;
		dis_best[i] = 999999;
	}

	double tZ;
	double drow_final, dcol_final;
	double minZ = rpc_right.heightOffset - rpc_right.heightScale;
	double maxZ = rpc_right.heightOffset + rpc_right.heightScale;
	double stepZ = 0.1;
	for (tZ = minZ; tZ <= maxZ;tZ+=stepZ)
	{
		for (int i = 0; i < ptNum;i++)
		{
			RPC_Image2Ground(ly[i], lx[i], tZ, rpc_left, lat[i], lon[i]);
			hei[i] = tZ;

			double iy_right, ix_right;
			RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, iy_right, ix_right);
				
			double tdis = sqrt((iy_right - ry[i])*(iy_right - ry[i]) + (ix_right - rx[i])*(ix_right - rx[i]));
			if (tdis<dis_best[i])
			{
				drow_best[i] = iy_right - ry[i];
				dcol_best[i] = ix_right - rx[i];
				dis_best[i] = tdis;
			}
		}
	}

	//To speed running time
	feaM.GetWeightedCenter(drow_best, ptNum, 2, drow_final);
	feaM.GetWeightedCenter(dcol_best, ptNum, 2, dcol_final);

	RefineRPC_bias_constant(rpc_right, drow_final, dcol_final);

	//According to the new rpc_right, re-compute the re-projection errors
	for (int i = 0; i < ptNum; i++)
	{
		RPC_ForwardIntersection(rpc_left, rpc_right, lx[i], ly[i], rx[i], ry[i], lat[i], lon[i], hei[i]);

		double lx_reproj, ly_reproj;
		double rx_reproj, ry_reproj;
		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_left, ly_reproj, lx_reproj);
		RPC_Ground2Image(lat[i], lon[i], hei[i], rpc_right, ry_reproj, rx_reproj);

		double disL, disR;
		disL = sqrt((lx_reproj - lx[i])*(lx_reproj - lx[i]) + (ly_reproj - ly[i])*(ly_reproj - ly[i]));
		disR = sqrt((rx_reproj - rx[i])*(rx_reproj - rx[i]) + (ry_reproj - ry[i])*(ry_reproj - ry[i]));

		dis[i] = (disL + disR) / 2;
	}

	//recompute the weight
	error_reprojection = 0;
	double weight_total = 0;
	for (int i = 0; i < ptNum;i++)
	{
		if (dis[i]>feaM.m_thresh_inliers)
		{
			dis[i] = feaM.m_thresh_inliers;
		}
		error_reprojection += dis[i];
		weight_total++;
	}
	error_reprojection /= weight_total;

	memset(inliers, 0, sizeof(bool)*ptNum);
	for (int i = 0; i < ptNum;i++)
	{
		if (dis[i]<feaM.m_thresh_inliers)
		{
			inliers[i] = 1;
		}
	}

	printf("reprojection error:= %lf\n", error_reprojection);

	//free memory
	delete[]lat; lat = NULL;
	delete[]lon; lon = NULL;
	delete[]hei; hei = NULL;
	delete[]dis; dis = NULL;
	delete[]drow_best; drow_best = NULL;
	delete[]dcol_best; dcol_best = NULL;
	delete[]dis_best; dis_best = NULL;

	return 1;
}

int CRPC_Fun::RefineRPC_bias_constant(RPC &rpc, double row_bias,double col_bias)
{
	rpc.lineOffset = rpc.lineOffset - row_bias;
	rpc.sampOffset = rpc.sampOffset - col_bias;

	return 1;
}