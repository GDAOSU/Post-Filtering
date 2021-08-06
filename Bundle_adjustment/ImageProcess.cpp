#include "stdafx.h"
#include "ImageProcess.h"
#include "imagebase.h"
#include "FileOperation.h"
#include "walis_HTYY.hpp"
#include "LL_UTM.h"
#include "direct.h"
#include "RPC_Fun.h"

#define MaxLen 512

CImageProcess::CImageProcess()
{
	m_rectification_resolution = 0.5;
	m_avgH = 0;
	m_tile_size = 10000;
	m_overlap_size = 200;
}


CImageProcess::~CImageProcess()
{
}

int CImageProcess::SaveAOIFromGivenPath16(char *input_path, char *output_path, int minx, int miny, int w_aoi, int h_aoi)
{
	unsigned short int *img_aoi = new unsigned short int[w_aoi*h_aoi];
	CImageBase img;
	img.Open(input_path);
	img.Read(img_aoi, minx, miny, w_aoi, h_aoi);
	img.Close();

	SaveImageFile(output_path, (unsigned char *)img_aoi, w_aoi, h_aoi, 1, 16);

	//free memory
	delete[]img_aoi; img_aoi = NULL;
	return 1;
}

int CImageProcess::PlaneRectification(char *prjPath, char **imgPath, char **rpc_path, int satNum, char *&rectif_list_path)
{
	rectif_list_path = new char[MaxLen];
	CFileOperation file;
	//According to the prjPath, create the fold of rectified image
	char RootPath[MaxLen];
	file.FindAllPath(prjPath, "Plane_Rectification", rectif_list_path, RootPath);
	_mkdir(RootPath);

	//decide the name of rectified image
	char **rectif_path = new char *[satNum];
	int i;
	for (i = 0; i < satNum;i++)
	{
		rectif_path[i] = new char[MaxLen];
	}

	for (i = 0; i < satNum;i++)
	{
		char Drive[MaxLen];
		char dir[MaxLen];
		char imgName[MaxLen];
		char Ext[MaxLen];
		_splitpath(imgPath[i], Drive, dir, imgName, Ext);

		strcpy(rectif_path[i], RootPath);
		strcat(rectif_path[i], "\\");
		strcat(rectif_path[i], imgName);
		strcat(rectif_path[i], "_rectif.tif");
	}

	//Writing list file
	FILE *fp_list = fopen(rectif_list_path, "w");
	fprintf(fp_list, "%d\n", satNum);
	for (i = 0; i < satNum;i++)
	{
		fprintf(fp_list, "%s\n", rectif_path[i]);
	}
	fclose(fp_list); fp_list = NULL;

	//multi-bands to PAN
	char **single_path;
	single_path = new char *[satNum];
	for (i = 0; i < satNum;i++)
	{
		single_path[i] = new char[MaxLen];
		file.FindDirPath_prj(rectif_path[i], single_path[i],true);
		strcat(single_path[i], "_single.tif");
	}
	for (i = 0; i < satNum;i++)
	{
		MultiBands2SingleBand(imgPath[i], single_path[i]);
	}

	//Wallis Filtering
	char **wallis_path;
	WallisFiltering_tile(single_path, wallis_path, satNum);

	for (i = 0; i < satNum;i++)
	{
		remove(single_path[i]);
	}

	//Plane rectification
	for (i = 0; i < /*1*/satNum;i++)
	{
		Plane_Rectification(wallis_path[i], rpc_path[i], rectif_path[i]);
	}
	for (i = 0; i < satNum; i++)
	{
		remove(wallis_path[i]);
	}

	//free memory
	for (i = 0; i < satNum;i++)
	{
		delete[]rectif_path[i]; rectif_path[i] = NULL;
		delete[]wallis_path[i]; wallis_path[i] = NULL;
		delete[]single_path[i]; single_path[i] = NULL;
	}
	delete[]wallis_path; wallis_path = NULL;
	delete[]rectif_path; rectif_path = NULL;
	delete[]single_path; single_path = NULL;
	return 1;
}

int CImageProcess::MultiBands2SingleBand(char *inputPath, char *outputPath)
{
	CImageBase img;
	img.Open(inputPath);
	int w, h, bands;
	int bits;
	w = img.GetCols();
	h = img.GetRows();
	bands = img.GetBands();
	bits = img.GetBits();

	int x, y;
	if (bands == 1)
	{
		CImageBase img_out;
		img_out.Create(outputPath, w, h, bands, bits);

		unsigned short int *img_ori;
// 		if (bits == 16)
// 		{
		img_ori = new unsigned short int[m_tile_size*m_tile_size*bands];
// 		}
// 		else
// 			img_ori = new BYTE[tile_size*tile_size*bands];

		int startx, starty, endx, endy;
		startx = 0; starty = 0;
		endx = startx + m_tile_size - 1;
		endy = starty + m_tile_size - 1;
		if (endx > w - 1) endx = w - 1;
		if (endy > h - 1) endy = h - 1;

		while (endy<h)
		{
			while (endx<w)
			{
				//code
				img.Read(img_ori, startx, starty, endx - startx + 1, endy - starty + 1);

				//wallis filtering
// 				WallisFlt(img_ori, endx - startx + 1, endy - starty + 1);
// 				img.Read(vBuf, endx + 1 - overlap_size, starty, overlap_size, endy - starty + 1);
// 				img.Read(hBuf, startx, endy + 1 - overlap_size, endx - startx + 1, overlap_size);
// 				Tilefusion(startx, starty, endx, endy, tile_size, overlap_size,
// 					img_ori, vBuf, hBuf);

				img_out.Write(img_ori, startx, starty, endx - startx + 1, endy - starty + 1);

				if (endx==w-1)
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

			if (endy==h-1)
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
		img_out.Close();
		delete[]img_ori; img_ori = NULL;
	}
	else if (bands>1)
	{
		CImageBase img_out;
		img_out.Create(outputPath, w, h, 1, bits);

		unsigned short int *img_ori, *img_tile_out;
		if (bits == 16)
		{
			img_ori = new unsigned short int[m_tile_size*m_tile_size*bands];
			img_tile_out = new unsigned short int[m_tile_size*m_tile_size];
		}
		else
		{
// 			img_ori = new short int[tile_size*tile_size*bands];
// 			img_tile_out = new short int[tile_size*tile_size];

			img.Close();
			return 0;
		}

		int startx, starty, endx, endy;
		startx = 0; starty = 0;
		endx = startx + m_tile_size - 1;
		endy = starty + m_tile_size - 1;
		if (endx > w - 1) endx = w - 1;
		if (endy > h - 1) endy = h - 1;

		while (endy < h)
		{
			while (endx < w)
			{
				//code
				int w_tile, h_tile;
				w_tile = endx - startx + 1;
				h_tile = endy - starty + 1;
				img.Read(img_ori, startx, starty, w_tile, h_tile);

				int b;
				float img_tile_result = 0;
				for (y = 0; y < h_tile; y++)
					for (x = 0; x < w_tile;x++)
					{
						img_tile_result = 0;
						for (b = 0; b < bands;b++)
						{
							img_tile_result += img_ori[b*w_tile*h_tile + y*w_tile + x];
						}

						img_tile_result /= bands;
						img_tile_out[y*w_tile + x] = (unsigned short int)(img_tile_result+0.5f);
					}

				img_out.Write(img_tile_out, startx, starty, w_tile, h_tile);

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
		img_out.Close();
		delete[]img_ori; img_ori = NULL;
		delete[]img_tile_out; img_tile_out = NULL;
	}

	//free memory
	img.Close();
	return 1;
}

int CImageProcess::ReadImgInfo(char **imgPath, int satNum, int *&w, int *&h, int *&bands, int *&bitcount)
{
	int i;
	w = new int[satNum];
	h = new int[satNum];
	bands = new int[satNum];
	bitcount = new int[satNum];
	for (i = 0; i < satNum;i++)
	{
		CImageBase img;
		img.Open(imgPath[i]);
		w[i] = img.GetCols();
		h[i] = img.GetRows();
		bands[i] = img.GetBands();
		bitcount[i] = img.GetBits();
		img.Close();
	}

	return 1;
}


int CImageProcess::GetImageSize(char *imgPath, int &w, int &h)
{
	CImageBase img;
	img.Open(imgPath);
	w = img.GetCols();
	h = img.GetRows();
	img.Close();
	return 1;
}

int CImageProcess::ReadImageData16(char *imgPath, unsigned short int *&img_data, int &w, int &h, int &bands)
{
	CImageBase img;
	img.Open(imgPath);
	w = img.GetCols();
	h = img.GetRows();
	bands = img.GetBands();

	img_data = new unsigned short int[w*h];
	img.Read(img_data, 0, 0, w, h);

	img.Close();
	return 1;
}

int ReadImageData8(char *imgPath, BYTE *&img_data, int &w, int &h, int &bands)
{
	CImageBase img;
	img.Open(imgPath);
	w = img.GetCols();
	h = img.GetRows();
	bands = img.GetBands();

	img_data = new BYTE[w*h];
	img.Read(img_data, 0, 0, w, h);

	img.Close();

	return 1;
}

int CImageProcess::WallisFiltering_tile(char **img_path, char **&output_path, int satNum)
{
	//create output_path
	CFileOperation file;
	output_path = new char *[satNum];
	int i;
	for (i = 0; i < satNum;i++)
	{
		output_path[i] = new char[MaxLen];
	}
	for (i = 0; i < satNum;i++)
	{
		file.FindDirPath_prj(img_path[i], output_path[i], true);
		strcat(output_path[i], "_wallis.tif");
	}

	for (i = 0; i < satNum;i++)
	{
		//original image
		CImageBase img;
		img.Open(img_path[i]);
		int w, h, bands;
		int bits;
		w = img.GetCols();
		h = img.GetRows();
		bands = img.GetBands();
		bits = img.GetBits();

		//create empty image
		CImageBase img_out;
		img_out.Create(output_path[i], w, h, 1, 16);
		int startx, starty, endx, endy;
		startx = 0; starty = 0;
		endx = startx + m_tile_size - 1;
		endy = starty + m_tile_size - 1;
		if (endx > w - 1) endx = w - 1;
		if (endy > h - 1) endy = h - 1;

		unsigned short int *img_emp;
		img_emp = new unsigned short int[m_tile_size*m_tile_size*bands];
		memset(img_emp, 0, sizeof(unsigned short int)*m_tile_size*m_tile_size*bands);
		while (endy < h)
		{
			while (endx < w)
			{
				img_out.Write(img_emp, startx, starty, endx - startx + 1, endy - starty + 1);

				if (endx == w - 1) { break; }
				else {
					startx = endx + 1;
					endx = startx + m_tile_size - 1;
					if (endx > w - 1) endx = w - 1;
				}
			}

			if (endy == h - 1) {  break;  }
			else {
				startx = 0;
				endx = startx + m_tile_size - 1;
				if (endx > w - 1) endx = w - 1;
				starty = endy + 1;
				endy = starty + m_tile_size - 1;
				if (endy > h - 1) endy = h - 1;
			}
		}
		delete[]img_emp; img_emp = NULL;
		img_out.Close();

		//processing
		img_out.Open(output_path[i], CImageBase::modeReadWrite);

//		int x, y;
		unsigned short int *vBuf = new unsigned short int[m_tile_size*m_overlap_size];
		unsigned short int *hBuf = new unsigned short int[m_tile_size*m_overlap_size];

		unsigned short int *img_ori;
		img_ori = new unsigned short int[m_tile_size*m_tile_size*bands];

		//int startx, starty, endx, endy;
		startx = 0; starty = 0;
		endx = startx + m_tile_size - 1;
		endy = starty + m_tile_size - 1;
		if (endx > w - 1) endx = w - 1;
		if (endy > h - 1) endy = h - 1;

		while (endy < h)
		{
			while (endx < w)
			{
				img.Read(img_ori, startx, starty, endx - startx + 1, endy - starty + 1);
				
				//wallis filtering
				WallisFlt(img_ori, endx - startx + 1, endy - starty + 1);

				img_out.Read(vBuf, startx, starty, m_overlap_size, endy - starty + 1);
				img_out.Read(hBuf, startx, starty, endx - startx + 1, m_overlap_size);
				Tilefusion(startx, starty, endx, endy, m_tile_size, m_overlap_size,
					img_ori, vBuf, hBuf);

				img_out.Write(img_ori, startx, starty, endx - startx + 1, endy - starty + 1);

				if (endx == w - 1)
				{
					break;
				}
				else
				{
					startx = endx + 1 - m_overlap_size;
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
				starty = endy + 1 - m_overlap_size;
				endy = starty + m_tile_size - 1;
				if (endy > h - 1) endy = h - 1;
			}
		}

		//free memory
		delete[]img_ori; img_ori = NULL;

		//free memory
		img.Close();
		img_out.Close();
		delete[]vBuf; vBuf = NULL;
		delete[]hBuf; hBuf = NULL;
	}
	
	return 1;
}

int CImageProcess::WallisFiltering(char *img_path)
{
	CImageBase img;
	img.Open(img_path);
	int w, h, bands, bits;
	w = img.GetCols();
	h = img.GetRows();
	bands = img.GetBands();
	bits = img.GetBits();

	unsigned short int *img_data = new unsigned short int[w*h];
	img.Read(img_data, 0, 0, w, h);
	img.Close();

	unsigned short int *img_data_backup = new unsigned short int[w*h];
	memcpy(img_data_backup, img_data, sizeof(unsigned short int)*w*h);
	WallisFlt(img_data, w, h);
	int x, y;
	for (y = 0; y < h; y++)
		for (x = 0; x < w;x++)
		{
			if (img_data_backup[y*w+x]==0)
			{
				img_data[y*w + x] = 0;
			}
		}

	SaveImageFile(img_path, (unsigned char *)img_data, w, h, bands, bits);
	delete[]img_data; img_data = NULL;
	delete[]img_data_backup; img_data_backup = NULL;
	return 1; 
}

int CImageProcess::Tilefusion(
	int startx, int starty, int endx, int endy,
	int tilesize, int Overlapsize, unsigned short int *data, unsigned short int *vbuf, unsigned short int *hbuf)
{
	int tileh, tilew;
	tileh = endy - starty + 1;
	tilew = endx - startx + 1;

	//重叠覆盖有四种情况
	int x, y;
	if (starty == 0) //第一行块
	{
		if (startx == 0) //第一行第一块
		{
			//img_out.Write(data, startx, starty, tilew, tileh);
		}
		else
		{
			//左右重叠
			int toverLap = Overlapsize;
			float Dl, Dr;
			float dl, dr;
			for (y = 0; y < tileh; y++)
				for (x = 0; x < toverLap; x++)
				{
					Dl = (float)x; Dr = (float)((toverLap - 1) - x);
					dl = vbuf[y*toverLap + x];
					dr = data[y*tilew + x];

					data[y*tilew + x] = (unsigned short int)((Dr / (Dl + Dr)*dl + Dl / (Dl + Dr)*dr) + 0.5f);
				}
		}
	}
	else
	{
		if (startx == 0) //换行后的第一块数据,只有上下重叠
		{
			int toverLap = Overlapsize;
			double Dt, Db;
			double dt, db;
			for (y = 0; y < toverLap; y++)
				for (x = 0; x < tilew; x++)
				{
					Db = y; Dt = (toverLap - 1) - y;
					db = hbuf[y*tilew + x];
					dt = data[y*tilew + x];

					data[y*tilew + x] = (unsigned short int)((Dt / (Dt + Db)*db + Db / (Dt + Db)*dt) + 0.5f);
				}
		}
		else
		{
			//左右重叠
			int toverLap = Overlapsize;
			float Dl, Dr;
			float dl, dr;
			for (y = 0; y < tileh; y++)
				for (x = 0; x < toverLap; x++)
				{
					Dl = (float)x; Dr = (float)((toverLap - 1) - x);
					dl = vbuf[y*toverLap + x];
					dr = data[y*tilew + x];

					data[y*tilew + x] = (unsigned short int)((Dr / (Dl + Dr)*dl + Dl / (Dl + Dr)*dr) + 0.5f);
				}

			//上下重叠
			double Dt, Db;
			double dt, db;
			for (y = 0; y < toverLap; y++)
				for (x = 0; x < tilew; x++)
				{
					Db = y; Dt = (toverLap - 1) - y;
					db = hbuf[y*tilew + x];
					dt = data[y*tilew + x];

					data[y*tilew + x] = (unsigned short int)((Dt / (Dt + Db)*db + Db / (Dt + Db)*dt) + 0.5f);
				}
		}
	}

	return 1;
}

int CImageProcess::Plane_Rectification(char *input_path, char *rpc_path, char  *output_path)
{
	CFileOperation file;
	CRPC_Fun rpcf;
	CLL_UTM geo_conv;

	char tfwPath[MaxLen];
	file.FindDirPath_prj(output_path, tfwPath, true);
	strcat(tfwPath, ".tfw");

	//read rpc file
	RPC rpc;
	rpcf.ReadRpc_singleFile(rpc_path, rpc);

	//compute the region of the image
	CImageBase img;
	int i;
	img.Open(input_path);
	int iw, ih;
	iw = img.GetCols();
	ih = img.GetRows();
	double col_corner[4] = { 0, iw - 1, iw - 1, 0 }, row_corner[4] = { 0, 0, ih - 1, ih - 1 };
	double Lat_corner[4], Lon_corner[4];
	int avgHeight = m_avgH;
	for (i = 0; i < 4;i++)
	{
		rpcf.RPC_Image2Ground(row_corner[i], col_corner[i], avgHeight, rpc, Lat_corner[i], Lon_corner[i]);
	}

	//compute zone and conduct LL to UTM
// 	double cenLat = 0, cenLon = 0;
// 	for (i = 0; i < 4;i++)
// 	{
// 		cenLat += Lat_corner[i];
// 		cenLon += Lon_corner[i];
// 	}
// 	cenLat /= 4; cenLon /= 4;
// 
// 	char zone[10];
// 	double tX, tY;
// 	geo_conv.LL2UTM(cenLat, cenLon, tX, tY, zone);

	double utmX_corner[4], utmY_corner[4];
	for (i = 0; i < 4;i++)
	{
		geo_conv.LL2UTM_fixedZone(Lat_corner[i], Lon_corner[i], m_zone, utmX_corner[i], utmY_corner[i]);
	}

	//compute the bounding box
	double minX = utmX_corner[0], minY = utmY_corner[0], maxX = utmX_corner[0], maxY = utmY_corner[0];
	for (i = 1; i < 4;i++)
	{
		if (minX > utmX_corner[i]) minX = utmX_corner[i];
		if (maxX < utmX_corner[i]) maxX = utmX_corner[i];

		if (minY > utmY_corner[i]) minY = utmY_corner[i];
		if (maxY < utmY_corner[i]) maxY = utmY_corner[i];
	}

	//create rectification results
	int gW, gH;
	gW = (int)((maxX - minX)/m_rectification_resolution);
	gH = (int)((maxY - minY)/m_rectification_resolution);

	CImageBase img_out;
	img_out.Create(output_path, gW, gH, 1, 16);

	int startx, starty, endx, endy;
	startx = 0; starty = 0;
	endx = startx + m_tile_size - 1;
	endy = starty + m_tile_size - 1;
	if (endx > gW - 1) endx = gW - 1;
	if (endy > gH - 1) endy = gH - 1;
	unsigned short int *img_tile = new unsigned short int[m_tile_size*m_tile_size];
	memset(img_tile, 0, sizeof(unsigned short int)*m_tile_size*m_tile_size);
	while (endy < gH)
	{
		while (endx < gW)
		{
			//Initialize
			img_out.Write(img_tile, startx, starty, endx - startx + 1, endy - starty + 1);

			if (endx == gW - 1)
			{
				break;
			}
			else
			{
				startx = endx + 1;
				endx = startx + m_tile_size - 1;
				if (endx > gW - 1) endx = gW - 1;
			}
		}

		if (endy == gH - 1)
		{
			break;
		}
		else
		{
			startx = 0;
			endx = startx + m_tile_size - 1;
			if (endx > gW - 1) endx = gW - 1;
			starty = endy + 1;
			endy = starty + m_tile_size - 1;
			if (endy > gH - 1) endy = gH - 1;
		}
	}
	img_out.Close();

	//compute rectification results in tiles
/*	CImageBase img_out;*/
	img_out.Open(output_path, CImageBase::modeReadWrite);

	startx = 0; starty = 0;
	endx = startx + m_tile_size - 1;
	endy = starty + m_tile_size - 1;
	if (endx > gW - 1) endx = gW - 1;
	if (endy > gH - 1) endy = gH - 1;
/*	unsigned short int *img_tile = new unsigned short int[tile_size*tile_size];*/
	memset(img_tile, 0, sizeof(unsigned short int)*m_tile_size*m_tile_size);
	while (endy < gH)
	{
		while (endx < gW)
		{
			//tile_rectification_process
			GetTileRectification(input_path, rpc_path, startx, starty, endx - startx + 1, endy - starty + 1, m_avgH, m_zone,
				minX, minY, m_rectification_resolution, img_tile);
			img_out.Write(img_tile, startx, starty, endx - startx + 1, endy - starty + 1);
			
			if (endx == gW - 1)
			{
				break;
			}
			else
			{
				startx = endx + 1;
				endx = startx + m_tile_size - 1;
				if (endx > gW - 1) endx = gW - 1;
			}
		}

		if (endy == gH - 1)
		{
			break;
		}
		else
		{
			startx = 0;
			endx = startx + m_tile_size - 1;
			if (endx > gW - 1) endx = gW - 1;
			starty = endy + 1;
			endy = starty + m_tile_size - 1;
			if (endy > gH - 1) endy = gH - 1;
		}
	}

//	WallisFiltering(output_path);

	FILE*fp_tfw = fopen(tfwPath, "w");
	fprintf(fp_tfw, "%lf\n%lf\n%lf\n%lf\n%lf\n%lf\n", m_rectification_resolution, 0, 0, m_rectification_resolution, minX, minY);
	fclose(fp_tfw); fp_tfw = NULL;

	//freem memory
	img_out.Close();
	img.Close();
	delete[]img_tile; img_tile = NULL;
	return 1;
}


int CImageProcess::GetTileRectification(char *imgPath, char *rpc_path, int startx, int starty, int Cols, int Rows, double height,
	char *zone, double minX, double minY, double resolution,
	unsigned short int * image_tile)
{
	CLL_UTM geo_conv;
	CRPC_Fun rpcf;
	RPC rpc;
	rpcf.ReadRpc_singleFile(rpc_path, rpc);

	//Read image
	CImageBase img;
	img.Open(imgPath);
	int iw, ih, bands, bits;
	iw = img.GetCols();
	ih = img.GetRows();
	bands = img.GetBands();
	bits = img.GetBits();

	//compute the image tile that corresponding to this region
	int start_row = ih - 1, start_col = iw - 1, end_row = 0, end_col = 0;
	int x, y;
	
	for (y = 0; y < Rows; y++)
		for (x = 0; x < Cols;x++)
		{
			double utmX, utmY;
			utmX = minX + (x + startx)*resolution;
			utmY = minY + (y + starty)*resolution;

			double Lat = 0, Lon = 0;
			geo_conv.UTM2LL(utmX, utmY, zone, Lat, Lon);

			double row, column;
			rpcf.RPC_Ground2Image(Lat, Lon, height, rpc, row, column);

			if (row >= 0 && row <= ih - 0.5 && column >= 0 && column <= iw - 0.5) //Point in image
			{
				if (start_row > row) start_row = (int)row;
				if (end_row < row) end_row = (int)row + 1;
				if (start_col > column) start_col = (int)column;
				if (end_col < column) end_col = (int)column + 1;
			}
		}

	if (end_row > ih - 1) end_row = ih - 1;
	if (end_col > iw - 1) end_col = iw - 1;

	int iw_tile, ih_tile;
	ih_tile = end_row - start_row + 1;
	iw_tile = end_col - start_col + 1;

	unsigned short int *image_tile0 = new unsigned short int[iw_tile*ih_tile];
	img.Read(image_tile0, start_col, start_row, iw_tile, ih_tile);

	//Then, compute the image tile on the plane rectification results
	memset(image_tile, 0, sizeof(unsigned short int)*Rows*Cols);
	for (y = 0; y < Rows; y++)
		for (x = 0; x < Cols; x++)
		{
			double utmX, utmY;
			utmX = minX + (x + startx)*resolution;
			utmY = minY + (y + starty)*resolution;

			double Lat = 0, Lon = 0;
			geo_conv.UTM2LL(utmX, utmY, zone, Lat, Lon);

			double row, column;
			rpcf.RPC_Ground2Image(Lat, Lon, height, rpc, row, column);

			if (row >= 0 && row <= ih - 0.5 && column >= 0 && column <= iw - 0.5) //Point in image
			{
				double row_relative, col_relative;
				row_relative = row - start_row;
				col_relative = column - start_col;

				unsigned short int intensity = 0;
				//Intersity Bilinear Interpolation
				Bilinear_interpolation(image_tile0, iw_tile, ih_tile, col_relative, row_relative, intensity);

				image_tile[y*Cols + x] = intensity;
			}
		}

	//free memory
	img.Close(); 
	delete[]image_tile0; image_tile0 = NULL;
	return 1;
}


int CImageProcess::Bilinear_interpolation(unsigned short int *img, int w, int h, double ix, double iy, unsigned short int &intensity)
{
	int px, py;
	px = (int)ix; py = (int)iy;
	double u, v;
	u = ix - px; v = iy - py;

	unsigned short int I1, I2, I3, I4;
	if (px >= 0 && px < w && py >= 0 && py < h)
		I1 = img[py*w + px];
	else
		I1 = 0;

	if (px + 1 >= 0 && px + 1 < w && py >= 0 && py < h)
		I2 = img[py*w + (px + 1)];
	else
		I2 = 0;

	if (px + 1 >= 0 && px + 1 < w && py + 1 >= 0 && py + 1 < h)
		I3 = img[(py + 1)*w + (px + 1)];
	else
		I3 = 0;

	if (px >= 0 && px < w && py + 1 >= 0 && py + 1 < h)
		I4 = img[(py + 1)*w + px];
	else
		I4 = 0;

	intensity = (unsigned short int)((1 - u)*(1 - v)*I1 + (1 - u)*v*I4 + u*(1 - v)*I2 + u*v*I3 + 0.5);

	return 1;
}

int CImageProcess::CreatePyramidImage(char *rectif_list_path, int pyrsize, int pyrlevel, char *Pyramid_list_path)
{
	CFileOperation file;

	char PyrFolderPath[MaxLen];
	file.FindDirPath_prj(Pyramid_list_path, PyrFolderPath, true);

	FILE *fp_rectify = fopen(rectif_list_path, "r");
	int imgNum = 0;
	fscanf(fp_rectify, "%d", &imgNum);
	
	FILE *fp_pyr = fopen(Pyramid_list_path, "w");
	fprintf(fp_pyr, "%d %d %d\n", imgNum, pyrlevel, pyrsize);
	int i;
	for (i = 0; i < imgNum;i++)
	{
		//Read rectified image path
		char rectify_img_path[MaxLen];
		fscanf(fp_rectify, "%s", rectify_img_path);

		char FileName[MaxLen];
		file.GetFileName(rectify_img_path, FileName);

		//Writing list file
		char **pyr_img_path;
		pyr_img_path = new char *[pyrlevel];
		int j;
		for (j = 0; j < pyrlevel;j++)
		{
			pyr_img_path[j] = new char[MaxLen];
		}

		for (j = 0; j < pyrlevel;j++)
		{
			file.FindDirPath_prj(Pyramid_list_path, pyr_img_path[j], true);
			strcat(pyr_img_path[j], "\\");
			strcat(pyr_img_path[j], FileName);
			char str[20];
			sprintf(str, "_%d.tif", j);
			strcat(pyr_img_path[j], str);
		}

		for (j = 0; j < pyrlevel;j++)
		{
			fprintf(fp_pyr, "%s\n", pyr_img_path[j]);
		}

		//pyramid image generation
		int pixel_size_top = (int)(powl(pyrsize, pyrlevel - 1) + 0.5);
		int tilesize = m_tile_size - m_tile_size%pixel_size_top;

		CImageBase img_read, *img_write;
		img_write = new CImageBase[pyrlevel];
		img_read.Open(rectify_img_path);
		int *w, *h;
		int bands, bits;
		w = new int[pyrlevel];
		h = new int[pyrlevel];
		w[0] = img_read.GetCols();
		h[0] = img_read.GetRows();
		bands = img_read.GetBands();
		bits = img_read.GetBits();

		//cut the  redundant edges
		w[0] = w[0] - w[0] % pixel_size_top;
		h[0] = h[0] - h[0] % pixel_size_top;
		img_write[0].Create(pyr_img_path[0], w[0], h[0], bands, bits);
		for (j = 1; j < pyrlevel;j++)
		{
			w[j] = w[j - 1] / pyrsize;
			h[j] = h[j - 1] / pyrsize;
			img_write[j].Create(pyr_img_path[j], w[j], h[j], bands, bits);
		}

		//tile processing
		unsigned short int **img_data;
		img_data = new unsigned short int *[pyrlevel];
		for (j = 0; j < pyrlevel;j++)
		{
			int tpixel_size = (int) (powl(pyrsize, j) + 0.5);
			int tsize = tilesize / tpixel_size;
			img_data[j] = new unsigned short int[tsize*tsize*bands];
		}

		int startx, starty, endx, endy;
		startx = 0; starty = 0;
		endx = startx + tilesize - 1;
		endy = starty + tilesize - 1;
		if (endx > w[0] - 1) endx = w[0] - 1;
		if (endy > h[0] - 1) endy = h[0] - 1;

		while (endy < h[0])
		{
			while (endx < w[0])
			{
				//code
				img_read.Read(img_data[0], startx, starty, endx - startx + 1, endy - starty + 1);

				//Pyramid
				img_write[0].Write(img_data[0], startx, starty, endx - startx + 1, endy - starty + 1);
				for (j = 1; j < pyrlevel; j++)
				{
					int tpixel_size0 = (int)(powl(pyrsize, j - 1) + 0.5);
					int tpixel_size1 = (int)(powl(pyrsize, j) + 0.5);
					//pyramid image generation
					CreatePyramid_onelevel(img_data[j - 1], (endx - startx + 1) / tpixel_size0, (endy - starty + 1) / tpixel_size0, pyrsize,
						img_data[j], (endx - startx + 1) / tpixel_size1, (endy - starty + 1) / tpixel_size1);
					img_write[j].Write(img_data[j], startx / tpixel_size1, starty / tpixel_size1, (endx - startx + 1) / tpixel_size1,
						(endy - starty + 1) / tpixel_size1);
				}

				if (endx == w[0] - 1)
				{
					break;
				}
				else
				{
					startx = endx + 1;
					endx = startx + tilesize - 1;
					if (endx > w[0] - 1) endx = w[0] - 1;
				}
			}

			if (endy == h[0] - 1)
			{
				break;
			}
			else
			{
				startx = 0;
				endx = startx + tilesize - 1;
				if (endx > w[0] - 1) endx = w[0] - 1;
				starty = endy + 1;
				endy = starty + tilesize - 1;
				if (endy > h[0] - 1) endy = h[0] - 1;
			}
		}

		//free memory
		for (j = 0; j < pyrlevel;j++)
		{
			delete[]pyr_img_path[j];
			img_write[j].Close();
			delete[]img_data[j]; img_data[j] = NULL;
		}
		delete[]img_write; img_write = NULL;
		delete[]img_data; img_data = NULL;
		img_read.Close();
		delete[]w; w = NULL;
		delete[]h; h = NULL;
		delete[]pyr_img_path; pyr_img_path = NULL;
		delete[]img_data; img_data = NULL;
	}
	fclose(fp_rectify); fp_rectify = NULL;
	fclose(fp_pyr); fp_pyr = NULL;
	return 1;
}

int CImageProcess::CreatePyramid_onelevel(unsigned short int *OriginImg, int w, int h, int size,
	unsigned short int *PyImg, int pw, int ph)
{
	memset(PyImg, 0, sizeof(unsigned short int)*pw*ph);

	double *tmp_PyImg = new double[pw*ph];
	memset(tmp_PyImg, 0, sizeof(double)*pw*ph);

	int y, x;
	for (y = 0; y < ph; y++)
		for (x = 0; x < pw; x++)
		{
			int tx, ty;
			for (ty = y*size; ty < (y + 1)*size; ty++)
				for (tx = x*size; tx < (x + 1)*size; tx++)
				{
					tmp_PyImg[y*pw + x] += OriginImg[ty*w + tx];
				}
		}

	for (y = 0; y < ph; y++)
		for (x = 0; x < pw; x++)
		{
			int tmp = (unsigned short int)(tmp_PyImg[y*pw + x] / (size*size) + 0.5);
			if (tmp > 65535) tmp = 65535;
			PyImg[y*pw + x] = tmp;
		}

	delete[]tmp_PyImg; tmp_PyImg = NULL;

	return 1;
}


int CImageProcess::img_16_to_8bits(unsigned short int *img_16, unsigned char *img_8, int w, int h)
{
	int intensity_min = 65535, intensity_max = 0;
	int x, y;
	for (y = 0; y < h; y++)
		for (x = 0; x < w;x++)
		{
			if (intensity_min > img_16[y*w + x]) intensity_min = img_16[y*w + x];
			if (intensity_max < img_16[y*w + x]) intensity_max = img_16[y*w + x];
		}

	for (y = 0; y < h; y++)
		for (x = 0; x < w; x++)
		{
			int tmp;
			tmp = (int)((double)(img_16[y*w + x] - intensity_min) / (intensity_max - intensity_min) * 255 + 0.5);
			if (tmp > 255) tmp = 255;

			img_8[y*w + x] = tmp;
		}
	return 1;
}
