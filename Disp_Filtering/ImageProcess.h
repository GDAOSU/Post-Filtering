#pragma once

typedef unsigned char BYTE;

class CImageProcess
{
public:
	CImageProcess();
	~CImageProcess();

public:
	int SaveAOIFromGivenPath16(char *input_path, char *output_path, int minx, int miny, int w_aoi, int h_aoi);
	int PlaneRectification(char *prjPath, char **imgPath, char **rpc_path, int satNum, char *&rectif_list_path);
	int MultiBands2SingleBand(char *inputPath, char *outputPath);
	int WallisFiltering_tile(char **img_path, char **&output_path, int satNum);
	int WallisFiltering(char *img_path);
	int Tilefusion(
		int startx, int starty, int endx, int endy,
		int tilesize, int Overlapsize, unsigned short int *data, unsigned short int *vbuf, unsigned short int *hbuf);
	int Plane_Rectification(char *input_path, char *rpc_path, char  *output_path);
	int GetTileRectification(char *imgPath, char *rpc_path, int startx, int starty, int cols, int rows, double height,
		char *zone, double minX, double minY, double resolution,
		unsigned short int * image_tile);
	int Bilinear_interpolation(unsigned short int *img, int w, int h, double ix, double iy, unsigned short int &intensity);
	int ReadImgInfo(char **imgPath, int satNum, int *&w, int *&h, int *&bands, int *&bitcount);
	int ReadImageData16(char *imgPath, unsigned short int *&img_data, int &w, int &h, int &bands);
	int ReadImageData8(char *imgPath, BYTE *&img_data, int &w, int &h, int &bands);
	int GetImageSize(char *imgPath, int &w, int &h);

	int CreatePyramidImage(char *rectif_list_path, int pyrsize, int pyrlevel, char *Pyramid_list_path);
	int CreatePyramid_onelevel(unsigned short int *OriginImg, int w, int h, int size,
		unsigned short int *PyImg, int pw, int ph);
	int img_16_to_8bits(unsigned short int *img_16, unsigned char *img_8, int w, int h);

public:
	double m_rectification_resolution;
	double m_avgH;
	int m_overlap_size;
	int m_tile_size;
	char m_zone[10];
};

