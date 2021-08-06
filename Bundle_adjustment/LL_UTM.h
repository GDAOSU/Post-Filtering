#pragma once
#include "ellipsoid_utm_info.h"
#include "converter_utm_latlon.h"
#include "math.h"
#include "stdlib.h"

class CLL_UTM
{
public:
	CLL_UTM();
	~CLL_UTM();

public:
	int LL2UTM(double Lat, double Lon, double &x, double &y, char n[10]);
	int LL2UTM_fixedZone(double Lat, double Lon, char n[10], double &x, double &y);
	int UTM2LL(double x, double y, char n[10], double &Lat, double &Lon);
};

