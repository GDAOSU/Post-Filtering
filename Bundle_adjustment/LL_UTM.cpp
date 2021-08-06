#include "stdafx.h"
#include "LL_UTM.h"

#define PI 3.14159265359


CLL_UTM::CLL_UTM()
{
}


CLL_UTM::~CLL_UTM()
{
}

int CLL_UTM::LL2UTM(double Lat, double Lon, double &x, double &y, char n[10])
{
	//initialize
	x = 0; y = y;
	strcpy(n, "60 X");

	double la, lo;
	la = Lat; lo = Lon;

	//ellipse parameters
	double sa = 6378137.000000, sb = 6356752.314245;

	double e2 = sqrt((sa * sa) - (sb * sb)) / sb;
	double e2cuadrada = e2 * e2;
	double c = (sa * sa) / sb;

	double lat = la * (PI / 180);
	double lon = lo * (PI / 180);

	int Huso = (int)((lo / 6) + 31);
	double S = ((Huso * 6) - 183);
	double deltaS = lon - (S * (PI / 180));

	char Letra;
	if (la < -72) Letra = 'C';
	else if (la < -64) Letra = 'D';
	else if (la < -56) Letra = 'E';
	else if (la < -48) Letra = 'F';
	else if (la < -40) Letra = 'G';
	else if (la < -32) Letra = 'H';
	else if (la < -24) Letra = 'J';
	else if (la < -16) Letra = 'K';
	else if (la < -8) Letra = 'L';
	else if (la < 0) Letra = 'M';
	else if (la < 8) Letra = 'N';
	else if (la < 16) Letra = 'P';
	else if (la < 24) Letra = 'Q';
	else if (la < 32) Letra = 'R';
	else if (la < 40) Letra = 'S';
	else if (la < 48) Letra = 'T';
	else if (la < 56) Letra = 'U';
	else if (la < 64) Letra = 'V';
	else if (la < 72) Letra = 'W';
	else Letra = 'X';

	sprintf(n, "%02d%c", Huso, Letra);

	//converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
	//East Longitudes are positive, West longitudes are negative. 
	//North latitudes are positive, South latitudes are negative
	//Lat and Long are in decimal degrees
	//Written by Chuck Gantz- chuck.gantz@globalstar.com
	int ReferenceEllipsoid = 23;
	double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
	double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
	double k0 = 0.9996;

	double LongOrigin;
	double eccPrimeSquared;
	double N, T, C, A, M;

	//Make sure the longitude is between -180.00 .. 179.9
	double LongTemp = (Lon + 180) - int((Lon + 180) / 360) * 360 - 180; // -180.00 .. 179.9;

	double LatRad = Lat*deg2rad;
	double LongRad = LongTemp*deg2rad;
	double LongOriginRad;
	int    ZoneNumber;

	if (strlen(n) != 0)
	{
		char tempstr[10];
		int p = 0;
		for (p = 0; p < strlen(n); p++)
		{
			if (n[p] >= '0' && n[p] <= '9')
				tempstr[p] = n[p];
			else
				break;
		} // end for
		tempstr[p] = 0;

		ZoneNumber = atoi(tempstr);
	}
	else
		ZoneNumber = int((LongTemp + 180) / 6) + 1;

	if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
		ZoneNumber = 32;

	// Special zones for Svalbard
	if (Lat >= 72.0 && Lat < 84.0)
	{
		if (LongTemp >= 0.0  && LongTemp < 9.0) ZoneNumber = 31;
		else if (LongTemp >= 9.0  && LongTemp < 21.0) ZoneNumber = 33;
		else if (LongTemp >= 21.0 && LongTemp < 33.0) ZoneNumber = 35;
		else if (LongTemp >= 33.0 && LongTemp < 42.0) ZoneNumber = 37;
	}
	LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  //+3 puts origin in middle of zone
	LongOriginRad = LongOrigin * deg2rad;

	//compute the UTM Zone from the latitude and longitude
	if (strlen(n) == 0)
		sprintf(n, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));


	eccPrimeSquared = (eccSquared) / (1 - eccSquared);

	N = a / sqrt(1 - eccSquared*sin(LatRad)*sin(LatRad));
	T = tan(LatRad)*tan(LatRad);
	C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
	A = cos(LatRad)*(LongRad - LongOriginRad);

	M = a*((1 - eccSquared / 4 - 3 * eccSquared*eccSquared / 64 - 5 * eccSquared*eccSquared*eccSquared / 256)*LatRad
		- (3 * eccSquared / 8 + 3 * eccSquared*eccSquared / 32 + 45 * eccSquared*eccSquared*eccSquared / 1024)*sin(2 * LatRad)
		+ (15 * eccSquared*eccSquared / 256 + 45 * eccSquared*eccSquared*eccSquared / 1024)*sin(4 * LatRad)
		- (35 * eccSquared*eccSquared*eccSquared / 3072)*sin(6 * LatRad));

	double UTMEasting, UTMNorthing;
	UTMEasting = (double)(k0*N*(A + (1 - T + C)*A*A*A / 6
		+ (5 - 18 * T + T*T + 72 * C - 58 * eccPrimeSquared)*A*A*A*A*A / 120)
		+ 500000.0);

	UTMNorthing = (double)(k0*(M + N*tan(LatRad)*(A*A / 2 + (5 - T + 9 * C + 4 * C*C)*A*A*A*A / 24
		+ (61 - 58 * T + T*T + 600 * C - 330 * eccPrimeSquared)*A*A*A*A*A*A / 720)));
	if (Lat < 0)
		UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere

	y = UTMNorthing;
	x = UTMEasting;

	// 	double a = cos(lat) * sin(deltaS);
	// 	double epsilon = 0.5 * log((1 + a) / (1 - a));
	// 	double nu = atan(tan(lat) / cos(deltaS)) - lat;
	// 	double v = (c / sqrt((1 + (e2cuadrada * cos(lat)*cos(lat))))) * 0.9996;
	// 	double ta = (e2cuadrada / 2) * epsilon * epsilon * cos(lat)*cos(lat);
	// 	double a1 = sin(2 * lat);
	// 	double a2 = a1 * cos(lat)*cos(lat);
	// 	double j2 = lat + (a1 / 2);
	// 	double j4 = ((3 * j2) + a2) / 4;
	// 	double j6 = ((5 * j4) + (a2 * cos(lat)*cos(lat))) / 3;
	// 	double alfa = (3.0 / 4.0) * e2cuadrada;
	// 	double beta = (5.0 / 3.0) * alfa * alfa;
	// 	double gama = (35.0 / 27.0) * alfa* alfa* alfa;
	// 	double Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
	// 	double xx = epsilon * v * (1 + (ta / 3)) + 500000;
	// 	double yy = nu * v * (1 + ta) + Bm;
	// 
	// 	if (yy < 0)
	// 		yy = 9999999 + yy;
	// 
	// 	x = xx;
	// 	y = yy;

	return 1;
}

int CLL_UTM::LL2UTM_fixedZone(double Lat, double Lon, char n[10], double &x, double &y)
{
	//initialize
	x = 0; y = 0;

	//converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
	//East Longitudes are positive, West longitudes are negative. 
	//North latitudes are positive, South latitudes are negative
	//Lat and Long are in decimal degrees
	//Written by Chuck Gantz- chuck.gantz@globalstar.com
	int ReferenceEllipsoid = 23;
	double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
	double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
	double k0 = 0.9996;

	double LongOrigin;
	double eccPrimeSquared;
	double N, T, C, A, M;

	//Make sure the longitude is between -180.00 .. 179.9
	double LongTemp = (Lon + 180) - int((Lon + 180) / 360) * 360 - 180; // -180.00 .. 179.9;

	double LatRad = Lat*deg2rad;
	double LongRad = LongTemp*deg2rad;
	double LongOriginRad;
	int    ZoneNumber;

	if (strlen(n) != 0)
	{
		char tempstr[10];
		int p = 0;
		for (p = 0; p < strlen(n); p++)
		{
			if (n[p] >= '0' && n[p] <= '9')
				tempstr[p] = n[p];
			else
				break;
		} // end for
		tempstr[p] = 0;

		ZoneNumber = atoi(tempstr);
	}
	else
		ZoneNumber = int((LongTemp + 180) / 6) + 1;

	if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
		ZoneNumber = 32;

	// Special zones for Svalbard
	if (Lat >= 72.0 && Lat < 84.0)
	{
		if (LongTemp >= 0.0  && LongTemp < 9.0) ZoneNumber = 31;
		else if (LongTemp >= 9.0  && LongTemp < 21.0) ZoneNumber = 33;
		else if (LongTemp >= 21.0 && LongTemp < 33.0) ZoneNumber = 35;
		else if (LongTemp >= 33.0 && LongTemp < 42.0) ZoneNumber = 37;
	}
	LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  //+3 puts origin in middle of zone
	LongOriginRad = LongOrigin * deg2rad;


	eccPrimeSquared = (eccSquared) / (1 - eccSquared);

	N = a / sqrt(1 - eccSquared*sin(LatRad)*sin(LatRad));
	T = tan(LatRad)*tan(LatRad);
	C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
	A = cos(LatRad)*(LongRad - LongOriginRad);

	M = a*((1 - eccSquared / 4 - 3 * eccSquared*eccSquared / 64 - 5 * eccSquared*eccSquared*eccSquared / 256)*LatRad
		- (3 * eccSquared / 8 + 3 * eccSquared*eccSquared / 32 + 45 * eccSquared*eccSquared*eccSquared / 1024)*sin(2 * LatRad)
		+ (15 * eccSquared*eccSquared / 256 + 45 * eccSquared*eccSquared*eccSquared / 1024)*sin(4 * LatRad)
		- (35 * eccSquared*eccSquared*eccSquared / 3072)*sin(6 * LatRad));

	double UTMEasting, UTMNorthing;
	UTMEasting = (double)(k0*N*(A + (1 - T + C)*A*A*A / 6
		+ (5 - 18 * T + T*T + 72 * C - 58 * eccPrimeSquared)*A*A*A*A*A / 120)
		+ 500000.0);

	UTMNorthing = (double)(k0*(M + N*tan(LatRad)*(A*A / 2 + (5 - T + 9 * C + 4 * C*C)*A*A*A*A / 24
		+ (61 - 58 * T + T*T + 600 * C - 330 * eccPrimeSquared)*A*A*A*A*A*A / 720)));
	if (Lat < 0)
		UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere

	x = UTMEasting;
	y = UTMNorthing;

	// 	double la, lo;
	// 	la = Lat; lo = Lon;
	// 
	// 	//ellipse parameters
	// 	double sa = 6378137.000000, sb = 6356752.314245;
	// 
	// 	double e2 = sqrt((sa * sa) - (sb * sb)) / sb;
	// 	double e2cuadrada = e2 * e2;
	// 	double c = (sa * sa) / sb;
	// 
	// 	double lat = la * (PI / 180);
	// 	double lon = lo * (PI / 180);
	// 
	// 	int Huso = (int)((lo / 6) + 31);
	// 	char Letra;
	// 	sscanf(n, "%d %c", &Huso, &Letra);
	// 	double S = ((Huso * 6) - 183);
	// 	double deltaS = lon - (S * (PI / 180));
	// 
	// // 	if (la < -72) Letra = 'C';
	// // 	else if (la < -64) Letra = 'D';
	// // 	else if (la < -56) Letra = 'E';
	// // 	else if (la < -48) Letra = 'F';
	// // 	else if (la < -40) Letra = 'G';
	// // 	else if (la < -32) Letra = 'H';
	// // 	else if (la < -24) Letra = 'J';
	// // 	else if (la < -16) Letra = 'K';
	// // 	else if (la < -8) Letra = 'L';
	// // 	else if (la < 0) Letra = 'M';
	// // 	else if (la < 8) Letra = 'N';
	// // 	else if (la < 16) Letra = 'P';
	// // 	else if (la < 24) Letra = 'Q';
	// // 	else if (la < 32) Letra = 'R';
	// // 	else if (la < 40) Letra = 'S';
	// // 	else if (la < 48) Letra = 'T';
	// // 	else if (la < 56) Letra = 'U';
	// // 	else if (la < 64) Letra = 'V';
	// // 	else if (la < 72) Letra = 'W';
	// // 	else Letra = 'X';
	// 
	// 	double a = cos(lat) * sin(deltaS);
	// 	double epsilon = 0.5 * log((1 + a) / (1 - a));
	// 	double nu = atan(tan(lat) / cos(deltaS)) - lat;
	// 	double v = (c / sqrt((1 + (e2cuadrada * cos(lat)*cos(lat))))) * 0.9996;
	// 	double ta = (e2cuadrada / 2) * epsilon * epsilon * cos(lat)*cos(lat);
	// 	double a1 = sin(2 * lat);
	// 	double a2 = a1 * cos(lat)*cos(lat);
	// 	double j2 = lat + (a1 / 2);
	// 	double j4 = ((3 * j2) + a2) / 4;
	// 	double j6 = ((5 * j4) + (a2 * cos(lat)*cos(lat))) / 3;
	// 	double alfa = (3.0 / 4.0) * e2cuadrada;
	// 	double beta = (5.0 / 3.0) * alfa * alfa;
	// 	double gama = (35.0 / 27.0) * alfa* alfa* alfa;
	// 	double Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
	// 	double xx = epsilon * v * (1 + (ta / 3)) + 500000;
	// 	double yy = nu * v * (1 + ta) + Bm;
	// 
	// 	if (yy < 0)
	// 		yy = 9999999 + yy;
	// 
	// 	x = xx;
	// 	y = yy;
	/*	sprintf(n, "%02d %c", Huso, Letra);*/

	return 1;
}

int CLL_UTM::UTM2LL(double gx, double gy, char n[10], double &Lat, double &Lon)
{
	//converts UTM coords to lat/long.  Equations from USGS Bulletin 1532 
	//East Longitudes are positive, West longitudes are negative. 
	//North latitudes are positive, South latitudes are negative
	//Lat and Long are in decimal degrees. 
	//Written by Chuck Gantz- chuck.gantz@globalstar.com

	int ReferenceEllipsoid = 23;

	double UTMEasting, UTMNorthing;
	UTMNorthing = gy;
	UTMEasting = gx;
	double k0 = 0.9996;
	double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
	double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
	double eccPrimeSquared;
	double e1 = (1 - sqrt(1 - eccSquared)) / (1 + sqrt(1 - eccSquared));
	double N1, T1, C1, R1, D, M;
	double LongOrigin;
	double mu, phi1, phi1Rad;
	double x, y;
	int ZoneNumber;
	char* ZoneLetter;
	int NorthernHemisphere; //1 for northern hemispher, 0 for southern

	x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
	y = UTMNorthing;

	ZoneNumber = strtoul(n, &ZoneLetter, 10);
	if ((*ZoneLetter - 'N') >= 0)
		NorthernHemisphere = 1;//point is in northern hemisphere
	else
	{
		NorthernHemisphere = 0;//point is in southern hemisphere
		y -= 10000000.0;//remove 10,000,000 meter offset used for southern hemisphere
	}

	LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  //+3 puts origin in middle of zone

	eccPrimeSquared = (eccSquared) / (1 - eccSquared);

	M = y / k0;
	mu = M / (a*(1 - eccSquared / 4 - 3 * eccSquared*eccSquared / 64 - 5 * eccSquared*eccSquared*eccSquared / 256));

	phi1Rad = mu + (3 * e1 / 2 - 27 * e1*e1*e1 / 32)*sin(2 * mu)
		+ (21 * e1*e1 / 16 - 55 * e1*e1*e1*e1 / 32)*sin(4 * mu)
		+ (151 * e1*e1*e1 / 96)*sin(6 * mu);
	phi1 = phi1Rad*rad2deg;

	N1 = a / sqrt(1 - eccSquared*sin(phi1Rad)*sin(phi1Rad));
	T1 = tan(phi1Rad)*tan(phi1Rad);
	C1 = eccPrimeSquared*cos(phi1Rad)*cos(phi1Rad);
	R1 = a*(1 - eccSquared) / pow(1 - eccSquared*sin(phi1Rad)*sin(phi1Rad), 1.5);
	D = x / (N1*k0);

	Lat = phi1Rad - (N1*tan(phi1Rad) / R1)*(D*D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1*C1 - 9 * eccPrimeSquared)*D*D*D*D / 24
		+ (61 + 90 * T1 + 298 * C1 + 45 * T1*T1 - 252 * eccPrimeSquared - 3 * C1*C1)*D*D*D*D*D*D / 720);
	Lat = Lat * rad2deg;

	Lon = (D - (1 + 2 * T1 + C1)*D*D*D / 6 + (5 - 2 * C1 + 28 * T1 - 3 * C1*C1 + 8 * eccPrimeSquared + 24 * T1*T1)
		*D*D*D*D*D / 120) / cos(phi1Rad);
	Lon = LongOrigin + Lon * rad2deg;

	// 	int zone;
	// 	char Letra;
	// 	sscanf(n, "%d %c", &zone, &Letra);
	// 
	// 	char hemis;
	// 	if (Letra>'M')
	// 		hemis = 'N';
	// 	else
	// 		hemis = 'S';
	// 
	// 	double sa = 6378137.000000; 
	// 	double sb = 6356752.314245;
	// 
	// 	double e2 = sqrt((sa * sa) - (sb * sb)) / sb;
	// 	double e2cuadrada = e2*e2;
	// 	double c = (sa *sa) / sb;
	// 
	// 	double X = x - 500000;
	// 	double Y;
	// 	if (hemis == 'S' || hemis == 's')
	// 		Y = y - 10000000;
	// 	else
	// 		Y = y;
	// 
	// 	double S = ((zone * 6) - 183);
	// 	double lat = Y / (6366197.724 * 0.9996);
	// 	double v = (c / (sqrt(1 + (e2cuadrada * cos(lat)*cos(lat))))) * 0.9996;
	// 	double a = X / v;
	// 	double a1 = sin(2 * lat);
	// 	double a2 = a1 * cos(lat)*cos(lat);
	// 	double j2 = lat + (a1 / 2);
	// 	double j4 = ((3 * j2) + a2) / 4;
	// 	double j6 = ((5 * j4) + (a2 * cos(lat)*cos(lat))) / 3;
	// 	double alfa = (3.0 / 4.0) * e2cuadrada;
	// 	double beta = (5.0 / 3.0) * alfa *alfa;
	// 	double gama = (35.0 / 27.0) * alfa *alfa*alfa;
	// 	double Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
	// 	double b = (Y - Bm) / v;
	// 	double Epsi = ((e2cuadrada * a * a) / 2) * cos(lat)*cos(lat);
	// 	double Eps = a * (1 - (Epsi / 3));
	// 	double nab = (b * (1 - Epsi)) + lat;
	// 	double senoheps = (exp(Eps) - exp(-Eps)) / 2;
	// 	double Delt = atan(senoheps / (cos(nab)));
	// 	double TaO = atan(cos(Delt) * tan(nab));
	// 	double longitude = (Delt *(180 / PI)) + S;
	// 	double latitude = (lat + (1 + e2cuadrada* (cos(lat)*cos(lat)) - (3 / 2) * e2cuadrada * sin(lat) * cos(lat) * (TaO - lat)) * (TaO - lat)) * (180 / PI);
	// 
	// 	Lat = latitude;
	// 	Lon = longitude;

	return 1;
}