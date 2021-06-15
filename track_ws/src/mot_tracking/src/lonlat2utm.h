#ifndef LONLAT2UTM_H
#define LONLAT2UTM_H

double PI = 3.1415926;
/**
 * @brief WGS84 TO UTM 
 * @details https://blog.csdn.net/bobsweetie/article/details/106824071
 * @param  longitude
 *         latitude
 *         UTME : east for x or y axis (resolution is meter)
 *         UTMN : north for x or y axis (resolution is meter)
 */
inline void LonLat2UTM(double longitude, double latitude, double& UTME, double& UTMN) {
	double lat = latitude;
	double lon = longitude;

	double kD2R = PI / 180.0;
	double ZoneNumber = floor((lon - 1.5) / 3.0) + 1;
	double L0 = ZoneNumber * 3.0;

	double a = 6378137.0;
	double F = 298.257223563;
	double f = 1 / F;
	double b = a * (1 - f);
	double ee = (a * a - b * b) / (a * a);
	double e2 = (a * a - b * b) / (b * b);
	double n = (a - b) / (a + b);
	double n2 = (n * n);
	double n3 = (n2 * n);
	double n4 = (n2 * n2);
	double n5 = (n4 * n);
	double al = (a + b) * (1 + n2 / 4 + n4 / 64) / 2.0;
	double bt = -3 * n / 2 + 9 * n3 / 16 - 3 * n5 / 32.0;
	double gm = 15 * n2 / 16 - 15 * n4 / 32;
	double dt = -35 * n3 / 48 + 105 * n5 / 256;
	double ep = 315 * n4 / 512;
	double B = lat * kD2R;
	double L = lon * kD2R;
	L0 = L0 * kD2R;
	double l = L - L0;
	double cl = (cos(B) * l);
	double cl2 = (cl * cl);
	double cl3 = (cl2 * cl);
	double cl4 = (cl2 * cl2);
	double cl5 = (cl4 * cl);
	double cl6 = (cl5 * cl);
	double cl7 = (cl6 * cl);
	double cl8 = (cl4 * cl4);
	double lB = al
			* (B + bt * sin(2 * B) + gm * sin(4 * B) + dt * sin(6 * B)
					+ ep * sin(8 * B));
	double t = tan(B);
	double t2 = (t * t);
	double t4 = (t2 * t2);
	double t6 = (t4 * t2);
	double Nn = a / sqrt(1 - ee * sin(B) * sin(B));
	double yt = e2 * cos(B) * cos(B);
	double N = lB;
	N = N + t * Nn * cl2 / 2;
	N = N + t * Nn * cl4 * (5 - t2 + 9 * yt + 4 * yt * yt) / 24;
	N = N + t * Nn * cl6 * (61 - 58 * t2 + t4 + 270 * yt - 330 * t2 * yt) / 720;
	N = N + t * Nn * cl8 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;
	double E = Nn * cl;
	E = E + Nn * cl3 * (1 - t2 + yt) / 6;
	E = E + Nn * cl5 * (5 - 18 * t2 + t4 + 14 * yt - 58 * t2 * yt) / 120;
	E = E + Nn * cl7 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;
	E = E + 500000;
	N = 0.9996 * N;
	E = 0.9996 * (E - 500000.0) + 500000.0;

	UTME = E;
	UTMN = N;
}


#endif

