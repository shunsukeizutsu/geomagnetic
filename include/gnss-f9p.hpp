#ifndef _GNSS_F9P_HPP_
#define _GNSS_F9P_HPP_

//#define GNSS_Golf_Update_Version		1

#define GNSS_VER			"gnss_gl, gnss_property: 2022/08/13"
#define GNSS_SNAME		"RTK-GNSS-F9P"

#define _UNKNOWN			0
#define _SINGLE			1
#define _DGPS				2
#define _FIX				4
#define _FLOAT			5

#ifdef GNSS_Golf_Update_Version
typedef struct
{
	double x;
	double y;
	double z;
} my_position_t;

typedef struct
{
	bool status;
	double utc_time;
	
	double latitude;	// 緯度(deg)
	double longitude;	// 経度(deg)
	double height;		// 楕円対高(m)
	int posStatus;		// FIX or FLOAT, etc
	double elevation;	// 標高(m)
	double Geoid_height;	// ジオイド高(m)
	my_position_t enu;
	my_position_t ecef;
	
} gnss_gl;
#else

typedef struct
{
	double utc_time;
	double latitude;		// 緯度(deg)
	double longitude;		// 経度(deg)
	double elevation;		// 標高(m)
	double Geoid_height;	// ジオイド高(m)
	int posStatus;		// FIX or FLOAT, etc
	int nSatellite;		// 使用衛生数
	double hdop;			// Horizontal Dilution Of Precision (水平精度低下率):数値が大きいほど精度が低い
	
	bool status;
	double estGeoid;		// 推定したジオイド高(m)
	double enz[ 3 ];		// 平面直角座標
} gnss_gl;

typedef struct {
	char ver[ 256 ];
	int coordinate_id;
} gnss_property;
#endif

#endif
