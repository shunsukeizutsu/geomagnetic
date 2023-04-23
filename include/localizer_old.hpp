#ifndef LOCALIZER_HPP
#define LOCALIZER_HPP

#define LOCALIZER_SNAME "localizer"
#define LOCALIZER_INFO_SNAME "localizer_info"

typedef struct
{
	bool status;
	double estPos[ 3 ];		// 推定した位置
	double estAng[ 3 ];		// 推定した方位

	double estVel[ 3 ];
	double estVel_xy;
	double estAngvel[ 3 ];

	bool dir;		// 前後退のフラグに使用(前進:true, 後退:false)
	double imu_offset;
	double estYaw_gnss;

} localizer;

#endif
