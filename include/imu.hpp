#ifndef IMU_HPP
#define IMU_HPP

//#define IMU_Golf_Update_Version		1

#define IMU_VER			"imu_fs, imu_property: 2022/08/13"
#define IMU_SNAME			"imu_fs"
#define CONV_GRAVITY		9.80665

#ifdef IMU_Golf_Update_Version
typedef struct {
	bool status;	// 1:true. 2:false
	// Raw data
	double angvel[ 3 ];
	double accel[ 3 ];
	double mag[ 3 ];
	double temperature;
	// 推定した角度
	double estAng[ 3 ];
} imu_fs;

typedef struct {
	double gravity;
	double initAng[ 2 ];	// 設置角度 (roll, pitch)
	double angvel_offset[ 3 ];	// 角速度のオフセット
	double K_Coef;	// 相補フィルターの係数
	
} imu_property;
#else

typedef struct {
	bool status;	// 1:true. 2:false
	// Raw data
	double angvel[ 3 ];
	double accel[ 3 ];
	double mag[ 3 ];
	double temperature;
	// 推定した角度
	double estAng[ 3 ];
	// グローバル座標における角速度
	double angvel_g[ 3 ];
} imu_fs;

typedef struct {
	char ver[ 256 ];
	double gravity;
	double initAng[ 2 ];			// 設置角度 (roll, pitch)
	double angvel_offset[ 3 ];	// 角速度のオフセット
	double K_Coef;				// 相補フィルターの係数
} imu_property;
#endif

#endif // IMU_HPP
