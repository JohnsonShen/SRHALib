/*================================================================================*
 * O     O          __             ______  __   __  ____     __  ___          __  *
 *  \   /      /\  / /_      _    / /___/ / /  / / / __ \   / / /   \    /\  / /  *
 *   [+]      /  \/ / \\    //   / /____ / /  / /  \ \_    / / | | | |  /  \/ /   *
 *  /   \    / /\  /   \\__//   / /----// /__/ /  \ \__ \ / /  | | | | / /\  /    *
 * O     O  /_/  \/     \__/   /_/      \_ ___/    \___ //_/    \___/ /_/  \/     *
 *                                                                                *
 *                                                                                *
 * Nuvoton A.H.R.S Library for Cortex M4 Series                                   *
 *                                                                                *
 * Written by by T.L. Shen for Nuvoton Technology.                                *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                      *
 *                                                                                *
 *================================================================================*
 */
#ifndef __COMMON_H__
#define __COMMON_H__
#include <stdint.h>
#include "Gauss_Newton_Calibrate.h"
//#define DBG
#ifdef DBG
#ifndef DBG_PRINT
#define DBG_PRINT(S...)  printf(S)
#endif 
#else
#define DBG_PRINT(S...)
#endif
#define RIGHT 0
#define LEFT  1
#define MAX_SENSOR 5
#define MAX_AHRS   2
#define null 0
#define false 0
#define true 	1
#define bool unsigned char
#define M_PI 3.1415926535f
#define GYRO_DYNAMIC_CAL
#define ACC_DIM  3 
#define GYRO_DIM 3 
#define MAG_DIM  3
#define BARO_DIM 2 
#define HALL_DIM 2 
#define AHRDID_0 0
#define AHRDID_1 1
#define RAW_SMOOTH_BUFFER_SIZE (ACC_DIM+GYRO_DIM+MAG_DIM+BARO_DIM+HALL_DIM)
#define DOF      (ACC_DIM + GYRO_DIM + MAG_DIM + BARO_DIM)
#define GUARD	if(CheckSecurityID()) return;
extern char AHRSID;
typedef struct {
	float w;
	float x;
	float y;
	float z;
}Axis4f;
typedef struct {
	float 	AccMean[ACC_DIM];
	float 		AccScale[ACC_DIM];
	float 	  GyroMean[GYRO_DIM];
	float 		GyroScale[GYRO_DIM];
	float 		MagInvW[MAG_BETA_SIZE]; 
	float     AccG_PER_LSB;
	float 		GyroDegPLSB; 
	float 		MagGaussPLSB; 
	float 		MagCalQuality; 
	bool      RestoreRawSmooth[MAX_SENSOR];
} CalibrateInfo;
extern CalibrateInfo CalInfo[MAX_AHRS];
typedef struct {
	float     RingBuffer[DOF][RAW_SMOOTH_BUFFER_SIZE];	
	char			Index[MAX_SENSOR];
	bool			enable[MAX_SENSOR];
}RawSmooth_T;

typedef struct {
	bool  					beCalibrated[MAX_SENSOR];
	bool						beSensorEnable[MAX_SENSOR];
	bool  					beGyroSteady;
	bool 						beGyroSteadyPrev;
	float 					GyroDynamicCenter[GYRO_DIM];
	float     			RawACC[ACC_DIM];
	float     			RawGYRO[GYRO_DIM];
	float     			RawMAG[MAG_DIM];
	float           RawBARO[BARO_DIM];
	float           RawHALL[HALL_DIM];
	RawSmooth_T     RawSmooth;
} SensorStateInfo;
extern SensorStateInfo SensorState[MAX_AHRS];

typedef struct {
uint32_t lastTime;
uint32_t interval;
uint32_t currentTime;
uint32_t tick_counter;
} TimeFrameInfo;
extern TimeFrameInfo TimeInfo;

typedef struct {
float Euler[3];
float NormAttitude[3];
float NormACC[ACC_DIM];
float NormMAG[MAG_DIM];
float EulerNormMAG[3];
float	MAGHead[MAG_DIM];
float	GYRODeg[GYRO_DIM];
	
float CalACC[ACC_DIM];
float CalGYRO[GYRO_DIM];
float CalMAG[MAG_DIM];

} AttitudeInfo_T;
extern AttitudeInfo_T AttitudeInfo[MAX_AHRS];

float* GetCalibrateParams(int8_t sensorType);
float TimerRead(void);
void TimerStart(void);
void TimerSet(void);
float GetAHRSPeriod(void);
void SetZWithoutG_Base(float base);
void SetMagGuass(void);
float GetMagGuass(void);

uint32_t GetTickCounter(void);
int8_t AccCalibrate(int16_t* data_acc, int sample_count);
void AccZCalibrate(int16_t* data_acc, int sample_count);
int8_t MagCalibrate(int16_t* data_mag, int sample_count);
void BaroFilter(int16_t *raw);
int16_t kalmanFilter1(int16_t inData);
int16_t kalmanFilter2(int16_t inData);
void SetCalibrateACC(void);
void SetCalibrateGYRO(void);
void SetCalibrateMAG(void);

void sensfusion9UpdateQ(float gxf, float gyf, float gzf, float axf, float ayf, float azf, float mxf, float myf, float mzf, float dt);
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az);
void sensfusion6GetGyro(float* gyroOut);
void sensfusion6GetMagDegree(float* magdOut);
void sensfusion6GetMagEuler(float* magOut);
void sensfusion6GetNormMag(float* magOut);
void sensfusion6GetGyroDeg(float* gyroDegout);
void sensfusion6UpdateMagByEuler(void);
void sensfusion6GetVelocity(float* Velocity);
void sensfusion6GetMove(float* Move);
void sensfusion6ResetMove(void);
void sensfusion6SetMove(float* Move);

void AccFilterInit(void);
void imuAccIIRLPFilter(int16_t* in, int16_t* out);
void MultiwiiAccFilter(int16_t* in, int16_t* out);
#endif	//__COMMON_H__


